// The interleaved readout: science rows alternating with a guide window. The
// commands that set it up, the controller writes made ahead of each trigger, and
// putting the previous readout geometry back when the readout is left. The
// consumer splits its frames in hispec_tracking_camera_exposure_modes.cpp.
#include "hispec_tracking_camera_instrument.h"

#include <charconv>
#include <chrono>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

namespace Camera {

  namespace {
    // H2RG window bounds: each register takes its base address plus a pixel index
    constexpr int H2RG_VSTART_REGISTER = 32768;  // 1000 000000000000
    constexpr int H2RG_VSTOP_REGISTER  = 36864;  // 1001 000000000000
    constexpr int H2RG_HSTART_REGISTER = 40960;  // 1010 000000000000
    constexpr int H2RG_HSTOP_REGISTER  = 45056;  // 1011 000000000000

    // Each write restarts the VCPU, which sends the data register's word to
    // the H2RG if it finds the trigger register set. Level, not edge, so a
    // write repeated after a refusal still lands exactly as intended.
    constexpr int VCPU_DATA_INREG    = 1;
    constexpr int VCPU_TRIGGER_INREG = 0;

    // How long a register write waits out a frame fetch on the same socket.
    // Far longer than fetching the largest frame, so it only expires on a
    // stalled readout; the poll is coarse because each refusal is logged.
    constexpr auto INREG_BUSY_WAIT = std::chrono::milliseconds(2000);
    constexpr auto INREG_BUSY_POLL = std::chrono::milliseconds(5);

    // The interleaved science window spans two amplifiers; the reference
    // channel rides along on a third
    constexpr int INTERLEAVE_SCIENCE_AMPS = 2;

    constexpr int H2RG_BITS_PER_PIXEL = 16;

    // Whole tokens, all or nothing. Run ahead of validate_roi(), whose std::stoi
    // reads "12x" as 12 and throws on a value too large for an int.
    std::optional<std::vector<int>> parse_ints(const std::vector<std::string> &tokens) {
      std::vector<int> values;
      for (const auto &token : tokens) {
        int value = 0;
        const char* end = token.data() + token.size();
        const auto [stop, status] = std::from_chars(token.data(), end, value);
        if (status != std::errc{} || stop != end) return std::nullopt;
        values.push_back(value);
      }
      return values;
    }
  }


  /***** Camera::HispecTrackingCamera::parse_window **************************/
  /**
   * @brief      parse the <count> integers of a window command and check them as roi does
   * @return     the values, or nothing once the reason is logged and in retstring
   */
  std::optional<std::vector<int>> HispecTrackingCamera::parse_window(const std::string &function,
                                                                     const std::string &args, size_t count,
                                                                     const std::string &usage,
                                                                     std::string &retstring) {
    std::vector<std::string> tokens;
    Tokenize(args, tokens, " ");
    if (tokens.size() != count) {
      this->fail_detailed(function, retstring, "wrong argument count",
                          "got "+std::to_string(tokens.size())+" argument(s) \""+args+"\"; expected "+usage);
      return std::nullopt;
    }
    auto values = parse_ints(tokens);
    if (!values) {
      this->fail_detailed(function, retstring, "arguments not numeric",
                          "could not parse \""+args+"\" as integers "+usage);
      return std::nullopt;
    }
    if (this->validate_roi(args, retstring) != NO_ERROR) return std::nullopt;
    return values;
  }
  /***** Camera::HispecTrackingCamera::parse_window **************************/


  /***** Camera::HispecTrackingCamera::interleave_science ********************/
  /**
   * @brief      set or report the science window of the interleaved readout
   * @details    Only records the window; it reaches the controller at the next
   *             expose, so the interleave commands can come in any order.
   */
  long HispecTrackingCamera::interleave_science(const std::string &args, std::string &retstring) {
    const std::string function("Camera::HispecTrackingCamera::interleave_science");

    if (args=="?" || args=="help") {
      retstring = "interleave_science [ <height> <width> ]\n";
      retstring.append( "  Set or report the science window of the interleaved readout, centred\n" );
      retstring.append( "  like \"roi <height> <width>\". <width> must be even and <height> a\n" );
      retstring.append( "  multiple of interleave_set. Applied at the next expose. Starts as 128 128.\n" );
      return HELP;
    }

    const std::lock_guard<std::mutex> lock(this->interleave_mutex);
    auto &geometry = this->interleave_geometry;

    if (!args.empty()) {
      if (this->is_interleave_frozen()) {
        return this->fail_detailed(function, retstring, "exposure in progress",
                   "the science window is fixed while an interleaved exposure runs; "
                   "change it once the exposure has finished");
      }
      const auto values = this->parse_window(function, args, 2, "<height> <width>", retstring);
      if (!values) return ERROR;
      const int height = (*values)[0];
      const int width  = (*values)[1];
      if (width % INTERLEAVE_SCIENCE_AMPS != 0) {
        return this->fail_detailed(function, retstring, "science width must be even",
                   "width="+std::to_string(width)+" cannot be split evenly across "+
                   std::to_string(INTERLEAVE_SCIENCE_AMPS)+" amplifiers");
      }
      geometry.science_rows           = height;
      geometry.science_pixels_per_tap = width / INTERLEAVE_SCIENCE_AMPS;
      geometry.science_start_row      = (this->detector_rows() - height) / 2;
    }

    retstring = std::to_string(geometry.science_rows) + " " +
                std::to_string(geometry.science_pixels_per_tap * INTERLEAVE_SCIENCE_AMPS);
    return NO_ERROR;
  }
  /***** Camera::HispecTrackingCamera::interleave_science ********************/


  /***** Camera::HispecTrackingCamera::send_inreg_offstream ******************/
  /**
   * @brief      write a VCPU input register clear of the frame traffic
   * @details    The primary connection is shared with the acquisition thread,
   *             so a write there can fail in two ways while frames are moving.
   *             A frame fetch in flight leaves the Archon busy, which is waited
   *             out. Pushed autofetch frames bury the reply altogether, so then
   *             the write goes over the control connection, which carries none.
   * @return     ERROR|NO_ERROR|BUSY
   */
  long HispecTrackingCamera::send_inreg_offstream(int module, int inreg, int value) {
    if (!this->is_autofetch_mode) {
      const auto deadline = std::chrono::steady_clock::now() + INREG_BUSY_WAIT;
      long error = this->send_inreg(module, inreg, value);
      while (error == BUSY && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(INREG_BUSY_POLL);
        error = this->send_inreg(module, inreg, value);
      }
      return error;
    }

    const std::string key = "MOD" + std::to_string(module) + "/VCPU_INREG" + std::to_string(inreg);
    const auto entry = this->controller->configmap.find(key);
    if (entry == this->controller->configmap.end()) return ERROR;

    // The same WCONFIG and APPLYDIO pair that set_vcpu_inreg() sends
    std::ostringstream wconfig;
    wconfig << "WCONFIG" << std::uppercase << std::setfill('0') << std::setw(4) << std::hex
            << entry->second.line << key << "=" << std::to_string(value);
    std::ostringstream applydio;
    applydio << "APPLYDIO" << std::uppercase << std::setfill('0') << std::setw(2) << std::hex
             << (module - 1);

    long error = this->controller->send_control_cmd(wconfig.str());
    // write_config_key() skips a value it believes the controller already holds
    if (error == NO_ERROR) entry->second.value = std::to_string(value);
    if (error == NO_ERROR) error = this->controller->send_control_cmd(applydio.str());
    return error;
  }
  /***** Camera::HispecTrackingCamera::send_inreg_offstream ******************/


  /***** Camera::HispecTrackingCamera::write_guide_window ********************/
  /**
   * @brief      write the H2RG window bound registers, stopping at the first failure
   * @details    Sets where the window is and nothing else: the firmware decides
   *             when the detector reads it.
   * @return     ERROR|NO_ERROR|BUSY
   */
  long HispecTrackingCamera::write_guide_window(int vstart, int vstop, int hstart, int hstop) {
    for (const int value : { H2RG_VSTART_REGISTER + vstart, H2RG_VSTOP_REGISTER + vstop,
                             H2RG_HSTART_REGISTER + hstart, H2RG_HSTOP_REGISTER + hstop }) {
      long error = this->send_inreg_offstream(this->lvds_module, VCPU_DATA_INREG, value);
      if (error == NO_ERROR) error = this->send_inreg_offstream(this->lvds_module, VCPU_TRIGGER_INREG, 1);
      if (error == NO_ERROR) error = this->send_inreg_offstream(this->lvds_module, VCPU_TRIGGER_INREG, 0);
      if (error != NO_ERROR) return error;
    }
    return NO_ERROR;
  }
  /***** Camera::HispecTrackingCamera::write_guide_window ********************/


  /***** Camera::HispecTrackingCamera::interleave_guide **********************/
  /**
   * @brief      set, move or report the guide window of the interleaved readout
   * @details    Writes the bound registers "roi <vstart> <vstop> <hstart> <hstop>"
   *             does, but not the window mode register: the firmware switches
   *             the detector into window mode for each guide read.
   *
   *             Safe while frames are being read, so the window can follow a
   *             star during an exposure: the writes keep clear of the frame
   *             traffic, one mover runs at a time, and a move that fails part
   *             way puts the previous window back.
   */
  long HispecTrackingCamera::interleave_guide(const std::string &args, std::string &retstring) {
    const std::string function("Camera::HispecTrackingCamera::interleave_guide");

    if (args=="?" || args=="help") {
      retstring = "interleave_guide [ <vstart> <vstop> <hstart> <hstop> ]\n";
      retstring.append( "  Set, move or report the guide window of the interleaved readout, in\n" );
      retstring.append( "  detector pixels {0:2047}, inclusive. Needs the detector powered. Can\n" );
      retstring.append( "  move, but not resize, during an exposure. Its width must fit in the\n" );
      retstring.append( "  science width per amplifier. Starts as 10x10 at the detector centre.\n" );
      return HELP;
    }

    // One mover at a time: two register sequences interleaved on the wire
    // would leave the detector holding parts of both windows
    const std::lock_guard<std::mutex> lock(this->interleave_mutex);
    auto &geometry = this->interleave_geometry;

    if (!args.empty()) {
      const auto values = this->parse_window(function, args, 4, "<vstart> <vstop> <hstart> <hstop>", retstring);
      if (!values) return ERROR;

      // Checked before anything is written: the Archon accepts these writes
      // on behalf of a detector that cannot receive them
      if (!this->controller->is_connected) {
        return this->fail_detailed(function, retstring, "not connected",
                   "no socket is open to the Archon; run \"open\" first");
      }
      if (!this->controller->is_firmwareloaded) {
        return this->fail_detailed(function, retstring, "firmware not loaded",
                   "the register writes go through the ACF's configuration; run \"load\" first");
      }
      if (!this->controller->is_powered) {
        return this->fail_detailed(function, retstring, "power is off",
                   "an unpowered detector does not hold its registers; run "
                   "\"power on\", then send this command again");
      }

      const int vstart = (*values)[0];
      const int vstop  = (*values)[1];
      const int hstart = (*values)[2];
      const int hstop  = (*values)[3];

      // The sequencer and the CDS were given this window's size at the trigger
      if (this->is_interleave_frozen() &&
          (vstop - vstart != geometry.guide_vstop - geometry.guide_vstart ||
           hstop - hstart != geometry.guide_hstop - geometry.guide_hstart)) {
        return this->fail_detailed(function, retstring, "guide window size is fixed",
                   "an interleaved exposure is running with a "+
                   std::to_string(geometry.guide_rows())+"x"+std::to_string(geometry.guide_cols())+
                   " guide window; it can move, but not change size until the exposure has finished");
      }

      // Odd from here until the registers settle, so a frame read out meanwhile
      // can be told apart from the ones around it
      ++this->guide_changes;
      struct MoveFinished {
        std::atomic<uint64_t> &changes;
        ~MoveFinished() { ++changes; }
      } finished{this->guide_changes};

      if (this->write_guide_window(vstart, vstop, hstart, hstop) != NO_ERROR) {
        // Put the old window back so the detector and this record still agree
        const bool restored =
            this->write_guide_window(geometry.guide_vstart, geometry.guide_vstop,
                                     geometry.guide_hstart, geometry.guide_hstop) == NO_ERROR;
        return this->fail_detailed(function, retstring, "guide window not set",
                   "a register write to LVDS module "+std::to_string(this->lvds_module)+
                   " failed; "+(restored ? "the previous window was restored"
                                         : "the detector may hold parts of both windows, "
                                           "send the command again"));
      }

      const std::lock_guard<std::mutex> published(this->guide_position_mutex);
      geometry.guide_vstart = vstart;
      geometry.guide_vstop  = vstop;
      geometry.guide_hstart = hstart;
      geometry.guide_hstop  = hstop;
    }

    retstring = std::to_string(geometry.guide_vstart) + " " + std::to_string(geometry.guide_vstop) + " " +
                std::to_string(geometry.guide_hstart) + " " + std::to_string(geometry.guide_hstop);
    return NO_ERROR;
  }
  /***** Camera::HispecTrackingCamera::interleave_guide **********************/


  /***** Camera::HispecTrackingCamera::guide_position ************************/
  GuidePosition HispecTrackingCamera::guide_position() const {
    const std::lock_guard<std::mutex> lock(this->guide_position_mutex);
    const auto &geometry = this->interleave_geometry;
    return { geometry.guide_vstart, geometry.guide_vstop, geometry.guide_hstart,
             geometry.guide_hstop, this->guide_changes.load() };
  }
  /***** Camera::HispecTrackingCamera::guide_position ************************/


  /***** Camera::HispecTrackingCamera::interleave_set ************************/
  /**
   * @brief      set or report the science rows read before each guide window
   * @details    Checked against the science window height at the next expose.
   */
  long HispecTrackingCamera::interleave_set(const std::string &args, std::string &retstring) {
    const std::string function("Camera::HispecTrackingCamera::interleave_set");

    if (args=="?" || args=="help") {
      retstring = "interleave_set [ <rows> ]\n";
      retstring.append( "  Set or report the science rows read before each guide window. Applied\n" );
      retstring.append( "  at the next expose. Starts as 4.\n" );
      return HELP;
    }

    const std::lock_guard<std::mutex> lock(this->interleave_mutex);

    if (!args.empty()) {
      if (this->is_interleave_frozen()) {
        return this->fail_detailed(function, retstring, "exposure in progress",
                   "the set size is fixed while an interleaved exposure runs; "
                   "change it once the exposure has finished");
      }
      std::vector<std::string> tokens;
      Tokenize(args, tokens, " \t");
      const auto rows = tokens.size() == 1 ? parse_ints(tokens) : std::nullopt;
      if (!rows || rows->front() < 1 || rows->front() > this->detector_rows()) {
        return this->fail_detailed(function, retstring, "invalid row count",
                   "\""+args+"\" must be an integer in {1:"+std::to_string(this->detector_rows())+"}");
      }
      this->interleave_geometry.rows_per_set = rows->front();
    }

    retstring = std::to_string(this->interleave_geometry.rows_per_set);
    return NO_ERROR;
  }
  /***** Camera::HispecTrackingCamera::interleave_set ************************/


  /***** Camera::HispecTrackingCamera::apply_interleave **********************/
  /**
   * @brief      send the interleaved readout's settings to the controller
   * @details    Called by expose() ahead of the trigger, so what the interleave
   *             commands recorded reaches the Archon as one consistent set
   *             whatever order they came in, after checking the rules that span
   *             more than one command. The guide window is written every time,
   *             because the detector forgets it on a power cycle or a reset.
   */
  long HispecTrackingCamera::apply_interleave(std::string &retstring) {
    const std::string function("Camera::HispecTrackingCamera::apply_interleave");

    // Held throughout, so a guide window move cannot land between these writes
    const std::lock_guard<std::mutex> lock(this->interleave_mutex);
    const auto &geometry = this->interleave_geometry;

    if (geometry.science_rows % geometry.rows_per_set != 0) {
      return this->fail_detailed(function, retstring, "science rows not a multiple of the set size",
                 std::to_string(geometry.science_rows)+" science rows cannot be read "+
                 std::to_string(geometry.rows_per_set)+" at a time without a remainder; "
                 "change interleave_science or interleave_set");
    }
    if (geometry.guide_cols() > geometry.science_pixels_per_tap) {
      return this->fail_detailed(function, retstring, "guide window too wide",
                 std::to_string(geometry.guide_cols())+" guide columns do not fit in the "+
                 std::to_string(geometry.science_pixels_per_tap)+" pixels each amplifier "
                 "reads per line; narrow interleave_guide or widen interleave_science");
    }

    const auto selected = this->controller->modemap.find(this->controller->selectedmode);
    if (selected == this->controller->modemap.end() || selected->second.tapinfo.num_taps < 1) {
      return this->fail_detailed(function, retstring, "no camera mode selected",
                 "the interleaved frame is laid out across the camera mode's taplines; "
                 "run \"mode <name>\" first");
    }
    auto &mode = selected->second;

    // Only the first exposure saves: later ones would save the interleaved geometry itself
    if (!this->pre_interleave_readout) {
      this->pre_interleave_readout = SavedReadout{
        this->controller->selectedmode, mode.geometry.pixelcount, mode.geometry.linecount,
        this->current_h2rg_window(), this->camera_info.region_of_interest,
        this->camera_info.detector_pixels};
    }

    const int pixelcount = geometry.science_pixels_per_tap;
    const int linecount  = geometry.lines_per_frame();

    if (this->clear_controller_abort(retstring) != NO_ERROR) return ERROR;

    if (this->set_parameter_offstream("H2RG_columns", pixelcount)               != NO_ERROR ||
        this->set_parameter_offstream("interleave_set", geometry.rows_per_set)  != NO_ERROR ||
        this->set_parameter_offstream("interleave_rows", geometry.guide_rows()) != NO_ERROR ||
        this->set_parameter_offstream("interleave_cols", geometry.guide_cols()) != NO_ERROR) {
      return this->fail_detailed(function, retstring, "interleave not applied",
                 "the controller rejected H2RG_columns, interleave_set, interleave_rows "
                 "or interleave_cols; check that all four exist in the loaded ACF");
    }

    // One Archon frame holds a set: its science lines, then the guide window's
    if (this->write_cds_geometry(pixelcount, linecount) != NO_ERROR) {
      return this->fail_detailed(function, retstring, "interleave not applied",
                 "the controller rejected the CDS geometry "+std::to_string(pixelcount)+"x"+
                 std::to_string(linecount)+" (PIXELCOUNT x LINECOUNT)");
    }

    // That frame is not a region of the detector, so the host geometry
    // describes the frame itself. The autofetch reader checks each frame's
    // row count against it.
    const auto width  = static_cast<uint32_t>(pixelcount * mode.tapinfo.num_taps);
    const auto height = static_cast<uint32_t>(linecount);
    mode.geometry.pixelcount = pixelcount;
    mode.geometry.linecount  = linecount;
    this->camera_info.region_of_interest = {0, width - 1, 0, height - 1};
    this->camera_info.detector_pixels    = {width, height};
    this->camera_info.set_axes(H2RG_BITS_PER_PIXEL);

    if (this->write_guide_window(geometry.guide_vstart, geometry.guide_vstop,
                                 geometry.guide_hstart, geometry.guide_hstop) != NO_ERROR) {
      return this->fail_detailed(function, retstring, "guide window not written",
                 "a register write to LVDS module "+std::to_string(this->lvds_module)+
                 " failed, so the detector may not hold the guide window");
    }

    logwrite(function, "science "+std::to_string(geometry.science_rows)+" rows from row "+
                       std::to_string(geometry.science_start_row)+", "+
                       std::to_string(geometry.rows_per_set)+" per set, "+
                       std::to_string(geometry.sets_per_pass())+" sets per pass; guide "+
                       std::to_string(geometry.guide_rows())+"x"+std::to_string(geometry.guide_cols())+
                       "; frame "+std::to_string(width)+"x"+std::to_string(height));
    return NO_ERROR;
  }
  /***** Camera::HispecTrackingCamera::apply_interleave **********************/


  /***** Camera::HispecTrackingCamera::arm_interleave_pass ********************/
  /**
   * @details    Called by the acquisition thread ahead of every trigger. Reads
   *             only settings that are frozen while an exposure runs, so it
   *             takes no lock and never waits on a guide window move.
   */
  long HispecTrackingCamera::arm_interleave_pass() {
    const auto &geometry = this->interleave_geometry;
    long error = this->set_parameter_offstream("H2RG_rows_skip", geometry.science_start_row);
    if (error == NO_ERROR) error = this->set_parameter_offstream("interleave_nsets", geometry.sets_per_pass());
    return error;
  }
  /***** Camera::HispecTrackingCamera::arm_interleave_pass ********************/


  /***** Camera::HispecTrackingCamera::record_h2rg_window *********************/
  /**
   * @brief      note the H2RG window a command has just written
   * @details    Called before that command sets its geometry, which supersedes
   *             any saved before an interleaved exposure.
   */
  void HispecTrackingCamera::record_h2rg_window(int columns, int rows_skip) {
    this->discard_pre_interleave_readout();
    const std::lock_guard<std::mutex> lock(this->interleave_mutex);
    this->h2rg_window = H2rgWindow{columns, rows_skip};
  }
  /***** Camera::HispecTrackingCamera::record_h2rg_window *********************/


  /***** Camera::HispecTrackingCamera::put_back_mode_geometry *****************/
  // Caller holds interleave_mutex
  void HispecTrackingCamera::put_back_mode_geometry(const SavedReadout &saved) {
    const auto mode = this->controller->modemap.find(saved.camera_mode);
    if (mode == this->controller->modemap.end()) return;
    mode->second.geometry.pixelcount = saved.pixelcount;
    mode->second.geometry.linecount  = saved.linecount;
  }
  /***** Camera::HispecTrackingCamera::put_back_mode_geometry *****************/


  /***** Camera::HispecTrackingCamera::discard_pre_interleave_readout *********/
  /**
   * @brief      drop the saved geometry for a command that sets its own
   * @details    The camera mode's stored geometry is still put back: commands
   *             such as mode read it, and would otherwise push the interleaved
   *             frame shape. Call before the command sets its geometry.
   */
  void HispecTrackingCamera::discard_pre_interleave_readout() {
    const std::lock_guard<std::mutex> lock(this->interleave_mutex);
    if (!this->pre_interleave_readout) return;
    this->put_back_mode_geometry(*this->pre_interleave_readout);
    this->pre_interleave_readout.reset();
  }
  /***** Camera::HispecTrackingCamera::discard_pre_interleave_readout *********/


  /***** Camera::HispecTrackingCamera::current_h2rg_window ********************/
  /**
   * @details    Caller holds interleave_mutex. Before any command has written
   *             the window, the controller still holds the ACF's values.
   */
  std::optional<HispecTrackingCamera::H2rgWindow> HispecTrackingCamera::current_h2rg_window() const {
    if (this->h2rg_window) return this->h2rg_window;
    const auto acf_value = [this](const std::string &name) {
      const auto param = this->controller->parammap.find(name);
      return param == this->controller->parammap.end() ? std::string() : param->second.value;
    };
    const auto values = parse_ints({acf_value("H2RG_columns"), acf_value("H2RG_rows_skip")});
    if (!values) return std::nullopt;
    return H2rgWindow{(*values)[0], (*values)[1]};
  }
  /***** Camera::HispecTrackingCamera::current_h2rg_window ********************/


  /***** Camera::HispecTrackingCamera::restore_pre_interleave_readout *********/
  /**
   * @brief      put back what apply_interleave() overwrote
   * @details    On failure the saved geometry is kept, so leaving the readout
   *             can be tried again.
   */
  long HispecTrackingCamera::restore_pre_interleave_readout(std::string &retstring) {
    const std::string function("Camera::HispecTrackingCamera::restore_pre_interleave_readout");
    const std::lock_guard<std::mutex> lock(this->interleave_mutex);
    if (!this->pre_interleave_readout) return NO_ERROR;
    const SavedReadout &saved = *this->pre_interleave_readout;
    this->put_back_mode_geometry(saved);

    // A mode switch that failed part way leaves another mode selected, which
    // the saved controller geometry does not belong to
    if (saved.camera_mode != this->controller->selectedmode ||
        this->controller->modemap.find(saved.camera_mode) == this->controller->modemap.end()) {
      logwrite(function, "WARNING camera mode "+saved.camera_mode+" is no longer selected, so at most "
                         "its stored geometry was restored; run mode or roi before the next exposure");
      this->pre_interleave_readout.reset();
      return NO_ERROR;
    }

    if (saved.h2rg_window) {
      if (this->set_parameter_offstream("H2RG_columns", saved.h2rg_window->columns)     != NO_ERROR ||
          this->set_parameter_offstream("H2RG_rows_skip", saved.h2rg_window->rows_skip) != NO_ERROR) {
        return this->fail_detailed(function, retstring, "readout geometry not restored",
                   "the controller rejected H2RG_columns or H2RG_rows_skip");
      }
    }
    else {
      logwrite(function, "WARNING H2RG_columns and H2RG_rows_skip were never written and the "
                         "ACF has no value for them, so they were not restored; run mode or roi "
                         "before the next exposure");
    }

    if (this->write_cds_geometry(saved.pixelcount, saved.linecount) != NO_ERROR) {
      return this->fail_detailed(function, retstring, "readout geometry not restored",
                 "the controller rejected the CDS geometry "+std::to_string(saved.pixelcount)+"x"+
                 std::to_string(saved.linecount)+" (PIXELCOUNT x LINECOUNT)");
    }

    this->camera_info.region_of_interest = saved.region_of_interest;
    this->camera_info.detector_pixels    = saved.detector_pixels;
    this->camera_info.set_axes(H2RG_BITS_PER_PIXEL);

    logwrite(function, "restored "+saved.camera_mode+" geometry "+std::to_string(saved.pixelcount)+"x"+
                       std::to_string(saved.linecount)+" after the interleaved readout");
    this->pre_interleave_readout.reset();
    return NO_ERROR;
  }
  /***** Camera::HispecTrackingCamera::restore_pre_interleave_readout *********/

}
