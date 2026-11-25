#include "hispec_tracking_camera_instrument.h"
#include "hispec_tracking_camera_exposure_modes.h"
#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iterator>
#include <mutex>
#include <optional>
#include <sstream>
#include <thread>
#include <vector>
#include <sched.h>
#include <string_view>
#include <unistd.h>

namespace Camera {

  namespace {
    // Archon parameters are 20 bits, and prep_parameter() throws above this
    constexpr int ARCHON_PARAM_MAX = 0xFFFFF;

    // Set by the ACF's CDS timing (SHD2=700 ticks of 10ns); measured 7.07 us/pixel
    constexpr double DEFAULT_PIXEL_TIME_USEC = 7.0;

    // Covers per-row and per-frame overhead the pixel count does not model.
    // Generous on purpose: an overrun aborts a good acquisition, an oversized
    // deadline only delays reporting a failure.
    constexpr double DEFAULT_READOUT_MARGIN_MSEC = 5000.0;

    // Pixel period, a sequencer parameter a mode section can override.
    // Preferred over the config file: the ACF owns this timing and camerad
    // cannot edit the ACF.
    constexpr const char* ACF_PIXEL_PERIOD_PARAM = "tpix";

    // End of the CDS signal sample window, short of the full pixel period;
    // for an ACF without tpix
    constexpr const char* ACF_PIXEL_TIME_KEY = "SHD2";

    // Archon timing is counted in 10 ns ticks
    constexpr double ARCHON_TICK_USEC = 0.01;

    constexpr double USEC_PER_SEC = 1.0e6;
    constexpr double MSEC_PER_SEC = 1.0e3;

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

    constexpr std::string_view INTERLEAVE_READOUT = "interleave";

    // Reads every pixel twice, so its CDS line is twice the detector's
    constexpr std::string_view RXR_READOUT = "rxr";
    constexpr int RXR_SAMPLES_PER_PIXEL = 2;

    constexpr int H2RG_BITS_PER_PIXEL = 16;

    std::optional<double> positive_ticks(const std::string &text) {
      try {
        const double ticks = std::stod(text);
        if (ticks > 0) return ticks;
      }
      catch (const std::exception &) { }  // unparsable, so the next source decides
      return std::nullopt;
    }

    constexpr std::string_view ANY_CPU_TOKEN = "any";
    constexpr std::string_view WHITESPACE = " \t";

    std::string_view trim(std::string_view text) {
      const auto first = text.find_first_not_of(WHITESPACE);
      if (first == std::string_view::npos) return {};
      return text.substr(first, text.find_last_not_of(WHITESPACE) - first + 1);
    }

    // Whole-token integer: std::stoi would read "4x" as 4
    std::optional<int> parse_int(std::string_view text) {
      int value = 0;
      const char* end = text.data() + text.size();
      const auto [stop, status] = std::from_chars(text.data(), end, value);
      if (status != std::errc{} || stop != end) return std::nullopt;
      return value;
    }

    // A misspelt value fails startup, where the alternative is to publish
    // something other than what the .cfg asked for without saying so
    InterleaveOutput parse_interleave_output(const std::string &arg) {
      if (arg == "raw")   return InterleaveOutput::Raw;
      if (arg == "split") return InterleaveOutput::Split;
      if (arg == "guide") return InterleaveOutput::GuideOnly;
      throw std::runtime_error("INTERLEAVE_OUTPUT="+arg+": expected raw, split or guide");
    }

    // All or nothing. Run ahead of validate_roi(), whose std::stoi reads "12x"
    // as 12 and throws on a value too large for an int, which nothing catches.
    std::optional<std::vector<int>> parse_ints(const std::vector<std::string> &tokens) {
      std::vector<int> values;
      values.reserve(tokens.size());
      for (const auto &token : tokens) {
        const auto value = parse_int(token);
        if (!value) return std::nullopt;
        values.push_back(*value);
      }
      return values;
    }

    // Parse "<cpu|any>[,<SCHED_FIFO priority>]". Rejects at startup what the
    // kernel would refuse when the thread starts, where it can only be logged.
    ThreadPlacement parse_placement(const std::string &param, const std::string &arg) {
      const auto invalid = [&](const std::string &why) {
        return std::runtime_error(param+"="+arg+": "+why+", expected <cpu|any>[,<priority>]");
      };
      const std::string_view text(arg);
      const auto comma = text.find(',');
      const auto cpu_text = trim(text.substr(0, comma));
      ThreadPlacement placement;

      if (cpu_text != ANY_CPU_TOKEN) {
        const long num_cpus = sysconf(_SC_NPROCESSORS_CONF);
        const auto cpu = parse_int(cpu_text);
        if (!cpu || *cpu < 0 || *cpu >= num_cpus) {
          throw invalid("cpu must be \"any\" or {0:"+std::to_string(num_cpus-1)+"}");
        }
        placement.cpu = *cpu;
      }

      if (comma != std::string_view::npos) {
        const int min_priority = sched_get_priority_min(SCHED_FIFO);
        const int max_priority = sched_get_priority_max(SCHED_FIFO);
        const auto priority = parse_int(trim(text.substr(comma + 1)));
        if (!priority || *priority < min_priority || *priority > max_priority) {
          throw invalid("priority must be {"+std::to_string(min_priority)+":"+
                        std::to_string(max_priority)+"}");
        }
        placement.rt_priority = *priority;
      }
      return placement;
    }
  }

  const std::unordered_map<std::string, HispecTrackingCamera::CmdHandler>
  HispecTrackingCamera::command_handlers_ = {
    {"h2rg_init",   &HispecTrackingCamera::h2rg_init},
    {"window_mode", &HispecTrackingCamera::window_mode},
    {"roi",  &HispecTrackingCamera::roi},
    {"exposure", &HispecTrackingCamera::_exposure_mode},
    {"autofetch_mode", &HispecTrackingCamera::_autofetch_mode},
    {"mode", &HispecTrackingCamera::mode},
    {"freerun", &HispecTrackingCamera::freerun},
    {"debug", &HispecTrackingCamera::_debug},
    {"take_stats", &HispecTrackingCamera::_take_stats},
    {"interleave_science", &HispecTrackingCamera::interleave_science},
    {"interleave_guide", &HispecTrackingCamera::interleave_guide},
    {"interleave_set", &HispecTrackingCamera::interleave_set}
  };
  // FASTLOADPARAM is case-sensitive, so these must match the ACF exactly
  const std::unordered_map<std::string, std::string>
  HispecTrackingCamera::_exposure_modes = {
    {"utr_rr", "mode_UTR_RR"},
    {"utr_gr", "mode_UTR_GR"},
    {"rx", "mode_RX"},
    {"rxr", "mode_RXR"},
    {"interleave", "mode_InterleavedGuiding"}
  };

  /***** Camera::HispecTrackingCamera::is_instrument_command ******************/
  /**
   * @brief  true if cmd names an instrument-specific handler
   */
  bool HispecTrackingCamera::is_instrument_command(const std::string &cmd) {
    return command_handlers_.find(cmd) != command_handlers_.end();
  }
  /***** Camera::HispecTrackingCamera::is_instrument_command ******************/


  /***** Camera::HispecTrackingCamera::instrument_commands ********************/
  /**
   * @brief  names of every instrument-specific command, sorted
   */
  std::vector<std::string> HispecTrackingCamera::instrument_commands() const {
    std::vector<std::string> names;
    names.reserve(command_handlers_.size());
    for (const auto &[name, handler] : command_handlers_) names.push_back(name);
    std::sort(names.begin(), names.end());   // unordered_map order is unspecified
    return names;
  }
  /***** Camera::HispecTrackingCamera::instrument_commands ********************/


  /***** Camera::HispecTrackingCamera::instrument_cmd *************************/
  /**
   * @brief  dispatch an instrument-specific command
   */
  long HispecTrackingCamera::instrument_cmd(const std::string &cmd,
                                            const std::string &args,
                                            std::string &retstring) {
    auto it = command_handlers_.find(cmd);
    if (it == command_handlers_.end()) {
      retstring = "unrecognized command";
      return ERROR;
    }
    return (this->*(it->second))(args, retstring);
  }
  /***** Camera::HispecTrackingCamera::instrument_cmd *************************/


  /***** Camera::HispecTrackingCamera::state_summary *************************/
  /**
   * @brief  Snapshot of controller and geometry state, for error logs
   */
  std::string HispecTrackingCamera::state_summary() const {
    std::ostringstream oss;
    oss << "[connected=" << (this->controller->is_connected ? "y" : "n")
        << " powered="   << (this->controller->is_powered ? "y" : "n")
        << " firmware="  << (this->controller->is_firmwareloaded ? "y" : "n")
        << " mode="      << (this->controller->selectedmode.empty()
                             ? "none" : this->controller->selectedmode)
        << " readout="   << (this->cur_exposure_mode.empty()
                             ? "none" : this->cur_exposure_mode);

    const auto mode = this->controller->modemap.find(this->controller->selectedmode);
    if (mode != this->controller->modemap.end()) {
      const auto &geometry = mode->second.geometry;
      oss << " geometry=" << geometry.pixelcount << "x" << geometry.linecount
          << " taps="     << geometry.amps[0] << "x" << geometry.amps[1];
    }

    oss << " window="  << (this->is_window ? "y" : "n")
        << " exptime=" << std::fixed << std::setprecision(3) << this->controller->get_exptime()
        << " lastframe=" << this->controller->lastframe
        << " deadline=" << this->controller->readout_time_msec << "ms]";
    return oss.str();
  }
  /***** Camera::HispecTrackingCamera::state_summary *************************/


  /***** Camera::HispecTrackingCamera::log_error *****************************/
  /**
   * @brief  Log what failed, why, and what the camera was doing
   */
  void HispecTrackingCamera::log_error(const std::string &function,
                                       const std::string &brief,
                                       const std::string &detail) const {
    std::string message = "ERROR " + brief;
    if (!detail.empty()) message += ": " + detail;
    logwrite(function, message + " " + this->state_summary());
  }
  /***** Camera::HispecTrackingCamera::log_error *****************************/


  /***** Camera::HispecTrackingCamera::fail_detailed *************************/
  /**
   * @brief  Hand the caller the short reason, give the log the root cause
   */
  long HispecTrackingCamera::fail_detailed(const std::string &function,
                                           std::string &retstring,
                                           const std::string &brief,
                                           const std::string &detail) const {
    this->log_error(function, brief, detail);
    retstring = brief;
    return ERROR;
  }
  /***** Camera::HispecTrackingCamera::fail_detailed *************************/


  /***** Camera::HispecTrackingCamera::configure_instrument *******************/
  /**
   * @brief      extract+apply instrument-specific parameters from config file
   * @throws     std::runtime_error
   *
   */
  void HispecTrackingCamera::configure_instrument() {
    const std::string function("Camera::HispecTrackingCamera::configure_instrument");

    this->lvds_module = 10;
    this->h2rg_max_pixel = 2047;

    this->pixel_time_usec = DEFAULT_PIXEL_TIME_USEC;
    this->readout_margin_msec = DEFAULT_READOUT_MARGIN_MSEC;

    for (int row=0; row < this->configfile.n_rows; ++row) {
      const auto &param = this->configfile.param[row];
      const auto &arg   = this->configfile.arg[row];
      if (param=="REFPIX_AMP" && !arg.empty()) {
        this->refpix_amp = arg;
        continue;
      }
      if (param=="ACQUISITION_THREAD" || param=="PROCESSING_THREAD") {
        auto &placement = (param=="ACQUISITION_THREAD") ? this->acquisition_thread_placement
                                                        : this->processing_thread_placement;
        placement = parse_placement(param, arg);
        logwrite(function, param + ": " + describe_placement(placement));
        continue;
      }
      if (param=="INTERLEAVE_OUTPUT") {
        this->interleave_output = parse_interleave_output(arg);
        logwrite(function, param + ": " + arg);
        continue;
      }
      if (param=="INTERLEAVE_GUIDE_FITS") {
        this->is_guide_fits_enabled = parse_bool(arg);
        logwrite(function, param + ": " + (this->is_guide_fits_enabled ? "yes" : "no"));
        continue;
      }
      try {
        if      (param=="PIXEL_TIME_USEC")     this->pixel_time_usec     = std::stod(arg);
        else if (param=="READOUT_MARGIN_MSEC") this->readout_margin_msec = std::stod(arg);
        else continue;
      }
      catch (const std::exception &e) {
        throw std::runtime_error("parsing "+param+"="+arg+": "+e.what());
      }
      if (this->pixel_time_usec <= 0 || this->readout_margin_msec < 0) {
        throw std::runtime_error(param+"="+arg+" must be positive");
      }
    }

    logwrite(function, "LVDS module=" + std::to_string(this->lvds_module) +
                       " H2RG max pixel=" + std::to_string(this->h2rg_max_pixel) +
                       " pixel_time=" + std::to_string(this->pixel_time_usec) + " usec" +
                       " readout_margin=" + std::to_string(this->readout_margin_msec) + " msec" +
                       " reference channel amp=" + this->refpix_amp);

    // Optimize Archon socket for high-speed streaming
    constexpr int socket_buf_size = 1024 * 1024;  // 1 MB
    this->controller->archon.set_tcp_nodelay(true);
    this->controller->archon.set_recv_buf_size(socket_buf_size);
    this->controller->archon.set_send_buf_size(socket_buf_size);
  }
  /***** Camera::HispecTrackingCamera::configure_instrument *******************/


  /***** Camera::HispecTrackingCamera::get_exposure_modes *********************/
  /**
   * @brief      return a vector of strings of recognized exposure modes
   * @details    This adds HispecTrackingCamera exposure modes to the base exposure modes.
   * @return     vector<string>
   *
   */
  std::vector<std::string> HispecTrackingCamera::get_exposure_modes() {
    // base exposure modes
    auto modes = this->ArchonInterface::get_exposure_modes();

    // add hispec tracking camera exposure modes
    for (const auto &mode : Camera::HispecTrackingCameraExposureMode::ALLMODES) { modes.push_back(mode); }

    return modes;
  }
  /***** Camera::HispecTrackingCamera::get_exposure_modes *********************/


  /***** Camera::HispecTrackingCamera::set_exposure_mode **********************/
  /**
   * @brief      actually sets the exposure mode
   * @details    This creates the appropriate exposure mode object for the
   *             requested exposure mode, providing access to that mode's functions.
   *             This is hispec tracking camera-specific but gets called by
   *             ArchonInterface because it overrides. If the requested mode is
   *             not a hispec tracking camera mode then this will call the
   *             set_exposure_mode in the base class.
   * @return     ERROR|NO_ERROR
   *
   */
  long HispecTrackingCamera::set_exposure_mode(const std::string &modein, const std::vector<std::string> &modeargs) {

    if (modein==HispecTrackingCameraExposureMode::DEFAULT) {
      if (this->is_autofetch_mode)
        this->exposuremode = std::make_unique<ExposureModeHispecTrackingAutofetch>(this);
      else
        this->exposuremode = std::make_unique<ExposureModeHispecTrackingDefault>(this);
    }
    else {
      return this->ArchonInterface::set_exposure_mode(modein, modeargs);
    }

    return NO_ERROR;
  }
  /***** Camera::HispecTrackingCamera::set_exposure_mode **********************/


  /***** Camera::HispecTrackingCamera::load_firmware *************************/
  /**
   * @brief      load the ACF, then select the hispec exposure pipeline
   * @details    A load leaves no exposure mode selected, and this instrument
   *             has one pipeline, so it is selected after every load. Help, or
   *             a failed load, selects nothing.
   * @param[in]  args       fully qualified path of ACF
   * @param[out] retstring  contains help on request, otherwise not used
   * @return     ERROR|NO_ERROR|HELP
   *
   */
  long HispecTrackingCamera::load_firmware(const std::string &args, std::string &retstring) {
    long error = this->ArchonInterface::load_firmware(args, retstring);
    if (error != NO_ERROR) return error;
    {
      // A load puts the ACF's geometry and parameters back, so nothing saved still applies
      const std::lock_guard<std::mutex> lock(this->interleave_mutex);
      this->h2rg_window.reset();
      this->pre_interleave_readout.reset();
    }
    return this->set_exposure_mode(HispecTrackingCameraExposureMode::DEFAULT, {});
  }
  /***** Camera::HispecTrackingCamera::load_firmware *************************/


  /***** Camera::HispecTrackingCamera::_autofetch_mode ***********************/
  /**
   * @brief  toggle autofetch, then re-select the exposure pipeline so the
   *         autofetch/default class matches the new state
   */
  long HispecTrackingCamera::_autofetch_mode(const std::string &args, std::string &retstring) {
    std::string state = args;
    std::transform(state.begin(), state.end(), state.begin(), ::toupper);
    const bool disabling = (state == "FALSE" || state == "0");

    long error = NO_ERROR;
    if (disabling) {
      error = this->stop_autofetch_stream(retstring);
    }
    else {
      error = this->ArchonInterface::autofetch_mode(args, retstring);
    }

    if (error == NO_ERROR && !args.empty()) {
      error = this->set_exposure_mode(HispecTrackingCameraExposureMode::DEFAULT, {});
    }
    return error;
  }
  /***** Camera::HispecTrackingCamera::_autofetch_mode ***********************/


  /***** Camera::HispecTrackingCamera::set_parameter_offstream ***************/
  /**
   * @brief  Write an Archon parameter without using the streaming socket
   * @details While autofetch is armed the primary connection carries pushed
   *          frames, so a reply arrives behind them and never matches. The
   *          control connection never sees pushes, so it is used instead.
   */
  long HispecTrackingCamera::set_parameter_offstream(const std::string &name, int value) {
    const std::string arg = name + " " + std::to_string(value);
    // A freerun readout keeps the primary connection busy with FETCH traffic
    // just as autofetch does with pushes, so both divert
    const bool streaming = this->is_autofetch_mode || this->is_freerun_active.load();
    if (!streaming) {
      std::string dummy;
      return this->set_parameter(arg, dummy);
    }
    long error = this->controller->send_control_cmd("FASTPREPPARAM " + arg);
    if (error == NO_ERROR) error = this->controller->send_control_cmd("FASTLOADPARAM " + arg);
    return error;
  }
  /***** Camera::HispecTrackingCamera::set_parameter_offstream ***************/


  /***** Camera::HispecTrackingCamera::stop_autofetch_stream *****************/
  /**
   * @brief  Stop the autofetch stream without disturbing the data connection
   * @details The base disable calls ArchonController::stop_autofetch, which
   *          closes and reconnects the data connection. That abandons frames
   *          the controller is still sending, which wedges it, and it does so
   *          under the producer thread still reading that socket, which
   *          intermittently aborts the process. Here the push is stopped and
   *          the producer is left to drain and exit on its own.
   */
  long HispecTrackingCamera::stop_autofetch_stream(std::string &retstring) {
    const std::string function("Camera::HispecTrackingCamera::stop_autofetch_stream");
    long error = NO_ERROR;

    for (const auto &cmd : {"FASTPREPPARAM freerun 0", "FASTLOADPARAM freerun 0",
                            "FASTPREPPARAM Expose 0",  "FASTLOADPARAM Expose 0",
                            "FASTAUTOFETCH0"}) {
      if (this->controller->send_control_cmd(cmd) != NO_ERROR) error = ERROR;
    }
    if (error != NO_ERROR) {
      return this->fail_detailed(function, retstring, "autofetch not stopped",
                 "one or more disable commands failed on the control connection");
    }

    // Winds the producer down; the read it interrupts drains what is left
    this->set_abortstate();
    this->is_autofetch_mode = false;

    logwrite(function, "autofetch stream stopped");
    retstring = "false";
    return NO_ERROR;
  }
  /***** Camera::HispecTrackingCamera::stop_autofetch_stream *****************/


  /***** Camera::HispecTrackingCamera::expose ********************************/
  /**
   * @brief  Autofetch streams continuously, so run one producer session for all
   *         frames; non-autofetch keeps the base per-frame expose loop
   */
  long HispecTrackingCamera::expose(const std::string args, std::string &retstring) {
    const std::string function("Camera::HispecTrackingCamera::expose");
    long error = NO_ERROR;

    // Counted, not a flag: a second expose refused while the first still runs
    // must not mark the camera idle on its way out
    ++this->exposes_in_progress;
    struct ExposeFinished {
      std::atomic<int> &in_progress;
      ~ExposeFinished() { --in_progress; }
    } finished{this->exposes_in_progress};

    if (this->is_freerunning) {
      if (args=="?" || args=="help") {
        retstring = CAMERAD_EXPOSE;
        retstring.append( "\n" );
        retstring.append( "  Freerun mode: starts a continuous exposure loop in the background\n" );
        retstring.append( "  and returns immediately. Stop it with \"" + CAMERAD_ABORT + "\".\n" );
        return HELP;
      }
      // Check the camera is ready BEFORE taking the guard, so a failed check
      // cannot leave the guard set and lock freerun out permanently.
      if (!this->controller->is_connected) {
        return this->fail_detailed(function, retstring, "not connected",
                   "no socket is open to the Archon; run \"open\" first");
      }
      if (!this->controller->is_powered) {
        return this->fail_detailed(function, retstring, "power is off",
                   "Archon reports power state \""+this->controller->power_status+"\"");
      }
      if (!this->is_exposuremode_set()) {
        return this->fail_detailed(function, retstring, "exposure mode not set",
                   "no acquisition pipeline is selected");
      }

      // The freerun consumer is owned by the hispec exposure modes
      auto* mode = dynamic_cast<ExposureModeHispecTrackingBase*>(this->exposuremode.get());
      if (mode == nullptr) {
        return this->fail_detailed(function, retstring, "wrong exposure mode",
                   "\""+this->exposuremode->get_type()+"\" is a base Archon pipeline "
                   "and has no freerun loop");
      }

      // A session that ended on its own -- a producer error, with no abort to
      // tear it down -- leaves the guard set with no threads running. Reap it
      // here, otherwise every later start would be refused.
      if (this->is_freerun_active.load() && !mode->is_freerun_running()) {
        mode->stop_freerun();                  // joins the finished threads
        mode->is_freerun.store(false);
        this->is_freerun_active.store(false);
        logwrite(function, "reaped a freerun session that had already ended");
      }

      // Check if freerun is already active
      if (this->is_freerun_active.exchange(true)) {
        return this->fail_detailed(function, retstring, "freerun already running",
                   "a session is already active; run \"abort\" before starting another");
      }

      //Clear abort state before starting freerun loop
      this->clear_abortstate();
      if (this->prepare_trigger(retstring) != NO_ERROR) {
        this->is_freerun_active.store(false);  // release the guard
        return ERROR;
      }
      // The host re-arms every interleaved pass, so there the sequencer must
      // not loop on its own
      if (this->set_parameter_offstream("freerun", this->is_interleave() ? 0 : 1) != NO_ERROR) {
        this->is_freerun_active.store(false);  // release the guard
        return this->fail_detailed(function, retstring, "cannot set freerun",
                   "the Archon rejected \"freerun 1\"; check that freerun exists "
                   "in the loaded ACF");
      }
      
      // Spawn priority and cpu core pinned processing and consumer threads for freerun mode
      logwrite(function, "freerun mode: starting continuous exposure loop");
      mode->is_freerun.store(true);

      // Start the freerun loop in a background thread.
      //
      // This deliberately does not go through do_expose(), which spawns and
      // joins a producer and a consumer per call -- per frame, here.
      // start_freerun() spawns one of each for the whole session and returns
      // immediately, so no thread is created per frame and nothing is parked.
      // "abort" ends it: both loops watch the abort state, and abort() joins.
      if (mode->start_freerun() != NO_ERROR) {
        mode->is_freerun.store(false);
        this->is_freerun_active.store(false);  // release the guard
        return this->fail_detailed(function, retstring, "freerun did not start",
                   "the producer and consumer threads could not be spawned");
      }


      retstring = "Continuouse exposure loop started in the background. Stop it with \"" + CAMERAD_ABORT + "\".";
      return NO_ERROR;
    } else if (this->is_autofetch_mode) {
      if (!this->controller->is_connected) {
        return this->fail_detailed(function, retstring, "not connected",
                   "no socket is open to the Archon; run \"open\" first");
      }
      if (!this->controller->is_powered) {
        return this->fail_detailed(function, retstring, "power is off",
                   "Archon reports power state \""+this->controller->power_status+"\"");
      }
      if (!this->is_exposuremode_set()) {
        return this->fail_detailed(function, retstring, "exposure mode not set",
                   "no acquisition pipeline is selected");
      }
      if (this->prepare_trigger(retstring) != NO_ERROR) return ERROR;
      if (auto* m = dynamic_cast<ExposureModeHispecTrackingBase*>(this->exposuremode.get())) {
        m->set_args({args});
        // do_expose() joins the producer, so it must not be the endless one
        m->is_freerun.store(false);
      }
      error = this->do_expose();
      this->end_exposure();   // finalize the datacube, if one is open
      return error;
    } else {
      return this->run_exposure_sequence(args, retstring);
    }
  }
  /***** Camera::HispecTrackingCamera::expose ********************************/


  /***** Camera::HispecTrackingCamera::readout_timeout_msec ******************/
  /**
   * @brief      per-frame readout deadline for the current geometry
   * @details    exposure time + one pixel_time per pixel per tap, taps being
   *             parallel. Scaling with geometry is what lets a full frame and a
   *             small ROI share one setting, where wait_for_readout()'s fixed
   *             fallback is too short for the former.
   * @return     deadline in msec, or 0 if no camera mode is selected
   *
   */
  int HispecTrackingCamera::readout_timeout_msec() const {
    const double readout_msec = this->frame_readout_sec() * MSEC_PER_SEC;
    if (readout_msec <= 0.0) return 0;   // no camera mode selected

    const double exposure_msec = this->controller->get_exptime() * MSEC_PER_SEC;
    const double total_msec = exposure_msec + readout_msec + this->readout_margin_msec;

    return std::max(1, static_cast<int>(std::lround(total_msec)));
  }
  /***** Camera::HispecTrackingCamera::readout_timeout_msec ******************/


  /***** Camera::HispecTrackingCamera::effective_pixel_time_usec *************/
  /**
   * @brief      pixel time in force, from the ACF when it carries one
   * @details    The selected mode's tpix, then the global tpix, then the CDS
   *             sample end SHD2, then PIXEL_TIME_USEC from the config file.
   * @return     microseconds per pixel
   *
   */
  double HispecTrackingCamera::effective_pixel_time_usec() const {
    const auto* controller = this->controller;
    std::vector<std::string> sources;
    const auto mode = controller->modemap.find(controller->selectedmode);
    if (mode != controller->modemap.end()) {
      const auto param = mode->second.parammap.find(ACF_PIXEL_PERIOD_PARAM);
      if (param != mode->second.parammap.end()) sources.push_back(param->second.value);
    }
    if (const auto param = controller->parammap.find(ACF_PIXEL_PERIOD_PARAM);
        param != controller->parammap.end()) {
      sources.push_back(param->second.value);
    }
    if (const auto entry = controller->configmap.find(ACF_PIXEL_TIME_KEY);
        entry != controller->configmap.end()) {
      sources.push_back(entry->second.value);
    }
    for (const auto &source : sources) {
      if (const auto ticks = positive_ticks(source)) return *ticks * ARCHON_TICK_USEC;
    }
    return this->pixel_time_usec;
  }
  /***** Camera::HispecTrackingCamera::effective_pixel_time_usec *************/


  /***** Camera::HispecTrackingCamera::frame_readout_sec *********************/
  /**
   * @brief      time to clock out one amplifier region in the selected mode
   * @details    The readout deadline and the FRAMETME keyword are the same
   *             quantity, so both come from here and cannot disagree.
   * @return     seconds, or 0 if no camera mode is selected
   *
   */
  double HispecTrackingCamera::frame_readout_sec() const {
    const auto mode = this->controller->modemap.find(this->controller->selectedmode);
    if (mode == this->controller->modemap.end()) return 0.0;

    // Pixels one tap clocks out, which is what sets the readout duration
    const auto &geometry = mode->second.geometry;
    if (geometry.pixelcount <= 0 || geometry.linecount <= 0) return 0.0;
    const double pixels_per_tap = static_cast<double>(geometry.pixelcount) * geometry.linecount;

    return pixels_per_tap * this->effective_pixel_time_usec() / USEC_PER_SEC;
  }
  /***** Camera::HispecTrackingCamera::frame_readout_sec *********************/


  /***** Camera::HispecTrackingCamera::run_exposure_sequence *****************/
  /**
   * @brief      acquire a sequence of frames from a single Archon trigger
   * @details    The ACF grabs <nseq> frames per Expose=<nseq> and pulses reset
   *             only on the first of them, so one trigger yields one ramp.
   *             ArchonInterface::expose() instead sends Expose=1 <nseq> times,
   *             which re-enters the sequence and resets on every frame, giving
   *             <nseq> unrelated single reads rather than a ramp.
   * @param[in]  args       number of frames, default 1
   * @param[out] retstring  number of frames on success, reason on failure
   * @return     ERROR | NO_ERROR | HELP
   *
   */
  long HispecTrackingCamera::run_exposure_sequence(const std::string &args, std::string &retstring) {
    const std::string function("Camera::HispecTrackingCamera::run_exposure_sequence");

    if (args=="?" || args=="help") {
      retstring = CAMERAD_EXPOSE;
      retstring.append( " [ <nseq> ]\n" );
      retstring.append( "  Acquire <nseq> frames (default 1) from a single Archon trigger,\n" );
      retstring.append( "  resetting the detector only before the first.\n" );
      retstring.append( "  In the interleave readout <nseq> counts passes over the science\n" );
      retstring.append( "  window, each one a trigger that yields a frame per set.\n" );
      return HELP;
    }

    if (!this->controller->is_connected) {
      return this->fail_detailed(function, retstring, "not connected",
                 "no socket is open to the Archon; run \"open\" first");
    }
    if (!this->controller->is_powered) {
      return this->fail_detailed(function, retstring, "power is off",
                 "Archon reports power state \""+this->controller->power_status+
                 "\"; run \"power on\" first");
    }
    if (!this->is_exposuremode_set()) {
      return this->fail_detailed(function, retstring, "exposure mode not set",
                 "no acquisition pipeline is selected; \"load\" or \"mode\" "
                 "normally selects one");
    }

    auto* mode = dynamic_cast<ExposureModeHispecTrackingBase*>(this->exposuremode.get());
    if (mode == nullptr) {
      return this->fail_detailed(function, retstring, "wrong exposure mode",
                 "\""+this->exposuremode->get_type()+"\" is a base Archon pipeline "
                 "with no sequence support; select a hispec mode");
    }

    // Range-checked here rather than in the producer: the producer hands this
    // to prep_parameter(), which throws when it is out of range, and a throw
    // leaving a std::thread terminates the process.
    int nseq = 1;
    try {
      if (!args.empty()) nseq = std::stoi(args);
    }
    catch (const std::exception &) {
      return this->fail_detailed(function, retstring, "invalid frame count",
                 "\""+args+"\" is not an integer");
    }
    if (nseq < 1 || nseq > ARCHON_PARAM_MAX) {
      return this->fail_detailed(function, retstring, "frame count out of range",
                 std::to_string(nseq)+" is outside {1:"+std::to_string(ARCHON_PARAM_MAX)+
                 "}; the Archon Expose parameter is 20 bits");
    }

    // The producer exits on the abort state before reading anything, so without
    // this an exposure following an abort would quietly return no frames
    this->clear_abortstate();

    // Ahead of the deadline below, which is worked out from the geometry the interleaved readout sets
    if (this->prepare_trigger(retstring) != NO_ERROR) return ERROR;

    // Recomputed per command because roi and mode change the geometry
    const int timeout_msec = this->readout_timeout_msec();
    if (timeout_msec > 0) {
      this->controller->readout_time_msec = timeout_msec;
      logwrite(function, "readout deadline "+std::to_string(timeout_msec)+" msec per frame");
    }

    // In the interleave readout the count is passes, each one a frame per set
    const long long frames_wanted = this->is_interleave()
        ? static_cast<long long>(nseq) * this->interleave_geometry.sets_per_pass() : nseq;

    mode->nseq.store(nseq);
    mode->is_freerun.store(false);  // do_expose() joins the producer, so it must terminate

    const long error = this->do_expose();

    this->end_exposure();           // finalize the datacube, if one is open

    if (error != NO_ERROR) {
      const long long got = mode->frames_acquired.load();
      std::string cause = "the acquisition thread stopped";
      if (mode->is_consumer_error) cause = "the processing thread stopped";
      else if (!mode->is_producer_error) cause = "neither thread flagged an error, so the stage is unknown";
      return this->fail_detailed(function, retstring, "exposure failed",
                                 cause+" after acquiring "+std::to_string(got)+" of "+
                                 std::to_string(frames_wanted)+" frame(s); the preceding log line "
                                 "gives the failing step");
    }

    // The frames that did arrive are written; the caller still has to know the sequence has gaps
    if (const long long dropped = mode->frames_dropped.load(); dropped > 0) {
      return this->fail_detailed(function, retstring, "frames dropped",
                                 std::to_string(dropped)+" of "+std::to_string(frames_wanted)+
                                 " frame(s) were overwritten before the host fetched them; "
                                 "the other "+std::to_string(frames_wanted-dropped)+" were written");
    }

    retstring = std::to_string(nseq);

    return NO_ERROR;
  }
  /***** Camera::HispecTrackingCamera::run_exposure_sequence *****************/

  /***** Camera::HispecTrackingCamera::abort *********************************/
  /**
   * @brief      abort, tearing down a freerun session if one is running
   * @param[in]  args
   * @param[out] retstring
   * @return     ERROR|NO_ERROR
   *
   */
  long HispecTrackingCamera::abort(const std::string args, std::string &retstring) {
    const std::string function("Camera::HispecTrackingCamera::abort");

    // Both freerun loops watch the abort state, so they start winding down as
    // soon as it is set. The parameter write goes through set_parameter_offstream
    // rather than the base abort, whose set_parameter() would put it on the
    // primary socket, where a readout in progress collides with the reply.
    this->set_abortstate();
    long error = NO_ERROR;
    if (!this->controller->abort_param.empty()) {
      error = this->set_parameter_offstream(this->controller->abort_param, 1);
    }

    if (auto* m = dynamic_cast<ExposureModeHispecTrackingBase*>(this->exposuremode.get())) {
      if (this->is_freerun_active.load()) {
        // blocks only until the in-flight frame read finishes
        error |= m->stop_freerun();
        m->is_freerun.store(false);
        this->is_freerun_active.store(false);  // release the guard
        // Only safe once both threads are joined: nothing is still dispatching
        this->end_exposure();
        logwrite(function, "freerun session stopped");
      }
    }

    if (error != NO_ERROR && retstring.empty()) {
      return this->fail_detailed(function, retstring, "abort incomplete",
                 "the abort was delivered but winding the session down reported "
                 "a failure; the preceding log line names the thread");
    }
    return error;
  }
  /***** Camera::HispecTrackingCamera::abort *********************************/


  /***** Camera::HispecTrackingCamera::freerun ********************************/
  /**
   * @brief      start the freerun exposure loop
   * @details    This method starts the freerun exposure loop in the background.
   * @param[in]  args       arguments for the freerun mode
   * @param[out] retstring  return string
   * @return     ERROR|NO_ERROR
   *
   */
  long HispecTrackingCamera::freerun(const std::string &args, std::string &retstring) {
    const std::string function("Camera::HispecTrackingCamera::freerun");
    long error = NO_ERROR;
    // Preps archon and camera-interface for freerun mode
     if (args.empty()) {
      return this->fail_detailed(function, retstring, "missing argument",
                 "freerun expects 1 to arm continuous exposure or 0 to disarm it");
    }
    const std::string value = args;
    error = this->set_parameter_offstream("freerun", value == "1" ? 1 : 0);
    this->is_freerunning = (value=="1");

    return error;
  }
  /***** Camera::HispecTrackingCamera::freerun ********************************/

  /***** Camera::HispecTrackingCamera::_debug ********************************/
  /**
   * @brief      enable or disable per-frame debug logging
   * @details    Every informational logwrite() in the hispec acquisition and
   *             processing threads is gated on this, as is timing-statistics
   *             collection. Logging serializes on a mutex and hits the disk, so
   *             leaving it on measurably slows acquisition and processing --
   *             hence off by default. Errors are logged either way.
   *
   *             The setting is kept on the instrument and pushed into the
   *             exposure mode, so it survives an exposure mode change, and it
   *             can be toggled while an exposure or freerun loop is running.
   *
   * @param[in]  args       "true"|"1" to enable, "false"|"0" to disable, empty to query
   * @param[out] retstring  current state ("true" or "false")
   * @return     ERROR|NO_ERROR|HELP
   *
   */
  long HispecTrackingCamera::_debug(const std::string &args, std::string &retstring) {
    const std::string function("Camera::HispecTrackingCamera::_debug");

    if (args=="?" || args=="help") {
      retstring = "debug [ true | false ]\n";
      retstring.append( "  Enable or disable per-frame debug logging for the hispec\n" );
      retstring.append( "  exposure threads. Off by default to avoid blowing up logs,\n" );
      retstring.append( "  and hindering acquisition and processing. Errors are always\n" );
      retstring.append( "  logged. With no argument, returns the current state.\n" );
      return HELP;
    }

    if (!args.empty()) {
      std::string state = args;
      std::transform(state.begin(), state.end(), state.begin(), ::toupper);

      if      (state=="TRUE"  || state=="1") this->is_debug = true;
      else if (state=="FALSE" || state=="0") this->is_debug = false;
      else {
        retstring = "ERROR expected true|false";
        logwrite(function, retstring);
        return ERROR;
      }

      // applies to the running exposure mode immediately
      if (auto* m = dynamic_cast<ExposureModeHispecTrackingBase*>(this->exposuremode.get())) {
        m->set_debug(this->is_debug); 
      }
      logwrite(function, this->is_debug ? "debug logging enabled" : "debug logging disabled");
    }

    retstring = (this->is_debug ? "true" : "false");
    return NO_ERROR;
  }
  /***** Camera::HispecTrackingCamera::_debug ********************************/

  /**** Camera::HispecTrackingCamera::_take_stats *************/
  /**
   * @brief      enable or disable per-frame timing statistics collection
   * @details    Every timing measurement in the hispec acquisition and
   *             processing threads is gated on this, so it can be toggled while
   *             an exposure or freerun loop is running. Statistics are logged
   *             at the end of each exposure.
   *
   * @param[in]  args       "true"|"1" to enable, "false"|"0" to disable, empty to query
   * @param[out] retstring  current state ("true" or "false")
   * @return     ERROR|NO_ERROR|HELP
   *
   */
  long HispecTrackingCamera::_take_stats(const std::string &args, std::string &retstring) {
    const std::string function("Camera::HispecTrackingCamera::_take_stats");

    if (args=="?" || args=="help") {
      retstring = "take_stats [ true | false ]\n";
      retstring.append( "  Enable or disable per-frame timing statistics collection for the hispec\n" );
      retstring.append( "  exposure threads. Off by default to minimize impact on performance,\n" );
      retstring.append( "  and hindering acquisition and processing. Errors are always\n" );
      retstring.append( "  logged. With no argument, returns the current state.\n" );
      return HELP;
    }

    if (!args.empty()) {
      std::string state = args;
      std::transform(state.begin(), state.end(), state.begin(), ::toupper);

      if      (state=="TRUE"  || state=="1") this->take_stats = true;
      else if (state=="FALSE" || state=="0") this->take_stats = false;
      else {
        retstring = "ERROR expected true|false";
        logwrite(function, retstring);
        return ERROR;
      }

      // applies to the running exposure mode immediately
      if (auto* m = dynamic_cast<ExposureModeHispecTrackingBase*>(this->exposuremode.get())) {
        m->set_take_stats(this->take_stats); 
      }
      logwrite(function, this->take_stats ? "timing statistics collection enabled" : "timing statistics collection disabled");
    }

    retstring = (this->take_stats ? "true" : "false");
    return NO_ERROR;
  }
  /**** Camera::HispecTrackingCamera::_take_stats *************/


  /***** Camera::HispecTrackingCamera::send_inreg ****************************/
  /**
   * @brief      send a VCPU INREG command to the Archon
   * @param[in]  module  module number
   * @param[in]  inreg   input register number
   * @param[in]  value   register value
   * @return     ERROR|NO_ERROR
   *
   */
  long HispecTrackingCamera::send_inreg(int module, int inreg, int value) {
    std::string cmd = std::to_string(module) + " " +
                      std::to_string(inreg) + " " +
                      std::to_string(value);
    std::string retstring;
    return this->set_vcpu_inreg(cmd, retstring);
  }
  /***** Camera::HispecTrackingCamera::send_inreg ****************************/


  /***** Camera::HispecTrackingCamera::send_inreg_clocked ********************/
  /**
   * @brief      send an INREG value and clock it to the detector
   * @details    Writes the value to inreg 1, then pulses inreg 0 (1 then 0)
   *             to latch the value into the detector.
   * @param[in]  module  module number
   * @param[in]  inreg   input register number (typically 1 for data)
   * @param[in]  value   register value
   * @return     ERROR|NO_ERROR
   *
   */
  long HispecTrackingCamera::send_inreg_clocked(int module, int inreg, int value) {
    long error = this->send_inreg(module, inreg, value);
    if (error == NO_ERROR) error = this->send_inreg(module, 0, 1);
    if (error == NO_ERROR) error = this->send_inreg(module, 0, 0);
    return error;
  }
  /***** Camera::HispecTrackingCamera::send_inreg_clocked ********************/


  /***** Camera::HispecTrackingCamera::h2rg_init *****************************/
  /**
   * @brief      initialize the H2RG detector for operation
   * @details    Enables output to Pad B and sets HIGHOHM via LVDS module INREG.
   *             Register value 16402 = 0100 000000010010 (Pad B + HIGHOHM).
   * @param[in]  args       unused
   * @param[out] retstring  return string
   * @return     ERROR|NO_ERROR
   *
   */
  long HispecTrackingCamera::h2rg_init(const std::string &args, std::string &retstring) {
    const std::string function("Camera::HispecTrackingCamera::h2rg_init");

    // Start defaults to 1 in the ACF, so it's already true at load time and
    // never sees a 0->1 transition on power-up. The H2RG main reset only
    // fires on that rising edge, so re-trigger it here now that power is on.
    if (this->controller->set_parameter("Start", 1) != NO_ERROR) {
      return this->fail_detailed(function, retstring, "H2RG init failed",
                 "could not set the Archon Start parameter, which the H2RG main "
                 "reset needs as a 0 to 1 edge; check that Start exists in the ACF");
    }

    // Enable output to Pad B and HIGHOHM: 0100 000000010010 = 16402
    long error = this->send_inreg_clocked(this->lvds_module, 1, 16402);
    if (error != NO_ERROR) {
      return this->fail_detailed(function, retstring, "H2RG init failed",
                 "INREG write of 16402 (Pad B output + HIGHOHM) to LVDS module "+
                 std::to_string(this->lvds_module)+" was not acknowledged");
    }

    logwrite(function, "H2RG initialized: Pad B output and HIGHOHM enabled");
    retstring = "done";
    return NO_ERROR;
  }
  /***** Camera::HispecTrackingCamera::h2rg_init *****************************/

  /***** Camera::HispecTrackingCamera::mode **********************************
  /**
   * @brief      set the camera mode
   * @details    This method sets the camera mode based on the input arguments.
   * @param[in]  args       arguments for the camera mode
   * @param[out] retstring  return string
   * @return     ERROR|NO_ERROR
   *
   */
  long HispecTrackingCamera::mode(const std::string &args, std::string &retstring) {
    const std::string function("Camera::HispecTrackingCamera::mode");
    std::stringstream errstr;
    bool changed = false;

    // With no argument, report the current mode instead of switching
    if (args.empty()) {
      retstring = "Current camera mode: " + this->controller->selectedmode;
      retstring += "\nProvide a valid camera mode to switch to.";
      logwrite(function, retstring);
      return NO_ERROR;
    }

    // Let the base Archon interface select the mode. This validates the mode
    // name, loads the mode's parameters and geometry from the ACF, and applies
    // them (with its own APPLYCDS when the geometry changes). On success it sets
    // controller->selectedmode to the canonical modemap key.
    this->discard_pre_interleave_readout();
    long error = this->ArchonInterface::acfmode(args, retstring);
    if (error != NO_ERROR) {
      return this->fail_detailed(function, retstring, "camera mode not set",
                 "\""+args+"\" was rejected; it must name a [MODE_*] section of the "
                 "loaded ACF and firmware must already be loaded");
    }

    // Use the canonical key the base class just set so our lookup always matches
    // the modemap entry that was selected (avoids case/whitespace drift between
    // the raw arg and how modemap was keyed).
    auto &mode = this->controller->modemap[this->controller->selectedmode];

    // set_camera_mode() only pushes geometry (LINE/PIXELCOUNT) and parameters to
    // the Archon. The [MODE_*] section's configmap also carries the tapline
    // layout (TAPLINES, TAPLINE0..N) and other readout keys, so stage every
    // config key from this mode into the controller's config memory.
    for (const auto &[key, cfg] : mode.configmap) {
      error = this->controller->write_config_key(key.c_str(), cfg.value.c_str(), changed);
      if (error != NO_ERROR) {
        errstr << "ERROR writing config key " << key << "=" << cfg.value
               << " for mode " << this->controller->selectedmode;
        return this->fail_detailed(function, retstring, "camera mode not set",
                   "staging config key "+key+"="+cfg.value+" for mode "+
                   this->controller->selectedmode+" was rejected by the controller");
      }
    }

    // Activate the staged tapline/readout geometry in the CDS core. APPLYCDS
    // reconfigures readout without power-cycling the detector (unlike APPLYALL).
    if (changed && this->controller->send_cmd(APPLYCDS) != NO_ERROR) {
      return this->fail_detailed(function, retstring, "camera mode not set",
                 "APPLYCDS failed after staging the tapline configuration for "+
                 this->controller->selectedmode);
    }

    // The ACF's [MODE_*] sections hold config keys only, so selecting a mode
    // moves the CDS geometry while the detector keeps clocking the previous
    // window. Push the H2RG side to match, otherwise the frame never completes
    // and the readout times out. Any mode whose geometry differs from the ACF
    // defaults (ROI, GUIDING, FAST_GUIDING) is unusable without this.
    const int rows = mode.geometry.linecount;
    const int columns = mode.geometry.pixelcount;
    std::string dummy;
    if (this->set_parameter("H2RG_rows "+std::to_string(rows), dummy)       != NO_ERROR ||
        this->set_parameter("H2RG_columns "+std::to_string(columns), dummy) != NO_ERROR ||
        this->set_parameter("H2RG_rows_skip 0", dummy)                      != NO_ERROR) {
      return this->fail_detailed(function, retstring, "camera mode not set",
                 "the controller rejected the H2RG geometry for "+
                 this->controller->selectedmode+" ("+std::to_string(columns)+"x"+
                 std::to_string(rows)+"); check H2RG_rows, H2RG_columns and "
                 "H2RG_rows_skip exist in the ACF");
    }
    this->record_h2rg_window(columns, 0);

    // mode.tapinfo (num_taps, ampname, readoutdir, gain, offset) is already
    // populated for every mode at ACF-load time (ArchonController::parse_tapinfo),
    // so it is available here via modemap[selectedmode] without re-parsing.

    logwrite(function, "Camera mode set to " + this->controller->selectedmode +
                       " (" + std::to_string(columns) + "x" + std::to_string(rows) + " per tap)");
    retstring = "done";
    return NO_ERROR;
  }
  /**** Camera::HispecTrackingCamera::mode **********************************/

  /***** Camera::HispecTrackingCamera::_exposure_mode ******************************/
  /**
   * @brief      sets the exposure mode for the camera
   * @details    This method sets the exposure mode for the camera based on the input arguments.
   * @param[in]  args       arguments for the exposure mode
   * @param[out] retstring  return string
   * @return     ERROR|NO_ERROR
   *
   */
  long HispecTrackingCamera::_exposure_mode(const std::string &args, std::string &retstring) {
    const std::string function("Camera::HispecTrackingCamera::_exposure_mode");
    std::string dummy;
    std::string param_cmd;
    long error = NO_ERROR;

    // No argument is a query, not a mode change
    if (args.empty()) {
      retstring = this->cur_exposure_mode;
      return NO_ERROR;
    }

    auto req_param = _exposure_modes.find(args);
    if (req_param == _exposure_modes.end()) {
      std::vector<std::string> valid;
      valid.reserve(_exposure_modes.size());
      for (const auto &[name, param] : _exposure_modes) valid.push_back(name);
      std::sort(valid.begin(), valid.end());
      std::string expected;
      for (const auto &name : valid) expected += (expected.empty() ? "" : "|") + name;
      return this->fail_detailed(function, retstring, "invalid exposure mode",
                 "\""+args+"\" is not a readout mode; expected one of "+expected+
                 ", which map to the mode_* parameters in the ACF");
    }
    const std::string &mode_name = req_param->first;
    const std::string &mode_value = req_param->second;

    // The interleaved readout reshapes the frame; the next readout needs the shape from before it
    if (this->is_interleave() && mode_name != INTERLEAVE_READOUT) {
      if (this->is_interleave_frozen()) {
        return this->fail_detailed(function, retstring, "exposure in progress",
                   "the interleaved readout cannot be left while an exposure or freerun "
                   "session runs; abort it or wait for it to finish");
      }
      if (this->restore_pre_interleave_readout(retstring) != NO_ERROR) return ERROR;
    }

    // Every readout's parameter goes to 0, then the chosen one to 1. An ACF that
    // predates a readout lacks its parameter, which then needs no clearing; any
    // other refusal is reported rather than lost behind a later write.
    const auto &acf_parameters = this->controller->parammap;
    for (const auto &[key, value] : _exposure_modes) {
      if (value != mode_value && !acf_parameters.empty() && acf_parameters.find(value) == acf_parameters.end()) {
        logwrite(function, "WARNING "+value+" is not in the loaded ACF, so readout \""+key+
                           "\" is unavailable; not clearing it");
        continue;
      }
      param_cmd = value + " 0";
      if (this->set_parameter(param_cmd, dummy) != NO_ERROR) error = ERROR;
    }

    // set current exposure mode
    param_cmd = mode_value + " 1";
    if (error == NO_ERROR) error = this->set_parameter(param_cmd, dummy);

    // update the exposure mode in the camera
    const bool was_rxr = (this->cur_exposure_mode == RXR_READOUT);
    this->cur_exposure_mode = mode_name;

    if (error == NO_ERROR) error = this->match_pixelcount_to_readout(was_rxr);
    //return retstring and error
    retstring = mode_name;
    return error;
  }
  /***** Camera::HispecTrackingCamera::_exposure_mode ******************************/

  /***** Camera::HispecTrackingCamera::roi ******************************/
  /**
   * @brief      
   * @details    This roi method takes in arguments for the region of interest.
   *                - Checks mode (POSSIBLY SPLIT INTO SEPARATE FUNCTIONS)
   *                - if args = 4
   *                  - vstart, vstop, hstart, hstop
   *                - if args = 2
   *                  - hight and width from center point
   *                - if args = 0
   *                  - query current ROI
   * @param[in]  
   * @param[out] 
   * @return     
   *
   */
  long HispecTrackingCamera::roi(const std::string &args, std::string &retstring) {
    const std::string function("Camera::HispecTrackingCamera::roi");
    long error = NO_ERROR;
    std::string upper_args = args;
    std::transform( upper_args.begin(), upper_args.end(), upper_args.begin(), ::toupper );  // make uppercase
    std::vector<std::string> tokens;
    Tokenize(args, tokens, " ");
    logwrite(function, "roi args: " + args + " tokens: " + std::to_string(tokens.size()));

    // Handle different argument counts
    if (tokens.size() == 4) {
      return this->guiding_roi(args, retstring);
    } else if (tokens.size() == 2) {
      return this->roi_exec(args, retstring);
    } else if (tokens.size() == 1 && upper_args == "FULLFRAME") {
      return this->fullframe(args, retstring);
    } else if (tokens.empty()) {
      retstring = std::to_string(this->win_vstart) + " " + std::to_string(this->win_vstop) + " " +
                  std::to_string(this->win_hstart) + " " + std::to_string(this->win_hstop);
      return NO_ERROR;
    }
    return this->fail_detailed(function, retstring, "invalid ROI arguments",
               "got "+std::to_string(tokens.size())+" argument(s) \""+args+
               "\"; expected none to query, \"<height> <width>\" for a centred ROI, "
               "\"<vstart> <vstop> <hstart> <hstop>\" for an explicit one, "
               "or \"fullframe\"");
  }
  /***** Camera::HispecTrackingCamera::roi ******************************/

  /***** Camera::HispecTrackingCamera::roi_exec ******************************/
  /**
   * @brief      sets a centered reagion of interest with  taplines > 2
   * @details    This roi method takes in arguments for the region of interest.
   *                - Validates the arguments
   *                - Sets the parameters from the acf file
   *                - Sets the CDS variables
   *                - Sets the geometry
   *                - Resizes the image buffer
   * @param[in]  args       2 arguments (height width)
   * @param[out] 
   * @return     
   *
   */
  long HispecTrackingCamera::roi_exec(const std::string &args, std::string &retstring) {
    const std::string function("Camera::HispecTrackingCamera::roi_exec");
    std::stringstream cmd;
    std::stringstream message;
    std::string dummy;
    long error = NO_ERROR;
    int height, width, vstart, vstop, hstart, hstop;

    //validate the arguments
    if (this->validate_roi(args, retstring) != NO_ERROR) { return ERROR; }

    // take detector out of window mode: // 0111 000000001100
    if (this->is_window) {
      error = this->send_inreg_clocked(this->lvds_module, 1, 28684);
      this->is_window = false;
    }   
    //Tokenize the arguments
    std::vector<std::string> tokens;
    Tokenize(args, tokens, " ");
    height = std::stoi(tokens[0]);
    width = std::stoi(tokens[1]);

    //center the ROI
    vstart = (2048 - height) / 2;
    vstop = vstart + height - 1;
    hstart = (1024 - width) / 2;
    hstop = hstart + width - 1;
    // Set Parameters for ROI here
      //H2RG_ rows, window_rows, columns, window_columns, rows_skip
    this->win_vstart = vstart; // set y lo lim
    this->win_vstop = vstop; // set y hi lim
    this->win_hstart = hstart; // set x lo lim
    this->win_hstop = hstop; // set roi x hi lim
    int rows = (this->win_vstop - this->win_vstart) + 1;
    int cols = std::round(((this->win_hstop - this->win_hstart) + 1)/2);
    cmd.str("");
    // Update Archon parameters
    this->set_parameter("H2RG_columns " + std::to_string(cols), dummy);
    this->set_parameter("H2RG_rows " + std::to_string(rows), dummy);
    this->set_parameter("H2RG_rows_skip " + std::to_string(vstart), dummy);
    this->record_h2rg_window(cols, vstart);

    //Check mode and set cds variables
    // Update CDS geometry via config keys
    bool changed = false;
    int pixelcount = cols;
    auto &mode = this->controller->modemap[this->controller->selectedmode];
    if (this->cur_exposure_mode == "rxr") {
      pixelcount = cols * 2;
    }
    this->controller->write_config_key("PIXELCOUNT", pixelcount, changed);
    if (changed) this->controller->send_cmd(APPLYCDS);
    this->controller->write_config_key("LINECOUNT", rows, changed);
    if (changed) this->controller->send_cmd(APPLYCDS);

    //set geometry
    mode.geometry.linecount = rows;
    mode.geometry.pixelcount = pixelcount;
    this->camera_info.region_of_interest = {
      static_cast<uint32_t>(this->win_hstart),
      static_cast<uint32_t>(this->win_hstop),
      static_cast<uint32_t>(this->win_vstart),
      static_cast<uint32_t>(this->win_vstop)
    };
    // Autofetch sizes its reads from section_size, which set_axes derives
    this->camera_info.detector_pixels = {
      static_cast<uint32_t>(pixelcount * mode.tapinfo.num_taps),
      static_cast<uint32_t>(rows)
    };
    this->camera_info.set_axes(16);
    //resize the image buffer TODO::
    //this->resize_image_buffer(pixelcount, rows);
    return error;
  }
  /***** Camera::HispecTrackingCamera::roi_exec ******************************/

  /***** Camera::HispecTrackingCamera::guiding_roi ****************************/
  /**
   * @brief      set window region-of-interest geometry for the H2RG
   * @details    Sets the vstart, vstop, hstart, hstop pixel limits via INREG
   *             commands. If window_mode is active, also updates CDS geometry,
   *             Archon parameters, and camera_info to match.
   *
   *             H2RG INREG register addresses:
   *             vstart = 32768 (1000 000000000000)
   *             vstop  = 36864 (1001 000000000000)
   *             hstart = 40960 (1010 000000000000)
   *             hstop  = 45056 (1011 000000000000)
   *
   * @param[in]  args       "vstart vstop hstart hstop" in pixels, or empty to query
   * @param[out] retstring  current ROI as "vstart vstop hstart hstop"
   * @return     ERROR|NO_ERROR
   *
   */
  long HispecTrackingCamera::guiding_roi(const std::string &args, std::string &retstring) {
    const std::string function("Camera::HispecTrackingCamera::guiding_roi");
    long error = NO_ERROR;

    if (!args.empty()) {
      std::vector<std::string> tokens;
      Tokenize(args, tokens, " ");

      if (this->validate_roi(args, retstring) != NO_ERROR) {return ERROR;}

      int vstart, vstop, hstart, hstop;
      try {
        vstart = std::stoi(tokens[0]);
        vstop  = std::stoi(tokens[1]);
        hstart = std::stoi(tokens[2]);
        hstop  = std::stoi(tokens[3]);
      } catch (const std::exception &e) {
        return this->fail_detailed(function, retstring, "invalid ROI",
                   "could not parse \""+args+"\" as four integers "
                   "<vstart> <vstop> <hstart> <hstop>");
      }
      // Set detector into window mode: 0111 000000001111 = 28687
      error = this->send_inreg_clocked(this->lvds_module, 1, 28687);
      this->is_window = true;

      // Set detector registers for each ROI limit
      // vstart: base address 32768
      error = this->send_inreg_clocked(this->lvds_module, 1, 32768 + vstart);
      if (error == NO_ERROR) this->win_vstart = vstart;

      // vstop: base address 36864
      if (error == NO_ERROR) error = this->send_inreg_clocked(this->lvds_module, 1, 36864 + vstop);
      if (error == NO_ERROR) this->win_vstop = vstop;

      // hstart: base address 40960
      if (error == NO_ERROR) error = this->send_inreg_clocked(this->lvds_module, 1, 40960 + hstart);
      if (error == NO_ERROR) this->win_hstart = hstart;

      // hstop: base address 45056
      if (error == NO_ERROR) error = this->send_inreg_clocked(this->lvds_module, 1, 45056 + hstop);
      if (error == NO_ERROR) this->win_hstop = hstop;

      // If window mode is active, update geometries to match
      //if (error == NO_ERROR && this->is_window) {
      const int rows = (this->win_vstop - this->win_vstart) + 1;
      const int cols = (this->win_hstop - this->win_hstart) + 1;
      std::string dummy;

      // Update Archon parameters
      this->set_parameter("H2RG_columns " + std::to_string(cols), dummy);
      this->set_parameter("H2RG_rows " + std::to_string(rows), dummy);
      this->set_parameter("H2RG_rows_skip 0", dummy);
      this->record_h2rg_window(cols, 0);

      // Update CDS geometry via config keys
      bool changed = false;
      int pixelcount = cols;
      auto &mode = this->controller->modemap[this->controller->selectedmode];
      if (this->cur_exposure_mode == "rxr") {
        pixelcount = cols * 2;
      }
      this->controller->write_config_key("PIXELCOUNT", pixelcount, changed);
      if (changed) this->controller->send_cmd(APPLYCDS);
      this->controller->write_config_key("LINECOUNT", rows, changed);
      if (changed) this->controller->send_cmd(APPLYCDS);

      // Update modemap and camera_info
      mode.geometry.linecount = rows;
      mode.geometry.pixelcount = cols;
      this->camera_info.region_of_interest = {
        static_cast<uint32_t>(this->win_hstart),
        static_cast<uint32_t>(this->win_hstop),
        static_cast<uint32_t>(this->win_vstart),
        static_cast<uint32_t>(this->win_vstop)
      };
      this->camera_info.detector_pixels = {
        static_cast<uint32_t>(pixelcount * this->taplines_store),
        static_cast<uint32_t>(rows)
      };

      // H2RG is 16-bit
      this->camera_info.set_axes(16);
      //}

      if (error != NO_ERROR) {
        return this->fail_detailed(function, retstring, "ROI not set",
                   "one or more INREG writes to LVDS module "+
                   std::to_string(this->lvds_module)+" failed while setting "
                   "vstart/vstop/hstart/hstop");
      }
    }

    retstring = std::to_string(this->win_vstart) + " " +
                std::to_string(this->win_vstop) + " " +
                std::to_string(this->win_hstart) + " " +
                std::to_string(this->win_hstop);
    return error;
  }
  /***** Camera::HispecTrackingCamera::guiding_roi ****************************/

  /**** Camera::HispecTrackingCamera::fullframe ******************************/
  /**
   * @brief      executes the fullframe command
   * @details    This function resets the ROI to fullframe and taplines.
   * @param[in]  args       arguments for the command
   * @param[out] retstring  error message if execution fails
   * @return     ERROR|NO_ERROR
   *
   */
  long HispecTrackingCamera::fullframe(const std::string &args, std::string &retstring) {
    const std::string function("Camera::HispecTrackingCamera::fullframe");
    std::string dummy;
    std::string cmd;
    long error = NO_ERROR;
    std::stringstream message;
    int cols = 512;
    int rows = 2048;
    int vstart = 0;

    //SET CAMERAMODE TO FULLFRAME
    
    //set cds and params back to fullframe default
    // Update Archon parameters
    this->set_parameter("H2RG_columns " + std::to_string(cols), dummy);
    this->set_parameter("H2RG_rows " + std::to_string(rows), dummy);
    this->set_parameter("H2RG_rows_skip " + std::to_string(vstart), dummy);
    this->record_h2rg_window(cols, vstart);

    // take detector out of window mode: // 0111 000000001100
    if (this->is_window) {
      error = this->send_inreg_clocked(this->lvds_module, 1, 28684);
      this->is_window = false;
         
    }   

    //Check mode and set cds variables
    // Update CDS geometry via config keys
    bool changed = false;
    int pixelcount = cols;
    auto &mode = this->controller->modemap[this->controller->selectedmode];
    this->controller->write_config_key("PIXELCOUNT", pixelcount, changed);
    if (changed) this->controller->send_cmd(APPLYCDS);
    this->controller->write_config_key("LINECOUNT", rows, changed);
    if (changed) this->controller->send_cmd(APPLYCDS);

    // Reset the ROI to fullframe and taplines
    this->win_hstart = 0;
    this->win_hstop = 2047;
    this->win_vstart = 0;
    this->win_vstop = 2047;
    this->camera_info.region_of_interest = {
      static_cast<uint32_t>(this->win_hstart),
      static_cast<uint32_t>(this->win_hstop),
      static_cast<uint32_t>(this->win_vstart),
      static_cast<uint32_t>(this->win_vstop)
    };
    //set detector.pixels = taplines * pixelcount
    this->camera_info.detector_pixels = {
      static_cast<uint32_t>(this->taplines_store * pixelcount),
      static_cast<uint32_t>(rows)
    };

    error = this->ArchonInterface::acfmode("FULLFRAME", retstring);

    return error;
  }
  /**** Camera::HispecTrackingCamera::fullframe ******************************/

  /**** Camera::HispecTrackingCamera::validate_roi ******************************/
  /**
   * @brief      validates the ROI arguments
   * @details    This function checks if the ROI arguments are valid.
   * @param[in]  args       4 arguments (vstart vstop hstart hstop) in pixels
   * @param[out] retstring  error message if validation fails
   * @return     ERROR|NO_ERROR
   *
   */
  long HispecTrackingCamera::validate_roi(const std::string &args, std::string &retstring) {
    const std::string function("Camera::HispecTrackingCamera::validate_roi");
    long error = NO_ERROR;
    std::stringstream message;
    std::vector<std::string> tokens;
    int vstart, vstop, hstart, hstop, height, width;
    Tokenize(args, tokens, " ");
    if (tokens.size() != 4 && tokens.size() != 2) {
        return this->fail_detailed(function, retstring, "wrong ROI argument count",
                   "got "+std::to_string(tokens.size())+", expected 2 (<height> <width>) "
                   "or 4 (<vstart> <vstop> <hstart> <hstop>)");
    }
    if (tokens.size() == 2) {
      // Check if the two arguments are valid height and width
      try {
          height = std::stoi(tokens[0]);
          width = std::stoi(tokens[1]);
          if (height <= 0 || width <= 0) {
              return this->fail_detailed(function, retstring, "ROI must be positive",
                         "height="+tokens[0]+" width="+tokens[1]+"; both must exceed 0");
          } else if (height > 2048 || width > 1024) {
              return this->fail_detailed(function, retstring, "ROI exceeds the detector",
                         "height="+tokens[0]+" width="+tokens[1]+
                         "; limits are height<=2048 and width<=1024");
          }
      } catch (const std::invalid_argument &) {
          return this->fail_detailed(function, retstring, "ROI not numeric",
                     "could not parse \""+args+"\" as <height> <width>");
      } catch (const std::out_of_range &) {
          return this->fail_detailed(function, retstring, "ROI exceeds the detector",
                     "\""+args+"\" is too large to be a pixel count");
      }
    } else {
      // Check if the four arguments are valid vstart, vstop, hstart, hstop
      try {
          vstart = std::stoi(tokens[0]);
          vstop = std::stoi(tokens[1]);
          hstart = std::stoi(tokens[2]);
          hstop = std::stoi(tokens[3]);
      } catch (const std::invalid_argument &) {
          return this->fail_detailed(function, retstring, "ROI not numeric",
                     "could not parse \""+args+"\" as four integers");
      } catch (const std::out_of_range &) {
          return this->fail_detailed(function, retstring, "ROI outside the detector",
                     "\""+args+"\" is too large to be a pixel index");
      }
      // Validate values are within detector
      if ( vstart < 0 || vstop > 2047 || hstart < 0 || hstop > 2047) {
          return this->fail_detailed(function, retstring, "ROI outside the detector",
                     "\""+args+"\" must satisfy 0<=vstart, vstop<=2047, 0<=hstart, hstop<=2047");
      }
      // Validate values have proper ordering
      if (vstart >= vstop || hstart >= hstop) {
          return this->fail_detailed(function, retstring, "ROI bounds out of order",
                     "\""+args+"\" must satisfy vstart<vstop and hstart<hstop");
      }
    }
    retstring = message.str();
    return error;
  }
  /***** Camera::HispecTrackingCamera::validate_roi ******************************/

  /***** Camera::HispecTrackingCamera::window_mode ***************************/
  /**
   * @brief      toggle H2RG window/guiding mode on or off
   * @details    Entering window mode:
   *             - Sets detector into window mode via INREG (28687 = 0111 000000001111)
   *             - Saves current tapline config, switches to GUIDING camera mode
   *             - Sets single tapline (AM33L,1,0) and updates CDS geometry
   *             Leaving window mode:
   *             - Sets detector out of window mode via INREG (28684 = 0111 000000001100)
   *             - Restores taplines, switches back to DEFAULT camera mode
   *             - Issues Abort parameter to complete the mode exit
   * @param[in]  args       "true"|"1" to enable, "false"|"0" to disable, empty to query
   * @param[out] retstring  current window state ("true" or "false")
   * @return     ERROR|NO_ERROR
   *
   */
  long HispecTrackingCamera::window_mode(const std::string &args, std::string &retstring) {
    const std::string function("Camera::HispecTrackingCamera::window_mode");
    long error = NO_ERROR;
    std::string dummy;

    if (!args.empty()) {
      std::string state = args;
      std::transform(state.begin(), state.end(), state.begin(), ::toupper);

      if (state == "FALSE" || state == "0") {
        this->discard_pre_interleave_readout();
        this->is_window = false;

        // Set detector out of window mode: 0111 000000001100 = 28684
        error = this->send_inreg_clocked(this->lvds_module, 1, 28684);

        // Restore taplines
        if (error == NO_ERROR) {
          bool changed = false;
          this->controller->write_config_key("TAPLINES", this->taplines_store, changed);
          if (changed) this->controller->send_cmd(APPLYCDS);
          this->controller->write_config_key("TAPLINE0", this->tapline0_store.c_str(), changed);
          if (changed) this->controller->send_cmd(APPLYCDS);
        }

        // Switch back to DEFAULT mode — resets internal buffer geometries
        if (error == NO_ERROR) error = this->set_camera_mode("DEFAULT");

        // Reset CDS to DEFAULT mode geometry
        if (error == NO_ERROR) {
          auto &mode = this->controller->modemap["DEFAULT"];
          bool changed = false;
          this->controller->write_config_key("PIXELCOUNT", mode.geometry.pixelcount, changed);
          if (changed) this->controller->send_cmd(APPLYCDS);
          this->controller->write_config_key("LINECOUNT", mode.geometry.linecount, changed);
          if (changed) this->controller->send_cmd(APPLYCDS);
        }

        // Issue Abort to complete window mode exit
        if (error == NO_ERROR) {
          this->set_parameter("Abort 1", dummy);
        }

        if (error == NO_ERROR) logwrite(function, "window mode disabled");
      }
      else if (state == "TRUE" || state == "1") {
        this->discard_pre_interleave_readout();
        this->is_window = true;

        // Set detector into window mode: 0111 000000001111 = 28687
        error = this->send_inreg_clocked(this->lvds_module, 1, 28687);

        // Save current tapline configuration before switching
        if (error == NO_ERROR) {
          this->controller->get_configmap_value("TAPLINES", this->taplines_store);

          auto it = this->controller->configmap.find("TAPLINE0");
          if (it != this->controller->configmap.end()) {
            this->tapline0_store = it->second.value;
          }
        }

        // Switch to GUIDING camera mode
        if (error == NO_ERROR) error = this->set_camera_mode("GUIDING");

        // Set single tapline for window mode
        if (error == NO_ERROR) {
          bool changed = false;
          this->controller->write_config_key("TAPLINES", 1, changed);
          if (changed) this->controller->send_cmd(APPLYCDS);
          this->controller->write_config_key("TAPLINE0", "AM33L,1,0", changed);
          if (changed) this->controller->send_cmd(APPLYCDS);
        }

        // Set window dimensions from current ROI
        if (error == NO_ERROR) {
          const int rows = (this->win_vstop - this->win_vstart) + 1;
          const int cols = (this->win_hstop - this->win_hstart) + 1;

          this->set_parameter("H2RG_win_columns " + std::to_string(cols), dummy);
          this->set_parameter("H2RG_win_rows " + std::to_string(rows), dummy);

          // Update CDS geometry
          bool changed = false;
          this->controller->write_config_key("PIXELCOUNT", cols, changed);
          if (changed) this->controller->send_cmd(APPLYCDS);
          this->controller->write_config_key("LINECOUNT", rows, changed);
          if (changed) this->controller->send_cmd(APPLYCDS);

          // Update modemap and camera_info
          auto &modeinfo = this->controller->modemap[this->controller->selectedmode];
          modeinfo.geometry.linecount = rows;
          modeinfo.geometry.pixelcount = cols;
          this->camera_info.region_of_interest = {
            static_cast<uint32_t>(this->win_hstart),
            static_cast<uint32_t>(this->win_hstop),
            static_cast<uint32_t>(this->win_vstart),
            static_cast<uint32_t>(this->win_vstop)
          };
          this->camera_info.detector_pixels = {
            static_cast<uint32_t>(cols),
            static_cast<uint32_t>(rows)
          };

          this->camera_info.set_axes(16);
        }

        if (error == NO_ERROR) logwrite(function, "window mode enabled");
      }
      else {
        return this->fail_detailed(function, retstring, "invalid argument",
                   "window_mode expects true|1|false|0 but got \""+args+"\"");
      }
    }

    retstring = this->is_window ? "true" : "false";
    if (error != NO_ERROR) {
      logwrite(function, "ERROR setting window mode");
    }
    return error;
  }
  /***** Camera::HispecTrackingCamera::window_mode ***************************/

  /***** Camera::HispecTrackingCamera::interleave_science ********************/
  /**
   * @brief      set or report the science window of the interleaved readout
   * @details    Centred on the detector like "roi <height> <width>". Only
   *             records the window; it reaches the controller at the next
   *             expose, so the interleave commands can come in any order.
   * @param[in]  args       "<height> <width>" in detector pixels, or empty to query
   * @param[out] retstring  current window as "<height> <width>"
   * @return     ERROR|NO_ERROR|HELP
   *
   */
  long HispecTrackingCamera::interleave_science(const std::string &args, std::string &retstring) {
    const std::string function("Camera::HispecTrackingCamera::interleave_science");

    if (args=="?" || args=="help") {
      retstring = "interleave_science [ <height> <width> ]\n";
      retstring.append( "  Set or report the science window of the interleaved readout, in\n" );
      retstring.append( "  detector pixels, centred on the detector like \"roi <height> <width>\".\n" );
      retstring.append( "  <width> is split across two amplifiers, so it must be even; the\n" );
      retstring.append( "  reference channel adds a third block of <width>/2 columns to each\n" );
      retstring.append( "  line. <height> must be a whole multiple of interleave_set. Takes\n" );
      retstring.append( "  effect at the next expose, and cannot be changed while an\n" );
      retstring.append( "  interleaved exposure is running. Starts as 128 128. With no\n" );
      retstring.append( "  argument, returns the current window.\n" );
      return HELP;
    }

    const std::lock_guard<std::mutex> lock(this->interleave_mutex);

    if (!args.empty()) {
      if (this->is_interleave_frozen()) {
        return this->fail_detailed(function, retstring, "exposure in progress",
                   "the science window is fixed while an interleaved exposure runs; "
                   "change it once the exposure has finished");
      }
      std::vector<std::string> tokens;
      Tokenize(args, tokens, " ");
      if (tokens.size() != 2) {
        return this->fail_detailed(function, retstring, "wrong argument count",
                   "got "+std::to_string(tokens.size())+" argument(s) \""+args+
                   "\"; expected <height> <width>");
      }
      const auto values = parse_ints(tokens);
      if (!values) {
        return this->fail_detailed(function, retstring, "arguments not numeric",
                   "could not parse \""+args+"\" as two integers <height> <width>");
      }
      if (this->validate_roi(args, retstring) != NO_ERROR) return ERROR;

      const int height = (*values)[0];
      const int width  = (*values)[1];
      if (width % INTERLEAVE_SCIENCE_AMPS != 0) {
        return this->fail_detailed(function, retstring, "science width must be even",
                   "width="+tokens[1]+" cannot be split evenly across "+
                   std::to_string(INTERLEAVE_SCIENCE_AMPS)+" amplifiers");
      }

      this->interleave_geometry.science_rows           = height;
      this->interleave_geometry.science_pixels_per_tap = width / INTERLEAVE_SCIENCE_AMPS;
      this->interleave_geometry.science_start_row      = (this->detector_rows() - height) / 2;
    }

    retstring = std::to_string(this->interleave_geometry.science_rows) + " " +
                std::to_string(this->interleave_geometry.science_pixels_per_tap * INTERLEAVE_SCIENCE_AMPS);
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
   * @param[in]  module  module number
   * @param[in]  inreg   input register number
   * @param[in]  value   register value
   * @return     ERROR|NO_ERROR|BUSY
   *
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
   * @brief      write the H2RG window bound registers for the guide window
   * @details    Sets where the window is and nothing else: the firmware decides
   *             when the detector reads it. Stops at the first write that fails.
   * @param[in]  vstart  first row, in detector pixels
   * @param[in]  vstop   last row, inclusive
   * @param[in]  hstart  first column
   * @param[in]  hstop   last column, inclusive
   * @return     ERROR|NO_ERROR|BUSY
   *
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
   * @details    Writes the same bound registers as "roi <vstart> <vstop>
   *             <hstart> <hstop>" but not the window mode register: the
   *             firmware switches the detector into window mode for each guide
   *             read and back out for the science rows.
   *
   *             Safe to send while frames are being read, so the window can
   *             follow a star during an exposure. The writes keep clear of the
   *             frame traffic, one mover runs at a time, and a move that fails
   *             part way puts the previous window back.
   * @param[in]  args       "<vstart> <vstop> <hstart> <hstop>" in detector pixels, or empty to query
   * @param[out] retstring  current window as "<vstart> <vstop> <hstart> <hstop>"
   * @return     ERROR|NO_ERROR|HELP
   *
   */
  long HispecTrackingCamera::interleave_guide(const std::string &args, std::string &retstring) {
    const std::string function("Camera::HispecTrackingCamera::interleave_guide");

    if (args=="?" || args=="help") {
      retstring = "interleave_guide [ <vstart> <vstop> <hstart> <hstop> ]\n";
      retstring.append( "  Set, move or report the guide window of the interleaved readout, in\n" );
      retstring.append( "  detector pixels {0:2047} with inclusive bounds. Writes the same\n" );
      retstring.append( "  detector registers as \"roi <vstart> <vstop> <hstart> <hstop>\" but\n" );
      retstring.append( "  leaves the detector out of window mode; the firmware switches it\n" );
      retstring.append( "  for each guide read. The window may sit anywhere on the detector.\n" );
      retstring.append( "  Can be sent during an exposure to move the window, though not to\n" );
      retstring.append( "  resize it; a move that fails part way restores the previous\n" );
      retstring.append( "  window. Needs the detector powered. Every expose writes the window\n" );
      retstring.append( "  again, since a power cycle clears the detector's registers. Its\n" );
      retstring.append( "  width must fit in the science window's columns per amplifier,\n" );
      retstring.append( "  which is checked at the next expose. Starts as a 10 by 10 window\n" );
      retstring.append( "  at the centre of the detector. With no argument, returns the\n" );
      retstring.append( "  current window.\n" );
      return HELP;
    }

    // One mover at a time: two register sequences interleaved on the wire
    // would leave the detector holding parts of both windows
    const std::lock_guard<std::mutex> lock(this->interleave_mutex);
    auto &geometry = this->interleave_geometry;

    if (!args.empty()) {
      std::vector<std::string> tokens;
      Tokenize(args, tokens, " ");
      if (tokens.size() != 4) {
        return this->fail_detailed(function, retstring, "wrong argument count",
                   "got "+std::to_string(tokens.size())+" argument(s) \""+args+
                   "\"; expected <vstart> <vstop> <hstart> <hstop>");
      }
      const auto values = parse_ints(tokens);
      if (!values) {
        return this->fail_detailed(function, retstring, "arguments not numeric",
                   "could not parse \""+args+"\" as four integers "
                   "<vstart> <vstop> <hstart> <hstop>");
      }
      if (this->validate_roi(args, retstring) != NO_ERROR) return ERROR;

      // Checked before anything is written: the Archon accepts these writes
      // on behalf of a detector that cannot receive them
      if (!this->controller->is_connected) {
        return this->fail_detailed(function, retstring, "not connected",
                   "no socket is open to the Archon; run \"open\" first");
      }
      if (!this->controller->is_firmwareloaded) {
        return this->fail_detailed(function, retstring, "firmware not loaded",
                   "the register writes go through the ACF's configuration; run "
                   "\"load\" first");
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
                   " guide window; it can move, but not change size until the "
                   "exposure has finished");
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

    retstring = std::to_string(geometry.guide_vstart) + " " +
                std::to_string(geometry.guide_vstop) + " " +
                std::to_string(geometry.guide_hstart) + " " +
                std::to_string(geometry.guide_hstop);
    return NO_ERROR;
  }
  /***** Camera::HispecTrackingCamera::interleave_guide **********************/

  /***** Camera::HispecTrackingCamera::guide_position ************************/
  /**
   * @brief      where the guide window is now, and whether it is moving
   * @return     the window's bounds and its change count
   *
   */
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
   * @details    Only records the value; it reaches the controller at the next
   *             expose, where it is checked against the science window height.
   * @param[in]  args       "<rows>", or empty to query
   * @param[out] retstring  current row count
   * @return     ERROR|NO_ERROR|HELP
   *
   */
  long HispecTrackingCamera::interleave_set(const std::string &args, std::string &retstring) {
    const std::string function("Camera::HispecTrackingCamera::interleave_set");

    if (args=="?" || args=="help") {
      retstring = "interleave_set [ <rows> ]\n";
      retstring.append( "  Set or report how many science rows the interleaved readout reads\n" );
      retstring.append( "  before each guide window. Fewer rows give a faster guide cadence\n" );
      retstring.append( "  and more guide frames per pass over the science window. The science\n" );
      retstring.append( "  window height must be a whole multiple of it, which is checked at\n" );
      retstring.append( "  the next expose. Cannot be changed while an interleaved exposure\n" );
      retstring.append( "  is running. Starts as 4. With no argument, returns the current\n" );
      retstring.append( "  value.\n" );
      return HELP;
    }

    const std::lock_guard<std::mutex> lock(this->interleave_mutex);

    if (!args.empty()) {
      if (this->is_interleave_frozen()) {
        return this->fail_detailed(function, retstring, "exposure in progress",
                   "the set size is fixed while an interleaved exposure runs; "
                   "change it once the exposure has finished");
      }
      const auto rows = parse_int(trim(args));
      if (!rows || *rows < 1 || *rows > this->detector_rows()) {
        return this->fail_detailed(function, retstring, "invalid row count",
                   "\""+args+"\" must be an integer in {1:"+
                   std::to_string(this->detector_rows())+"}");
      }
      this->interleave_geometry.rows_per_set = *rows;
    }

    retstring = std::to_string(this->interleave_geometry.rows_per_set);
    return NO_ERROR;
  }
  /***** Camera::HispecTrackingCamera::interleave_set ************************/

  /***** Camera::HispecTrackingCamera::apply_interleave **********************/
  /**
   * @brief      send the interleaved readout's settings to the controller
   * @details    Called by expose() ahead of the trigger, so that what the
   *             interleave commands recorded reaches the Archon as one
   *             consistent set whatever order they came in. Checks the rules
   *             that span more than one command, which no single command can.
   *
   *             The guide window is written again every time, because the
   *             detector forgets it on a power cycle or a reset.
   * @param[out] retstring  reason on failure
   * @return     ERROR|NO_ERROR
   *
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
    bool changed = false;
    long error = this->controller->write_config_key("PIXELCOUNT", pixelcount, changed);
    if (error == NO_ERROR) error = this->controller->write_config_key("LINECOUNT", linecount, changed);
    if (error == NO_ERROR && changed) error = this->controller->send_cmd(APPLYCDS);
    if (error != NO_ERROR) {
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


  /***** Camera::HispecTrackingCamera::match_pixelcount_to_readout ***********/
  /**
   * @brief      set PIXELCOUNT for a switch into or out of the rxr readout
   * @details    Derived from H2RG_columns as last written rather than from
   *             PIXELCOUNT, so a geometry set while in rxr by a command that
   *             does not double it is not halved on the way out, and entering
   *             rxr twice does not double it twice. In window mode, where
   *             H2RG_columns does not apply, the current width is scaled.
   * @param[in]  was_rxr  whether the readout being left was rxr
   * @return     ERROR|NO_ERROR
   *
   */
  long HispecTrackingCamera::match_pixelcount_to_readout(bool was_rxr) {
    const bool is_rxr = (this->cur_exposure_mode == RXR_READOUT);
    if (is_rxr == was_rxr) return NO_ERROR;

    auto &mode = this->controller->modemap[this->controller->selectedmode];
    std::optional<int> columns;
    if (!this->is_window) {
      const std::lock_guard<std::mutex> lock(this->interleave_mutex);
      if (this->h2rg_window) columns = this->h2rg_window->columns;
    }
    const int current = mode.geometry.pixelcount;
    const int detector_columns = columns.value_or(was_rxr ? current / RXR_SAMPLES_PER_PIXEL : current);
    const int pixelcount = is_rxr ? detector_columns * RXR_SAMPLES_PER_PIXEL : detector_columns;

    bool changed = false;
    long error = this->controller->write_config_key("PIXELCOUNT", pixelcount, changed);
    if (error == NO_ERROR && changed) error = this->controller->send_cmd(APPLYCDS);
    if (error == NO_ERROR) mode.geometry.pixelcount = pixelcount;
    return error;
  }
  /***** Camera::HispecTrackingCamera::match_pixelcount_to_readout ***********/


  /***** Camera::HispecTrackingCamera::clear_controller_abort *****************/
  /**
   * @brief      clear the controller's abort parameter ahead of a trigger
   * @details    Nothing in the sequencer consumes an abort sent while it is
   *             idle, so it stays set and cuts the next exposure short.
   * @param[out] retstring  reason on failure
   * @return     ERROR|NO_ERROR
   *
   */
  long HispecTrackingCamera::clear_controller_abort(std::string &retstring) {
    const std::string function("Camera::HispecTrackingCamera::clear_controller_abort");
    const std::string &abort_param = this->controller->abort_param;
    if (abort_param.empty()) return NO_ERROR;
    if (this->set_parameter_offstream(abort_param, 0) != NO_ERROR) {
      return this->fail_detailed(function, retstring, "cannot clear abort",
                 "the controller rejected "+abort_param+"=0; ABORT_PARAM in the .cfg must "
                 "match the ACF parameter name exactly, it is case sensitive");
    }
    return NO_ERROR;
  }
  /***** Camera::HispecTrackingCamera::clear_controller_abort *****************/


  /***** Camera::HispecTrackingCamera::prepare_trigger ************************/
  /**
   * @brief      the controller writes every exposure path makes before its trigger
   * @details    apply_interleave() clears the abort itself, after checking its
   *             settings, so that a refused exposure writes nothing.
   *
   */
  long HispecTrackingCamera::prepare_trigger(std::string &retstring) {
    return this->is_interleave() ? this->apply_interleave(retstring)
                                 : this->clear_controller_abort(retstring);
  }
  /***** Camera::HispecTrackingCamera::prepare_trigger ************************/

  /***** Camera::HispecTrackingCamera::arm_interleave_pass ********************/
  /**
   * @brief      point the sequencer at the start of the science window
   * @details    Called by the acquisition thread ahead of every trigger. Reads
   *             only settings that are frozen while an exposure runs, so it
   *             takes no lock and never waits on a guide window move.
   * @return     ERROR|NO_ERROR
   *
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
   *
   */
  void HispecTrackingCamera::record_h2rg_window(int columns, int rows_skip) {
    this->discard_pre_interleave_readout();
    const std::lock_guard<std::mutex> lock(this->interleave_mutex);
    this->h2rg_window = H2rgWindow{columns, rows_skip};
  }
  /***** Camera::HispecTrackingCamera::record_h2rg_window *********************/


  /***** Camera::HispecTrackingCamera::discard_pre_interleave_readout *********/
  /**
   * @brief      drop the saved geometry for a command that sets its own
   * @details    The camera mode's stored geometry is still put back: commands
   *             such as mode read it, and would otherwise push the interleaved
   *             frame shape. Call before the command sets its geometry.
   *
   */
  void HispecTrackingCamera::discard_pre_interleave_readout() {
    const std::lock_guard<std::mutex> lock(this->interleave_mutex);
    if (!this->pre_interleave_readout) return;
    const SavedReadout &saved = *this->pre_interleave_readout;
    const auto mode = this->controller->modemap.find(saved.camera_mode);
    if (mode != this->controller->modemap.end()) {
      mode->second.geometry.pixelcount = saved.pixelcount;
      mode->second.geometry.linecount  = saved.linecount;
    }
    this->pre_interleave_readout.reset();
  }
  /***** Camera::HispecTrackingCamera::discard_pre_interleave_readout *********/


  /***** Camera::HispecTrackingCamera::current_h2rg_window ********************/
  /**
   * @brief      the H2RG window the detector is set to read, if known
   * @details    Caller holds interleave_mutex. Before any command has written
   *             the window, the controller still holds the ACF's values.
   *
   */
  std::optional<HispecTrackingCamera::H2rgWindow> HispecTrackingCamera::current_h2rg_window() const {
    if (this->h2rg_window) return this->h2rg_window;

    const auto acf_value = [this](const std::string &name) -> std::optional<int> {
      const auto param = this->controller->parammap.find(name);
      if (param == this->controller->parammap.end()) return std::nullopt;
      const std::string &text = param->second.value;
      int value = 0;
      const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
      if (error != std::errc{} || end != text.data() + text.size()) return std::nullopt;
      return value;
    };
    const auto columns   = acf_value("H2RG_columns");
    const auto rows_skip = acf_value("H2RG_rows_skip");
    if (!columns || !rows_skip) return std::nullopt;
    return H2rgWindow{*columns, *rows_skip};
  }
  /***** Camera::HispecTrackingCamera::current_h2rg_window ********************/


  /***** Camera::HispecTrackingCamera::restore_pre_interleave_readout *********/
  /**
   * @brief      put back what apply_interleave() overwrote
   * @details    On failure the saved geometry is kept, so leaving the readout
   *             can be tried again.
   * @param[out] retstring  reason on failure
   * @return     ERROR|NO_ERROR
   *
   */
  long HispecTrackingCamera::restore_pre_interleave_readout(std::string &retstring) {
    const std::string function("Camera::HispecTrackingCamera::restore_pre_interleave_readout");
    const std::lock_guard<std::mutex> lock(this->interleave_mutex);
    if (!this->pre_interleave_readout) return NO_ERROR;
    const SavedReadout &saved = *this->pre_interleave_readout;

    const auto mode = this->controller->modemap.find(saved.camera_mode);
    if (mode == this->controller->modemap.end()) {
      this->pre_interleave_readout.reset();
      logwrite(function, "WARNING camera mode "+saved.camera_mode+" is gone, so its "
                         "geometry was not restored; run mode or roi before the next exposure");
      return NO_ERROR;
    }
    mode->second.geometry.pixelcount = saved.pixelcount;
    mode->second.geometry.linecount  = saved.linecount;

    // A mode switch that failed part way leaves another mode selected, whose
    // geometry the controller is not using the saved values for
    if (saved.camera_mode != this->controller->selectedmode) {
      this->pre_interleave_readout.reset();
      logwrite(function, "WARNING camera mode changed from "+saved.camera_mode+" to "+
                         this->controller->selectedmode+" since the interleaved exposure; "
                         "only the modemap geometry was restored");
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

    bool changed = false;
    long error = this->controller->write_config_key("PIXELCOUNT", saved.pixelcount, changed);
    if (error == NO_ERROR) error = this->controller->write_config_key("LINECOUNT", saved.linecount, changed);
    if (error == NO_ERROR && changed) error = this->controller->send_cmd(APPLYCDS);
    if (error != NO_ERROR) {
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
