/**
 * @file    Instruments/hispec_tracking_camera/hispec_tracking_camera_instrument.cpp
 * @brief   implementation for HISPEC Tracking Camera-specific properties
 * @author  Michael Langmayr <langmayr@astro.caltech.edu>
 *
 */

#include "hispec_tracking_camera_instrument.h"
#include "hispec_tracking_camera_exposure_modes.h"

#include <chrono>
#include <cmath>

namespace Camera {

  /***** Camera::HispecTrackingCamera::instrument_cmd *************************/
  /**
   * @brief      dispatcher for HISPEC Tracking Camera-specific instrument commands
   * @details    This allows dispatching instrument specific commands by receiving
   *             the command and args and calling the appropriate instrument
   *             specific function.
   * @param[in]  cmd        command
   * @param[in]  args       any number of arguments
   * @param[out] retstring  return string
   * @return     ERROR|NO_ERROR|HELP
   *
   */
  long HispecTrackingCamera::instrument_cmd(const std::string &cmd,
                              const std::string &args,
			      std::string &retstring) {
    if ( cmd == "readout" ) {
      return this->readout(args, retstring);
    }
    else
    if ( cmd == "expose" ) {
      return this->expose(args, retstring);
    }
    else
    if ( cmd == "h2rg_init" ) {
      return this->h2rg_init(args, retstring);
    }
    else
    if ( cmd == "window_mode" ) {
      return this->window_mode(args, retstring);
    }
    else
    if ( cmd == "window_roi" ) {
      return this->window_roi(args, retstring);
    }
    else {
      retstring = "unrecognized command";
      return ERROR;
    }
  }
  /***** Camera::HispecTrackingCamera::instrument_cmd *************************/


  /***** Camera::HispecTrackingCamera::configure_instrument *******************/
  /**
   * @brief      extract+apply instrument-specific parameters from config file
   * @throws     std::runtime_error
   *
   */
  void HispecTrackingCamera::configure_instrument() {
    const std::string function("Camera::HispecTrackingCamera::configure_instrument");
    logwrite(function, "");
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

    if (modein==HispecTrackingCameraExposureMode::TRACKING) {
      this->exposuremode = std::make_unique<ExposureModeTracking>(this);
    }
    else {
      return this->ArchonInterface::set_exposure_mode(modein, modeargs);
    }

    return NO_ERROR;
  }
  /***** Camera::HispecTrackingCamera::set_exposure_mode **********************/


  /***** Camera::HispecTrackingCamera::readout ********************************/
  /**
   * @brief      read a single frame from the Archon
   * @details    Two paths depending on autofetch state:
   *             - Normal: delegates to controller's read_frame() (sends FETCH)
   *             - Autofetch: reads streaming frame data directly from socket
   * @param[in]  args       unused
   * @param[out] retstring  return string
   * @return     ERROR|NO_ERROR
   *
   */
  long HispecTrackingCamera::readout(const std::string &args, std::string &retstring) {
    const std::string function("Camera::HispecTrackingCamera::readout");

    if (this->is_autofetch_mode) {
      long error = this->readout_autofetch();
      retstring = (error == NO_ERROR) ? "done" : "error";
      return error;
    }

    // Normal mode: use controller's standard FETCH-based read_frame
    char* imagebuf = this->controller->framebuf;
    long error = this->controller->read_frame(ArchonController::FRAME_IMAGE, imagebuf);
    if (error != NO_ERROR) {
      logwrite(function, "ERROR reading frame from controller");
    }
    retstring = (error == NO_ERROR) ? "done" : "error";
    return error;
  }
  /***** Camera::HispecTrackingCamera::readout ********************************/


  /***** Camera::HispecTrackingCamera::readout_autofetch **********************/
  /**
   * @brief      read a single autofetch frame from the Archon socket
   * @details    In autofetch mode the Archon streams frames continuously.
   *             Each frame is: 36-byte header + h*w*bytes_per_pixel of pixel data.
   *             The frame size is known from window geometry config, so we
   *             read the entire frame in one go and parse the header afterward.
   *
   *             Autofetch header format (36 bytes):
   *             <QFsffffffffwwwwhhhhtttttttttttttttt
   *             s=sample(0=16bit,1=32bit) f=frame# w=width h=height t=timestamp
   *
   * @return     ERROR|NO_ERROR
   *
   */
  long HispecTrackingCamera::readout_autofetch() {
    const std::string function("Camera::HispecTrackingCamera::readout_autofetch");
    auto fetch_start = std::chrono::steady_clock::now();

    auto* mode = &this->controller->modemap[this->controller->selectedmode];
    const int num_detect = mode->geometry.num_detect;

    // Frame size is known from the configured window geometry
    const auto pixel_bytes = static_cast<size_t>(this->camera_info.image_memory * num_detect);
    const size_t frame_size = AUTOFETCH_HEADER_LEN + pixel_bytes;

    char* buf = this->controller->framebuf;

    // Read the entire frame (header + pixels) from the socket
    size_t total_read = 0;
    while (total_read < frame_size) {
      if (!this->controller->archon.is_readable(1000)) {
        logwrite(function, "ERROR timeout waiting for autofetch data");
        return ERROR;
      }

      int retval = this->controller->archon.Read(
        buf + total_read, static_cast<int>(frame_size - total_read));
      if (retval <= 0) {
        logwrite(function, "ERROR socket read failed");
        return ERROR;
      }
      total_read += static_cast<size_t>(retval);
    }

    // Parse the 36-byte autofetch header from the buffer
    try {
      std::string header(buf, AUTOFETCH_HEADER_LEN);
      int frame_number = std::stoi(header.substr(4, 8), nullptr, 16);
      uint64_t timestamp = std::stoull(header.substr(20, 16), nullptr, 16);

      this->controller->frameinfo.bufframen[this->controller->frameinfo.index] = frame_number;
      this->controller->lastframe = frame_number;

      // Track Archon timestamp deltas (timestamps are in 0.01 us units)
      if (this->prev_archon_ts != 0) {
        double delta_us = static_cast<double>(timestamp - this->prev_archon_ts) * 0.01;
        this->archon_ts_deltas.add(delta_us);
      }
      this->prev_archon_ts = timestamp;

    } catch (const std::exception &e) {
      logwrite(function, "ERROR parsing autofetch header: " + std::string(e.what()));
      return ERROR;
    }

    this->fetch_stats.record_since(fetch_start);
    return NO_ERROR;
  }
  /***** Camera::HispecTrackingCamera::readout_autofetch **********************/


  /***** Camera::HispecTrackingCamera::expose *********************************/
  /**
   * @brief      run an exposure/acquisition sequence
   * @details    Validates configuration, prepares buffers, loads exposure
   *             parameters, then loops calling readout() for each frame.
   *             Reports timing statistics on completion.
   * @param[in]  args       number of frames (sequence count)
   * @param[out] retstring  return string
   * @return     ERROR|NO_ERROR
   *
   */
  long HispecTrackingCamera::expose(const std::string &args, std::string &retstring) {
    const std::string function("Camera::HispecTrackingCamera::expose");

    if (!this->controller->is_camera_mode) {
      logwrite(function, "ERROR no mode selected");
      retstring = "error";
      return ERROR;
    }

    if (this->controller->expose_param.empty()) {
      logwrite(function, "ERROR EXPOSE_PARAM not defined in configuration");
      retstring = "error";
      return ERROR;
    }

    // Parse sequence count from args, default to 1
    int nseq = 1;
    if (!args.empty()) {
      try {
        nseq = std::stoi(args);
      } catch (const std::exception &e) {
        logwrite(function, "ERROR invalid sequence count: " + args);
        retstring = "error";
        return ERROR;
      }
    }

    // Allocate the frame buffer
    const auto image_memory = this->camera_info.image_memory;
    auto* mode = &this->controller->modemap[this->controller->selectedmode];
    const auto num_detect = mode->geometry.num_detect;
    const uint32_t bufsize = static_cast<uint32_t>(
      std::ceil(static_cast<double>(image_memory * num_detect + BLOCK_LEN - 1) / BLOCK_LEN) * BLOCK_LEN
    );

    if (this->controller->allocate_framebuf(bufsize) != NO_ERROR) {
      logwrite(function, "ERROR unable to allocate frame buffer");
      retstring = "error";
      return ERROR;
    }

    // Reset timing stats for this sequence
    this->fetch_stats.clear();
    this->archon_ts_deltas.clear();
    this->prev_archon_ts = 0;

    // Trigger the exposure sequence on the Archon
    long error = this->controller->prep_parameter(this->controller->expose_param, nseq);
    if (error == NO_ERROR) {
      error = this->controller->load_parameter(this->controller->expose_param, nseq);
    }
    if (error != NO_ERROR) {
      logwrite(function, "ERROR failed to initiate exposure");
      retstring = "error";
      return error;
    }

    // Main acquisition loop
    std::string dummy_ret;
    int frames_read = 0;

    for (int i = 0; i < nseq; ++i) {
      error = this->readout("", dummy_ret);
      if (error != NO_ERROR) {
        logwrite(function, "ERROR reading frame " + std::to_string(i + 1) + " of " + std::to_string(nseq));
        break;
      }
      ++frames_read;
    }

    // Report timing statistics
    if (!this->fetch_stats.empty()) {
      logwrite(function, this->fetch_stats.summary());
      if (!this->archon_ts_deltas.empty()) {
        logwrite(function, "archon timestamps: " + this->archon_ts_deltas.summary());
      }
    }

    logwrite(function, "sequence complete: " + std::to_string(frames_read) + " of " + std::to_string(nseq) + " frames");

    retstring = (error == NO_ERROR) ? "done" : "error";
    return error;
  }
  /***** Camera::HispecTrackingCamera::expose *********************************/


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

    // Enable output to Pad B and HIGHOHM: 0100 000000010010 = 16402
    long error = this->send_inreg_clocked(LVDS_MODULE, 1, 16402);
    if (error != NO_ERROR) {
      logwrite(function, "ERROR enabling Pad B output and HIGHOHM");
      retstring = "error";
      return ERROR;
    }

    logwrite(function, "H2RG initialized: Pad B output and HIGHOHM enabled");
    retstring = "done";
    return NO_ERROR;
  }
  /***** Camera::HispecTrackingCamera::h2rg_init *****************************/


  /***** Camera::HispecTrackingCamera::window_roi ****************************/
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
  long HispecTrackingCamera::window_roi(const std::string &args, std::string &retstring) {
    const std::string function("Camera::HispecTrackingCamera::window_roi");
    long error = NO_ERROR;

    if (!args.empty()) {
      std::vector<std::string> tokens;
      Tokenize(args, tokens, " ");

      if (tokens.size() != 4) {
        logwrite(function, "ERROR expected 4 arguments (vstart vstop hstart hstop) but got " +
                 std::to_string(tokens.size()));
        return ERROR;
      }

      int vstart, vstop, hstart, hstop;
      try {
        vstart = std::stoi(tokens[0]);
        vstop  = std::stoi(tokens[1]);
        hstart = std::stoi(tokens[2]);
        hstop  = std::stoi(tokens[3]);
      } catch (const std::exception &e) {
        logwrite(function, "ERROR unable to convert geometry values: " + args);
        return ERROR;
      }

      if (vstart < 0 || vstop > H2RG_MAX_PIXEL || hstart < 0 || hstop > H2RG_MAX_PIXEL) {
        logwrite(function, "ERROR geometry values outside pixel range [0:" +
                 std::to_string(H2RG_MAX_PIXEL) + "]");
        return ERROR;
      }

      if (vstart >= vstop || hstart >= hstop) {
        logwrite(function, "ERROR geometry values not correctly ordered");
        return ERROR;
      }

      // Set detector registers for each ROI limit
      // vstart: base address 32768
      error = this->send_inreg_clocked(LVDS_MODULE, 1, 32768 + vstart);
      if (error == NO_ERROR) this->win_vstart = vstart;

      // vstop: base address 36864
      if (error == NO_ERROR) error = this->send_inreg_clocked(LVDS_MODULE, 1, 36864 + vstop);
      if (error == NO_ERROR) this->win_vstop = vstop;

      // hstart: base address 40960
      if (error == NO_ERROR) error = this->send_inreg_clocked(LVDS_MODULE, 1, 40960 + hstart);
      if (error == NO_ERROR) this->win_hstart = hstart;

      // hstop: base address 45056
      if (error == NO_ERROR) error = this->send_inreg_clocked(LVDS_MODULE, 1, 45056 + hstop);
      if (error == NO_ERROR) this->win_hstop = hstop;

      // If window mode is active, update geometries to match
      if (error == NO_ERROR && this->is_window) {
        const int rows = (this->win_vstop - this->win_vstart) + 1;
        const int cols = (this->win_hstop - this->win_hstart) + 1;
        std::string dummy;

        // Update Archon parameters
        this->set_parameter("H2RG_win_columns " + std::to_string(cols), dummy);
        this->set_parameter("H2RG_win_rows " + std::to_string(rows), dummy);

        // Update CDS geometry via config keys
        bool changed = false;
        this->controller->write_config_key("PIXELCOUNT", cols, changed);
        if (changed) this->controller->send_cmd(APPLYCDS);
        this->controller->write_config_key("LINECOUNT", rows, changed);
        if (changed) this->controller->send_cmd(APPLYCDS);

        // Update modemap and camera_info
        auto &mode = this->controller->modemap[this->controller->selectedmode];
        mode.geometry.linecount = rows;
        mode.geometry.pixelcount = cols;
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

        // H2RG is 16-bit
        this->camera_info.set_axes(16);
      }

      if (error != NO_ERROR) {
        logwrite(function, "ERROR setting window geometry");
        return ERROR;
      }
    }

    retstring = std::to_string(this->win_vstart) + " " +
                std::to_string(this->win_vstop) + " " +
                std::to_string(this->win_hstart) + " " +
                std::to_string(this->win_hstop);
    return error;
  }
  /***** Camera::HispecTrackingCamera::window_roi ****************************/


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
        this->is_window = false;

        // Set detector out of window mode: 0111 000000001100 = 28684
        error = this->send_inreg_clocked(LVDS_MODULE, 1, 28684);

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
        this->is_window = true;

        // Set detector into window mode: 0111 000000001111 = 28687
        error = this->send_inreg_clocked(LVDS_MODULE, 1, 28687);

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
        logwrite(function, "ERROR unrecognized argument: " + args);
        retstring = "invalid_argument";
        return ERROR;
      }
    }

    retstring = this->is_window ? "true" : "false";
    if (error != NO_ERROR) {
      logwrite(function, "ERROR setting window mode");
    }
    return error;
  }
  /***** Camera::HispecTrackingCamera::window_mode ***************************/

}
