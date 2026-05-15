/**
 * @file    Instruments/hispec_tracking_camera/hispec_tracking_camera_instrument.cpp
 * @brief   implementation for HISPEC Tracking Camera-specific properties
 * @author  Michael Langmayr <langmayr@astro.caltech.edu>
 *
 */

#include "hispec_tracking_camera_instrument.h"
#include "hispec_tracking_camera_exposure_modes.h"

namespace Camera {

  const std::unordered_map<std::string, HispecTrackingCamera::CmdHandler>
  HispecTrackingCamera::command_handlers_ = {
    {"h2rg_init",   &HispecTrackingCamera::h2rg_init},
    {"window_mode", &HispecTrackingCamera::window_mode},
    {"window_roi",  &HispecTrackingCamera::window_roi},
  };

  /***** Camera::HispecTrackingCamera::is_instrument_command ******************/
  /**
   * @brief  true if cmd names an instrument-specific handler
   */
  bool HispecTrackingCamera::is_instrument_command(const std::string &cmd) {
    return command_handlers_.find(cmd) != command_handlers_.end();
  }
  /***** Camera::HispecTrackingCamera::is_instrument_command ******************/


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

    logwrite(function, "LVDS module=" + std::to_string(this->lvds_module) +
                       " H2RG max pixel=" + std::to_string(this->h2rg_max_pixel));

    // Optimize Archon socket for high-speed streaming
    constexpr int socket_buf_size = 1024 * 1024;  // 1 MB
    this->controller->archon.set_tcp_nodelay(true);
    this->controller->archon.set_recv_buf_size(socket_buf_size);
    this->controller->archon.set_send_buf_size(socket_buf_size);

    Camera::FrameOutputsConfig fo_cfg;
    fo_cfg.shm_enabled         = true;
    fo_cfg.shm_segment_name    = "hispec_tracking_camera";
    fo_cfg.shm_max_frame_bytes = static_cast<size_t>(
        (this->h2rg_max_pixel + 1) * (this->h2rg_max_pixel + 1) * 4);

    Camera::apply_config_overrides(fo_cfg, this->configfile);
    this->frame_outputs = Camera::make_frame_outputs(fo_cfg);
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

    if (modein==HispecTrackingCameraExposureMode::FULLFRAME) {
      this->exposuremode = std::make_unique<ExposureModeFullFrame>(this);
    }
    else {
      return this->ArchonInterface::set_exposure_mode(modein, modeargs);
    }

    return NO_ERROR;
  }
  /***** Camera::HispecTrackingCamera::set_exposure_mode **********************/


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
    long error = this->send_inreg_clocked(this->lvds_module, 1, 16402);
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

      if (vstart < 0 || vstop > this->h2rg_max_pixel || hstart < 0 || hstop > this->h2rg_max_pixel) {
        logwrite(function, "ERROR geometry values outside pixel range [0:" +
                 std::to_string(this->h2rg_max_pixel) + "]");
        return ERROR;
      }

      if (vstart >= vstop || hstart >= hstop) {
        logwrite(function, "ERROR geometry values not correctly ordered");
        return ERROR;
      }

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
