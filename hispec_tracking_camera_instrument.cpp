/**
 * @file    Instruments/hispec_tracking_camera/hispec_tracking_camera_instrument.cpp
 * @brief   implementation for HISPEC Tracking Camera-specific properties
 * @author  Michael Langmayr <langmayr@astro.caltech.edu>
 *
 */

#include "hispec_tracking_camera_instrument.h"
#include "hispec_tracking_camera_exposure_modes.h"
#include <iterator>

namespace Camera {

  const std::unordered_map<std::string, HispecTrackingCamera::CmdHandler>
  HispecTrackingCamera::command_handlers_ = {
    {"h2rg_init",   &HispecTrackingCamera::h2rg_init},
    {"window_mode", &HispecTrackingCamera::window_mode},
    {"roi",  &HispecTrackingCamera::roi},
    {"exposure", &HispecTrackingCamera::_exposure_mode},
    {"mode", &HispecTrackingCamera::mode}
  };
  const std::unordered_map<std::string, std::string>
  HispecTrackingCamera::_exposure_modes = {
    {"utr_rr", "mode_utr_rr"},
    {"utr_gr", "mode_utr_gr"},
    {"rx", "mode_rx"},
    {"rxr", "mode_rxr"}
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
    long error = this->ArchonInterface::set_camera_mode(args, retstring);
    if (error != NO_ERROR) {
      logwrite(function, "ERROR setting camera mode to " + args);
      retstring = "error";
      return ERROR;
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
        logwrite(function, errstr.str());
        retstring = "error";
        return ERROR;
      }
    }

    // Activate the staged tapline/readout geometry in the CDS core. APPLYCDS
    // reconfigures readout without power-cycling the detector (unlike APPLYALL).
    if (changed && this->controller->send_cmd(APPLYCDS) != NO_ERROR) {
      logwrite(function, "ERROR applying tapline configuration (APPLYCDS)");
      retstring = "error";
      return ERROR;
    }

    // mode.tapinfo (num_taps, ampname, readoutdir, gain, offset) is already
    // populated for every mode at ACF-load time (ArchonController::parse_tapinfo),
    // so it is available here via modemap[selectedmode] without re-parsing.

    logwrite(function, "Camera mode set to " + this->controller->selectedmode);
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

    // validate the arguments
    auto req_param = _exposure_modes.find(args);
    if (req_param == _exposure_modes.end()) {
      retstring = "Current exposure mode: " + this->cur_exposure_mode;
      retstring += "\nProvide a valid exposure mode to switch to.";
      logwrite(function, retstring);
      return error;
    }
    const std::string &mode_name = req_param->first;
    const std::string &mode_value = req_param->second;

    // iterate through the arguments and set param to 0
    for (const auto &[key, value] : _exposure_modes) {
      param_cmd = value + " 0";
      error = this->set_parameter(param_cmd, dummy);
    }

    // set current exposure mode
    param_cmd = mode_value + " 1";
    if (error == NO_ERROR) error = this->set_parameter(param_cmd, dummy);

    // update the exposure mode in the camera
    this->cur_exposure_mode = mode_name;

    // update cds if needed
    if (this->cur_exposure_mode == "rxr") {
      bool changed = false;
      auto &mode = this->controller->modemap[this->controller->selectedmode];
      int pixelcount = mode.geometry.pixelcount * 2;
      if (error == NO_ERROR) error = this->controller->write_config_key("PIXELCOUNT", pixelcount, changed);
      if (changed) this->controller->send_cmd(APPLYCDS);
      mode.geometry.pixelcount = pixelcount;
    }
    //return retstring and error
    retstring = "attempted to set exposure mode to " + mode_name;
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
    const std::string function("Camera::HispecTrackingCamera::guiding_roi");
    long error = NO_ERROR;
    std::string upper_args = args;
    std::transform( upper_args.begin(), upper_args.end(), upper_args.begin(), ::toupper );  // make uppercase

    // Handle different argument counts
    if (args.size() == 4) {
      return this->guiding_roi(args, retstring);
    } else if (args.size() == 2) {
      return this->roi_exec(args, retstring);
    } else if (args.size() == 1 && upper_args == "FULLFRAME") {
      return this->fullframe(args, retstring);
    } else {
      //query current ROI
      retstring = "Current ROI: " + std::to_string(this->win_hstart) + ", " + std::to_string(this->win_vstart) + ", " + std::to_string(this->win_hstop) + ", " + std::to_string(this->win_vstop);
      retstring += "\nProvide arguments to execute ROI command.";
      return NO_ERROR;
    }
    return error;
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
    this->camera_info.detector_pixels = {
      static_cast<uint32_t>(pixelcount),
      static_cast<uint32_t>(rows)
    };
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
        logwrite(function, "ERROR unable to convert geometry values: " + args);
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
        this->set_parameter("H2RG_columns " + std::to_string(cols), dummy);
        this->set_parameter("H2RG_rows " + std::to_string(rows), dummy);
        this->set_parameter("H2RG_rows_skip 0", dummy);

        // Update CDS geometry via config keys
        bool changed = false;
        int pixelcount = cols;
        auto &mode = this->controller->modemap[this->controller->selectedmode];
        if (this->cur_exposure_mode == "rxr") {
          pixelcount = cols * 2;
        }
        this->controller->write_config_key("PIXELCOUNT", cols, changed);
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
        message.str(""); message << "param expected 4 or 2 arguments (vstart, vstop, hstart, hstop) || (Height and Width) but got " << tokens.size();
        retstring = message.str();
        logwrite(function, message.str() );
        return ERROR;
    }
    if (tokens.size() == 2) {
      // Check if the two arguments are valid height and width
      try {
          height = std::stoi(tokens[0]);
          width = std::stoi(tokens[1]);
          if (height <= 0 || width <= 0) {
              message.str(""); message << "Height and width must be positive integers";
              retstring = message.str();
              logwrite(function, message.str());
              return ERROR;
          } else if (height > 2048 || width > 1024) {
              message.str(""); message << "Height and width must be within the detector range";
              retstring = message.str();
              logwrite(function, message.str());
              return ERROR;
          }
      } catch (std::invalid_argument &) {
          message.str(""); message << "Height and width must be valid integers";
          retstring = message.str();
          logwrite(function, message.str());
          return ERROR;
      }
    } else {
      // Check if the four arguments are valid vstart, vstop, hstart, hstop
      try {
          vstart = std::stoi(tokens[0]);
          vstop = std::stoi(tokens[1]);
          hstart = std::stoi(tokens[2]);
          hstop = std::stoi(tokens[3]);
      } catch (std::invalid_argument &) {
          message.str(""); message << "vstart, vstop, hstart, hstop must be valid integers";
          retstring = message.str();
          logwrite(function, message.str());
          return ERROR;
      }
      // Validate values are within detector
      if ( vstart < 0 || vstop > 2047 || hstart < 0 || hstop > 2047) {
          message.str(""); message << "geometry values " << args << " outside pixel range";
          retstring = message.str();
          logwrite( function, message.str());
          return ERROR;
      }
      // Validate values have proper ordering
      if (vstart >= vstop || hstart >= hstop) {
          message.str(""); message << "geometry values " << args << " are not correctly ordered";
          retstring = message.str();
          logwrite( function, message.str());
          return ERROR;
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
