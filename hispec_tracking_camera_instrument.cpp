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
}
