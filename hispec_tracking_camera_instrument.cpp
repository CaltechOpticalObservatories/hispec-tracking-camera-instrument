/**
 * @file    Instruments/hispec_tracking_camera/hispec_tracking_camera_instrument.cpp
 * @brief   implementation for HISPEC Tracking Camera-specific properties
 * @author  Michael Langmayr <langmayr@astro.caltech.edu>
 *
 */

#include "hispec_tracking_camera_instrument.h"
#include "hispec_tracking_camera_exposure_modes.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iterator>

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

    // Preferred when present, so new firmware carries its own timing
    constexpr const char* ACF_PIXEL_TIME_KEYS[] = {"TPIX", "TCLOCK"};

    constexpr double USEC_PER_SEC = 1.0e6;
    constexpr double MSEC_PER_SEC = 1.0e3;
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
    {"take_stats", &HispecTrackingCamera::_take_stats}
  };
  // FASTLOADPARAM is case-sensitive, so these must match the ACF exactly
  const std::unordered_map<std::string, std::string>
  HispecTrackingCamera::_exposure_modes = {
    {"utr_rr", "mode_UTR_RR"},
    {"utr_gr", "mode_UTR_GR"},
    {"rx", "mode_RX"},
    {"rxr", "mode_RXR"}
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


  /***** Camera::HispecTrackingCamera::_autofetch_mode ***********************/
  /**
   * @brief  toggle autofetch, then re-select the exposure pipeline so the
   *         autofetch/default class matches the new state
   */
  long HispecTrackingCamera::_autofetch_mode(const std::string &args, std::string &retstring) {
    long error = this->ArchonInterface::autofetch_mode(args, retstring);
    if (error == NO_ERROR && !args.empty()) {
      error = this->set_exposure_mode(this->default_exposure_mode_name(), {});
    }
    return error;
  }
  /***** Camera::HispecTrackingCamera::_autofetch_mode ***********************/


  /***** Camera::HispecTrackingCamera::expose ********************************/
  /**
   * @brief  Autofetch streams continuously, so run one producer session for all
   *         frames; non-autofetch keeps the base per-frame expose loop
   */
  long HispecTrackingCamera::expose(const std::string args, std::string &retstring) {
    const std::string function("Camera::HispecTrackingCamera::expose");
    long error = NO_ERROR;
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
      std::string dummy;
      if (this->set_parameter("abort 0", dummy) != NO_ERROR) {
        this->is_freerun_active.store(false);  // release the guard
        return this->fail_detailed(function, retstring, "cannot clear abort",
                   "the Archon rejected \"abort 0\"; ABORT_PARAM in the .cfg must "
                   "match the ACF parameter name exactly, it is case sensitive");
      }
      if (this->set_parameter("freerun 1", dummy) != NO_ERROR) {
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
   * @brief      pixel time in force, preferring the ACF's own value
   * @details    New firmware carrying its own timing wins over PIXEL_TIME_USEC
   *             from the config file.
   * @return     microseconds per pixel
   *
   */
  double HispecTrackingCamera::effective_pixel_time_usec() const {
    for (const auto* key : ACF_PIXEL_TIME_KEYS) {
      const auto entry = this->controller->configmap.find(key);
      if (entry == this->controller->configmap.end()) continue;
      try {
        const double acf_value = std::stod(entry->second.value);
        if (acf_value > 0) return acf_value;
      }
      catch (const std::exception &) { }  // unparsable, keep the configured value
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

    // Recomputed per command because roi and mode change the geometry
    const int timeout_msec = this->readout_timeout_msec();
    if (timeout_msec > 0) {
      this->controller->readout_time_msec = timeout_msec;
      logwrite(function, "readout deadline "+std::to_string(timeout_msec)+" msec per frame");
    }

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
                                 std::to_string(nseq)+" frame(s); the preceding log line "
                                 "gives the failing step");
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

    // sets the abort state and the Archon abort parameter; both freerun loops
    // watch the abort state and start winding down here
    long error = this->ArchonInterface::abort(args, retstring);

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
    std::string param_cmd = "freerun " +  value;  // e.g. "freerun 1" or "freerun 0"
    std::string dummy;
    error = this->set_parameter(param_cmd, dummy);
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
    long error = this->ArchonInterface::set_camera_mode(args, retstring);
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

    error = this->set_camera_mode("FULLFRAME", retstring);

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
      } catch (std::invalid_argument &) {
          return this->fail_detailed(function, retstring, "ROI not numeric",
                     "could not parse \""+args+"\" as <height> <width>");
      }
    } else {
      // Check if the four arguments are valid vstart, vstop, hstart, hstop
      try {
          vstart = std::stoi(tokens[0]);
          vstop = std::stoi(tokens[1]);
          hstart = std::stoi(tokens[2]);
          hstop = std::stoi(tokens[3]);
      } catch (std::invalid_argument &) {
          return this->fail_detailed(function, retstring, "ROI not numeric",
                     "could not parse \""+args+"\" as four integers");
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

}
