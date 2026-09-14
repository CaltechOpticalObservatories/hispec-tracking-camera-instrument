/**
 * @file     Instruments/hispec_tracking_camera/hispec_tracking_camera_exposure_modes.cpp
 * @brief    implements HISPEC Tracking Camera-specific exposure modes
 * @author   Michael Langmayr <langmayr@astro.caltech.edu>
 *
 */

#include "archon_controller.h"
#include "archon_exposure_modes.h"
#include "common.h"
#include "fits_header_dictionary.h"
#include "hispec_tracking_camera_exposure_modes.h"
#include "hispec_tracking_camera_instrument.h"
#include "timing_stats.h"
#include "utilities.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace Camera {

  namespace {
    constexpr FitsCamera THIS_CAMERA = FitsCamera::ATC;

    // Resolve property's value (falling back to its dictionary default),
    // validate/normalize enums, and add it under its dictionary keyword.
    // Empty result or "N/A" (not applicable to this camera) writes nothing.
    void set_dict_value(Common::FitsKeys &keys, const std::string &property,
                         const std::string &value) {
      const auto *entry = find_header_entry(property);
      if (!entry) return;

      std::string resolved = value.empty() ? header_default(*entry, THIS_CAMERA) : value;
      if (resolved.empty() || resolved == "N/A") return;

      switch (entry->type) {
        case HeaderValueType::Number:
          try { keys.addkey(entry->keyword, std::stod(resolved), entry->comment); }
          catch (const std::exception &e) {
            logwrite("Camera::set_dict_value", "ERROR " + entry->keyword + ": " + e.what());
          }
          break;
        case HeaderValueType::Integer:
          try { keys.addkey(entry->keyword, std::stol(resolved), entry->comment); }
          catch (const std::exception &e) {
            logwrite("Camera::set_dict_value", "ERROR " + entry->keyword + ": " + e.what());
          }
          break;
        case HeaderValueType::Boolean:
          keys.addkey(entry->keyword, (resolved == "TRUE" || resolved == "T"), entry->comment);
          break;
        case HeaderValueType::String: {
          std::string normalized = resolved;
          std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                          [](unsigned char c) { return std::toupper(c); });
          if (entry->enum_values.empty()) {
            keys.addkey(entry->keyword, resolved, entry->comment);
          } else if (is_valid_header_enum(*entry, normalized)) {
            keys.addkey(entry->keyword, normalized, entry->comment);
          } else {
            logwrite("Camera::set_dict_value",
                     "ERROR invalid value for " + entry->keyword + ": " + resolved);
          }
          break;
        }
      }
    }

    std::string subframe_mode_for(HispecTrackingCamera *hispec) {
      if (!hispec->is_windowed()) return "fullframe";
      std::string mode = hispec->controller->selectedmode;
      std::transform(mode.begin(), mode.end(), mode.begin(),
                      [](unsigned char c) { return std::toupper(c); });
      return (mode.find("GUID") != std::string::npos) ? "guiding" : "ROI";
    }

    // std::to_string(double) truncates to 6 decimals; round-trip full precision instead
    std::string precise(double value) {
      std::ostringstream oss;
      oss << std::setprecision(15) << value;
      return oss.str();
    }
  }

  // Autofetch frame layout: 36-byte ASCII header followed by pixel data
  static constexpr int AUTOFETCH_HEADER_LEN = 36;

  // How often a freerun session reports and resets its timing statistics.
  // A session has no end, so the sample vectors cannot simply grow.
  static constexpr size_t STATS_REPORT_FRAMES = 1000;

  // How many fetches in a row may fail before a freerun session gives up. A
  // single bad frame is survivable; a persistent fault should not spin forever.
  static constexpr int MAX_CONSECUTIVE_ERRORS = 5;

  // The Archon clamps parameters to 20 bits, and the frame count goes straight
  // into the ACF's Expose parameter.
  static constexpr int MAX_SEQUENCE_COUNT = 0xFFFFF;

  static size_t block_align(size_t n) {
    return ((n + BLOCK_LEN - 1) / BLOCK_LEN) * BLOCK_LEN;
  }


  /***** read_autofetch_frame ***********************************************/
  /**
   * @brief  Read one autofetch frame: blocks until 36-byte header + pixels
   *         arrive on the Archon socket, then parses the header
   */
  static long read_autofetch_frame(HispecTrackingCamera* hispec,
                                   Utils::TimingStats &fetch_stats,
                                   Utils::TimingStats &archon_ts_deltas,
                                   uint64_t &prev_archon_ts) {
    const std::string function("Camera::read_autofetch_frame");
    const auto fetch_start = std::chrono::steady_clock::now();

    auto* controller = hispec->controller;
    // <QF frame = 36B header + width*height*2. Size from section_size (window
    // pixels), not image_memory (tap-folded, too big).
    const size_t pixel_bytes = static_cast<size_t>(hispec->camera_info.section_size) * 2;
    const size_t frame_size = AUTOFETCH_HEADER_LEN + pixel_bytes;

    char* buf = controller->framebuf;
    size_t total_read = 0;
    const auto timeout = std::chrono::seconds(10);

    while (total_read < frame_size) {
      if (hispec->is_aborted()) {
        logwrite(function, "aborted while reading autofetch frame");
        return ERROR;
      }
      if (std::chrono::steady_clock::now() - fetch_start > timeout) {
        logwrite(function, "ERROR overall timeout reading autofetch frame");
        return ERROR;
      }

      // Drain bytes send_cmd() captured while triggering this frame's
      // exposure (its reply can share a socket read with the frame data)
      // before waiting on the socket for more.
      if (!controller->autofetch_carryover.empty()) {
        const size_t take = std::min(controller->autofetch_carryover.size(), frame_size - total_read);
        std::memcpy(buf + total_read, controller->autofetch_carryover.data(), take);
        controller->autofetch_carryover.erase(0, take);
        total_read += take;
        continue;
      }

      if (!controller->archon.is_readable(1000)) {
        logwrite(function, "ERROR timeout waiting for autofetch data");
        return ERROR;
      }
      int retval = controller->archon.Read(buf + total_read,
                                            static_cast<int>(frame_size - total_read));
      if (retval <= 0) {
        logwrite(function, "ERROR socket read failed");
        return ERROR;
      }
      total_read += static_cast<size_t>(retval);
    }

    try {
      std::string header(buf, AUTOFETCH_HEADER_LEN);
      int frame_number = std::stoi(header.substr(4, 8), nullptr, 16);
      uint64_t timestamp = std::stoull(header.substr(20, 16), nullptr, 16);

      controller->frameinfo.bufframen[controller->frameinfo.index] = frame_number;
      controller->lastframe = frame_number;

      // Archon timestamps are in 0.01 us units
      if (prev_archon_ts != 0) {
        double delta_us = static_cast<double>(timestamp - prev_archon_ts) * 0.01;
        archon_ts_deltas.add(delta_us);
      }
      prev_archon_ts = timestamp;
    }
    catch (const std::exception &e) {
      logwrite(function, "ERROR parsing autofetch header: " + std::string(e.what()));
      return ERROR;
    }

    fetch_stats.record_since(fetch_start);
    return NO_ERROR;
  }
  /***** read_autofetch_frame ***********************************************/


  /***** Camera::ExposureModeHispecTrackingBase::enqueue *********************/
  void ExposureModeHispecTrackingBase::enqueue(std::shared_ptr<ArchonImageBuffer> buf) {
    std::lock_guard<std::mutex> lock(this->queue_mutex);
    this->imagebuf_queue.push(std::move(buf));
    this->queue_cv.notify_one();
  }
  /***** Camera::ExposureModeHispecTrackingBase::enqueue *********************/


  /***** Camera::ExposureModeHispecTrackingBase::sequence_count **************/
  /**
   * @brief  Resolve how many frames this exposure should read
   */
  std::optional<int> ExposureModeHispecTrackingBase::sequence_count() {
    const std::string function("Camera::ExposureModeHispecTrackingBase::sequence_count");

    const std::string args = this->get_args_string();
    if (args.empty()) return 1;

    int count = 0;
    try { count = std::stoi(args); }
    catch (const std::exception &) {
      logwrite(function, "ERROR sequence count is not a number:" + args);
      return std::nullopt;
    }

    if (count < 1 || count > MAX_SEQUENCE_COUNT) {
      logwrite(function, "ERROR sequence count " + std::to_string(count) +
               " outside {1:" + std::to_string(MAX_SEQUENCE_COUNT) + "}");
      return std::nullopt;
    }

    return count;
  }
  /***** Camera::ExposureModeHispecTrackingBase::sequence_count **************/


  /***** Camera::ExposureModeHispecTrackingBase::build_header_set ************/
  std::shared_ptr<Common::FitsKeys> ExposureModeHispecTrackingBase::build_header_set(
      const std::string &operational_mode, const std::string &subframe_mode, bool is_freerun,
      int n_reads) {
    auto* hispec = static_cast<HispecTrackingCamera*>(this->interface);
    auto* controller = hispec->controller;
    auto keys = std::make_shared<Common::FitsKeys>();

    for (const auto &e : hispec->camera_info.systemkeys.keydb) keys->keydb[e.first] = e.second;
    for (const auto &e : hispec->camera_info.userkeys.keydb)   keys->keydb[e.first] = e.second;

    set_dict_value(*keys, "operational_mode", operational_mode);
    set_dict_value(*keys, "subframe_mode", subframe_mode);
    set_dict_value(*keys, "FREERUN", is_freerun ? "TRUE" : "FALSE");
    // A freerun session has no fixed length; its per-frame READNUM counts instead
    if (!is_freerun) set_dict_value(*keys, "n_reads", std::to_string(n_reads));
    set_dict_value(*keys, "file_type", "");
    set_dict_value(*keys, "bitpix", std::to_string(hispec->camera_info.bitpix));
    set_dict_value(*keys, "ref_channel_position", "");
    set_dict_value(*keys, "refpix_channel", "");
    set_dict_value(*keys, "clock_rate", "");
    set_dict_value(*keys, "pixel_time", "");

    if (hispec->is_windowed()) {
      set_dict_value(*keys, "nskip_rows", std::to_string(hispec->window_vstart()));
      set_dict_value(*keys, "nskip_lines", std::to_string(hispec->window_hstart()));
    } else {
      set_dict_value(*keys, "nskip_rows", "0");
      set_dict_value(*keys, "nskip_lines", "0");
    }

    if (!controller->selectedmode.empty()) {
      auto &mode = controller->modemap[controller->selectedmode];
      if (auto it = mode.acfkeys.keydb.find("READOUTMODE"); it != mode.acfkeys.keydb.end())
        set_dict_value(*keys, "read_mode", it->second.keyvalue);
      if (mode.geometry.num_detect > 0)
        set_dict_value(*keys, "n_channels", std::to_string(mode.geometry.num_detect));
      else
        set_dict_value(*keys, "n_channels", "");
    }

    // Bias voltages: MOD10/LVLC_Vn in the ACF's global [CONFIG] section
    if (auto it = controller->configmap.find("MOD10/LVLC_V1"); it != controller->configmap.end())
      set_dict_value(*keys, "LVLC_V1", it->second.value);
    if (auto it = controller->configmap.find("MOD10/LVLC_V2"); it != controller->configmap.end())
      set_dict_value(*keys, "LVLC_V2", it->second.value);
    if (auto it = controller->configmap.find("MOD10/LVLC_V3"); it != controller->configmap.end())
      set_dict_value(*keys, "LVLC_V3", it->second.value);

    keys->addkey("CAMD_VER", std::string(__DATE__) + " " + std::string(__TIME__),
                 "camerad build date");

    return keys;
  }
  /***** Camera::ExposureModeHispecTrackingBase::build_header_set ************/


  /***** Camera::ExposureModeHispecTrackingBase::fetch_frame *****************/
  /**
   * @brief  Fetch one frame from an Archon buffer into dest
   */
  long ExposureModeHispecTrackingBase::fetch_frame(int bufindex, unsigned bufblocks,
                                                   char* dest, size_t dest_bytes) {
    const std::string function("Camera::ExposureModeHispecTrackingBase::fetch_frame");
    auto* controller = static_cast<HispecTrackingCamera*>(this->interface)->controller;

    const size_t needed = static_cast<size_t>(bufblocks) * BLOCK_LEN;
    if (needed > dest_bytes) {
      logwrite(function, "ERROR fetch of "+std::to_string(bufblocks)+" blocks needs "+
               std::to_string(needed)+" bytes, buffer holds "+std::to_string(dest_bytes));
      return ERROR;
    }

    const int bufready = bufindex + 1;  // Archon buffers are 1-based
    if (bufready < 1 || bufready > controller->activebufs) {
      logwrite(function, "ERROR invalid Archon buffer "+std::to_string(bufready)+
               ", expected {1:"+std::to_string(controller->activebufs)+"}");
      return ERROR;
    }

    if (controller->lock_buffer(bufready) == ERROR) {
      logwrite(function, "ERROR locking frame buffer "+std::to_string(bufready));
      return ERROR;
    }
    // Every path below this point must unlock, so let scope do it.
    struct BufferUnlock {
      ArchonController* controller;
      ~BufferUnlock() { controller->unlock_buffer(); }
    } unlock_guard{controller};

    controller->frametype = ArchonController::FRAME_IMAGE;

    // fetch() sets archon_busy and deliberately leaves it set for the reader to
    // clear; miss that and every later Archon command returns BUSY.
    if (controller->fetch(controller->frameinfo.bufbase[bufindex], bufblocks) != NO_ERROR) {
      logwrite(function, "ERROR fetching Archon buffer "+std::to_string(bufready));
      return ERROR;
    }
    struct BusyClear {
      ArchonController* controller;
      ~BusyClear() { controller->archon_busy.clear(); }
    } busy_guard{controller};

    // The Archon streams bufblocks x (4-byte "<XX:" header + BLOCK_LEN bytes).
    // Read it whole, then verify and compact: one syscall per socket-buffer
    // rather than four per kilobyte.
    const size_t wire_bytes = static_cast<size_t>(bufblocks) * (BLOCK_LEN + 4);
    if (this->fetch_buf.size() < wire_bytes) this->fetch_buf.resize(wire_bytes);

    size_t got = 0;
    while (got < wire_bytes) {
      const int n = controller->archon.Read(this->fetch_buf.data()+got, wire_bytes-got);
      if (n <= 0) {
        logwrite(function, "ERROR incomplete frame read: "+std::to_string(got)+
                 " of "+std::to_string(wire_bytes)+" bytes");
        controller->print_frame_status();
        return ERROR;
      }
      got += static_cast<size_t>(n);
    }

    char check[5];
    SNPRINTF(check, "<%02X:", controller->msgref);

    for (unsigned block = 0; block < bufblocks; ++block) {
      const char* record = this->fetch_buf.data() + static_cast<size_t>(block) * (BLOCK_LEN + 4);
      if (std::memcmp(record, check, 4) != 0) {
        if (record[0] == '?') controller->fetchlog();  // Archon has something to say
        logwrite(function, "ERROR bad block header at block "+std::to_string(block)+
                 " of "+std::to_string(bufblocks));
        controller->print_frame_status();
        return ERROR;
      }
      std::memcpy(dest + static_cast<size_t>(block) * BLOCK_LEN, record + 4, BLOCK_LEN);
    }

    return NO_ERROR;
  }
  /***** Camera::ExposureModeHispecTrackingBase::fetch_frame *****************/


  /***** Camera::ExposureModeHispecTrackingBase::~ExposureModeHispecTrackingBase */
  /**
   * @brief  stop and join the session consumer, if one is still running
   * @details  A thread left running against a destroyed object is undefined
   *           behaviour, and set_exposure_mode() can replace this object.
   */
  ExposureModeHispecTrackingBase::~ExposureModeHispecTrackingBase() {
    this->stop_freerun();
  }
  /***** Camera::ExposureModeHispecTrackingBase::~ExposureModeHispecTrackingBase */


  /***** Camera::ExposureModeHispecTrackingBase::dispatch_one ****************/
  /**
   * @brief  Build the metadata for one frame and fan it out to frame_outputs
   */
  void ExposureModeHispecTrackingBase::dispatch_one(const std::shared_ptr<ArchonImageBuffer> &buf,
                                                    uint64_t sequence_number) {
    auto* hispec = static_cast<HispecTrackingCamera*>(this->interface);

    Camera::FrameMetadata meta;
    meta.frame_number    = buf->bufframen_slice.empty()    ? 0 : static_cast<uint64_t>(buf->bufframen_slice[0]);
    meta.timestamp       = buf->buftimestamp_slice.empty() ? 0 : buf->buftimestamp_slice[0];
    meta.width           = buf->width;
    meta.height          = buf->height;
    meta.bytes_per_pixel = buf->bytes_per_pixel;
    meta.sequence_number = sequence_number;
    meta.header_set      = this->header_set;

    // sequence_number counts from 0 within this exposure, so reads are 1-based
    const uint64_t read_number = sequence_number + 1;

    auto frame_keys = std::make_shared<Common::FitsKeys>();
    set_dict_value(*frame_keys, "mjd_start", precise(mjd_now()));
    set_dict_value(*frame_keys, "acq_time", get_timestamp());
    set_dict_value(*frame_keys, "exposure_time",
                   precise(hispec->camera_info.exposure_time->get()));
    set_dict_value(*frame_keys, "read_number", std::to_string(read_number));
    meta.frame_keys = std::move(frame_keys);

    const size_t frame_bytes = static_cast<size_t>(buf->width) * buf->height * buf->bytes_per_pixel;
    this->interface->dispatch_frame(buf->rawpixels.get(), frame_bytes, meta);
  }
  /***** Camera::ExposureModeHispecTrackingBase::dispatch_one ****************/


  /***** Camera::ExposureModeHispecTrackingBase::process_frames **************/
  /**
   * @brief  Consumer loop: pop each queued frame and fan it out
   * @param[in]  continuous  false = one exposure, true = whole freerun session
   *
   * @details  Only the termination condition differs between the two modes, so
   *           there is one loop rather than two that would drift apart:
   *
   *             abort          exit now, dropping whatever is still queued
   *             stop_consumer  drain the queue first, then exit  (continuous)
   *             producer done  drain the queue first, then exit  (one-shot)
   */
  void ExposureModeHispecTrackingBase::process_frames(bool continuous) {
    const std::string function("Camera::ExposureModeHispecTrackingBase::process_frames");
    if (this->is_debug) { logwrite(function, continuous ? "enter (continuous)" : "enter"); }

    // whichever flag ends this loop, in this mode
    auto finished = [this, continuous] {
      return continuous ? this->stop_consumer.load() : this->is_producer_finished.load();
    };

    uint64_t sequence_number = 0;

    while (true) {
      std::shared_ptr<ArchonImageBuffer> buf;
      {
        std::unique_lock<std::mutex> lock(this->queue_mutex);
        this->queue_cv.wait(lock, [this, &finished] {
            return !this->imagebuf_queue.empty() || finished() || this->interface->is_aborted();
            });

        if (this->interface->is_aborted()) {
          if (this->is_debug) { logwrite(function, "aborted"); }
          break;
        }

        if (!this->imagebuf_queue.empty()) {
          buf = std::move(this->imagebuf_queue.front());
          this->imagebuf_queue.pop();
        }
        else if (finished()) {
          if (this->is_debug) { logwrite(function, "queue empty and producer finished"); }
          break;
        }
        else continue;
      }

      this->dispatch_one(buf, sequence_number++);
    }

    logwrite(function, "exit");
  }
  /***** Camera::ExposureModeHispecTrackingBase::process_frames **************/


  /***** Camera::ExposureModeHispecTrackingBase::image_processing_thread *****/
  /**
   * @brief  One-shot consumer, spawned and joined by do_expose()
   */
  void ExposureModeHispecTrackingBase::image_processing_thread() {
    this->process_frames(false);
  }
  /***** Camera::ExposureModeHispecTrackingBase::image_processing_thread *****/


  /***** Camera::ExposureModeHispecTrackingBase::signal_consumer_stop ********/
  /**
   * @brief  Tell the session consumer to drain the queue and exit
   */
  void ExposureModeHispecTrackingBase::signal_consumer_stop() {
    {
      std::lock_guard<std::mutex> lock(this->queue_mutex);
      this->is_producer_finished = true;
      this->stop_consumer.store(true);
    }
    this->queue_cv.notify_all();
  }
  /***** Camera::ExposureModeHispecTrackingBase::signal_consumer_stop ********/


  /***** Camera::ExposureModeHispecTrackingBase::start_freerun ***************/
  /**
   * @brief  Start a freerun session and return immediately
   * @return ERROR|NO_ERROR
   */
  long ExposureModeHispecTrackingBase::start_freerun() {
    const std::string function("Camera::ExposureModeHispecTrackingBase::start_freerun");

    if (this->session_running.load()) {
      logwrite(function, "ERROR freerun session already running");
      return ERROR;
    }

    // reap the previous session's threads, if it ended on its own
    this->stop_freerun();

    this->is_producer_finished = false;
    this->is_producer_error    = false;
    this->is_consumer_error    = false;
    this->stop_consumer.store(false);
    this->session_running.store(true);

    // One consumer and one producer for the whole session. Both are members, so
    // priority and CPU affinity can be applied to either via native_handle().
    this->consumer_thread = std::thread([this]() { this->process_frames(true); });

    this->producer_thread = std::thread([this]() {
        this->image_acquisition_thread();   // loops over frames until aborted
        this->signal_consumer_stop();       // let the consumer drain and exit
        this->session_running.store(false); // last act: the session is over
        });

    logwrite(function, "freerun session started");

    return NO_ERROR;
  }
  /***** Camera::ExposureModeHispecTrackingBase::start_freerun ***************/


  /***** Camera::ExposureModeHispecTrackingBase::stop_freerun ****************/
  /**
   * @brief  Stop a freerun session and join both threads
   * @return ERROR|NO_ERROR
   */
  long ExposureModeHispecTrackingBase::stop_freerun() {
    this->signal_consumer_stop();

    // The producer exits on the abort state, which the caller sets; joining
    // here waits only for the in-flight frame read to finish.
    if (this->producer_thread.joinable()) this->producer_thread.join();
    if (this->consumer_thread.joinable()) this->consumer_thread.join();

    this->session_running.store(false);

    long error = NO_ERROR;
    error |= (this->is_producer_error ? ERROR : NO_ERROR);
    error |= (this->is_consumer_error ? ERROR : NO_ERROR);

    return error;
  }
  /***** Camera::ExposureModeHispecTrackingBase::stop_freerun ****************/

  /***** Camera::ExposureModeHispecTrackingDefault::image_acquisition_thread */
  /**
   * @brief  Producer thread: trigger the exposure, then read each frame
   *         directly into its own buffer and queue it for the consumer
   */
  void ExposureModeHispecTrackingDefault::image_acquisition_thread() {
    const std::string function("Camera::ExposureModeHispecTrackingDefault::image_acquisition_thread");
    auto* hispec = static_cast<HispecTrackingCamera*>(this->interface);
    auto* controller = hispec->controller;

    if (controller->selectedmode.empty()) {
      logwrite(function, "ERROR no mode selected");
      this->is_producer_error = true;
      return;
    }
    if (controller->expose_param.empty()) {
      logwrite(function, "ERROR EXPOSE_PARAM not defined in configuration");
      this->is_producer_error = true;
      return;
    }

    // Read once so this thread has one fixed behaviour for its whole life
    const bool freerun = this->is_freerun.load();

    const auto requested = this->sequence_count();
    if (!requested) {
      this->is_producer_error = true;
      return;
    }
    const int nseq = *requested;

    this->header_set = build_header_set(HispecTrackingCameraExposureMode::DEFAULT,
                                         subframe_mode_for(hispec), freerun, nseq);

    auto* mode = &controller->modemap[controller->selectedmode];
    const int num_detect = mode->geometry.num_detect;
    const int fallback_bpp = (mode->samplemode == 1) ? 4 : 2;

    long e = controller->prep_parameter(controller->expose_param, nseq);
    if (e == NO_ERROR) e = controller->load_parameter(controller->expose_param, nseq);
    if (e != NO_ERROR) {
      logwrite(function, "ERROR failed to initiate exposure");
      this->is_producer_error = true;
      return;
    }

    // 64-bit: in freerun these count for the life of the session, and signed
    // overflow is undefined behaviour, not a harmless wrap.
    // Instantiate all variables possible so we don't have to do it in the loop, save time and avoid memory fragmentation.
    long long frames_read = 0;
    int consecutive_errors = 0;  //!< reset by every good frame; see MAX_CONSECUTIVE_ERRORS
    int previous_frame = 0;      //!< Archon frame number of the last frame fetched

    for (long long i = 0; freerun || i < nseq; ++i) {
      if (this->interface->is_aborted()) break;
      // Timeout comes from READOUT_TIME in the config file (readout_time_msec);
      // falls back to exptime + margin if that key isn't set.
      if (controller->wait_for_readout() == ERROR) {
        this->is_producer_error = true;
        return;
      }
      if (this->interface->is_aborted()) break;

      const auto idx = controller->frameinfo.index.load();

      // Geometry from the Archon-reported buffer dimensions (BUFnWIDTH/HEIGHT),
      // read signed so a negative value trips the fallback instead of wrapping
      // to a huge unsigned and sizing a monstrous fetch.
      int32_t  sw   = controller->frameinfo.bufwidth[idx];
      int32_t  sh   = controller->frameinfo.bufheight[idx];
      uint32_t fbpp = (controller->frameinfo.bufsample[idx] == 1) ? 4u : 2u;
      if (sw <= 0 || sh <= 0) {
        sw   = static_cast<int32_t>(hispec->camera_info.detector_pixels[0]);
        sh   = static_cast<int32_t>(hispec->camera_info.detector_pixels[1]);
        fbpp = static_cast<uint32_t>(fallback_bpp);
      }
      const uint32_t fw = static_cast<uint32_t>(sw);
      const uint32_t fh = static_cast<uint32_t>(sh);

      if (i == 0) {
        logwrite(function, "frame geometry " + std::to_string(fw) + "x" +
                 std::to_string(fh) + " bpp=" + std::to_string(fbpp));
      }

      // ONE size for this frame. bufblocks is what the Archon will send, and
      // nbytes is derived from it, so the allocation is by construction exactly
      // what the fetch delivers. Sizing these separately is what corrupted the
      // heap: two computations from two snapshots with two fallback rules.
      const size_t   frame_bytes = static_cast<size_t>(fw) * fh * fbpp * num_detect;
      const unsigned bufblocks   = static_cast<unsigned>((frame_bytes + BLOCK_LEN - 1) / BLOCK_LEN);
      const size_t   nbytes      = static_cast<size_t>(bufblocks) * BLOCK_LEN;

      auto imagebuffer = std::make_shared<ArchonImageBuffer>();
      try { imagebuffer->rawpixels = std::shared_ptr<char[]>(new char[nbytes]); }
      catch (const std::exception &ex) {
        logwrite(function, "ERROR allocating image buffer: " + std::string(ex.what()));
        this->is_producer_error = true;
        break;
      }
      // A failed fetch ends a one-shot sequence, but in freerun it is just a
      // lost frame: one bad block header should not tear down the session.
      if (this->fetch_frame(idx, bufblocks, imagebuffer->rawpixels.get(), nbytes) != NO_ERROR) {
        if (!freerun) { this->is_producer_error = true; break; }
        if (++consecutive_errors > MAX_CONSECUTIVE_ERRORS) {
          logwrite(function, "ERROR "+std::to_string(consecutive_errors)+
                   " consecutive fetch failures, ending freerun acquisition");
          this->is_producer_error = true;
          break;
        }
        continue;
      }
      consecutive_errors = 0;

      // wait_for_readout() rebaselines its reference frame on every call, so a
      // fetch slower than the detector skips frames without failing. A ramp is
      // only meaningful across consecutive reads, so name the gap.
      const int frame_number = controller->frameinfo.bufframen[idx];
      if (previous_frame != 0 && frame_number != previous_frame + 1) {
        logwrite(function, "WARNING frame discontinuity: expected " +
                 std::to_string(previous_frame + 1) + " got " + std::to_string(frame_number));
      }
      previous_frame = frame_number;

      imagebuffer->width           = fw;
      imagebuffer->height          = fh;
      imagebuffer->bytes_per_pixel = fbpp;
      imagebuffer->bufframen_slice.push_back(frame_number);
      imagebuffer->buftimestamp_slice.push_back(controller->frameinfo.buftimestamp[idx]);
      this->enqueue(std::move(imagebuffer));
      ++frames_read;
    }

    logwrite(function, freerun
             ? "freerun acquisition stopped after " + std::to_string(frames_read) + " frames"
             : "sequence complete: " + std::to_string(frames_read) + " of " +
               std::to_string(nseq) + " frames");
  }
  /***** Camera::ExposureModeHispecTrackingDefault::image_acquisition_thread */


  /***** Camera::ExposureModeHispecTrackingAutofetch::image_acquisition_thread */
  /**
   * @brief  Producer thread: stream frames from the Archon (header + pixels)
   *         into framebuf, copy each into its own buffer, and queue it
   */
  void ExposureModeHispecTrackingAutofetch::image_acquisition_thread() {
    const std::string function("Camera::ExposureModeHispecTrackingAutofetch::image_acquisition_thread");
    auto* hispec = static_cast<HispecTrackingCamera*>(this->interface);
    auto* controller = hispec->controller;

    if (controller->selectedmode.empty()) {
      logwrite(function, "ERROR no mode selected");
      this->is_producer_error = true;
      return;
    }

    // Read once so this thread has one fixed behaviour for its whole life
    const bool freerun = this->is_freerun.load();

    const auto requested = this->sequence_count();
    if (!requested) {
      this->is_producer_error = true;
      return;
    }
    const int nseq = *requested;

    this->header_set = build_header_set(HispecTrackingCameraExposureMode::AUTOFETCH,
                                         subframe_mode_for(hispec), freerun, nseq);

    auto* mode = &controller->modemap[controller->selectedmode];
    const int num_detect = mode->geometry.num_detect;
    const int bpp = (mode->samplemode == 1) ? 4 : 2;

    Utils::TimingStats fetch_stats;
    Utils::TimingStats archon_ts_deltas;
    uint64_t prev_archon_ts = 0;

    const uint32_t bufsize = static_cast<uint32_t>(
        block_align(hispec->camera_info.image_memory * num_detect + AUTOFETCH_HEADER_LEN));
    if (controller->allocate_framebuf(bufsize) != NO_ERROR) {
      logwrite(function, "ERROR unable to allocate frame buffer");
      this->is_producer_error = true;
      return;
    }

    // Trigger the exposure; in fast autofetch the Archon then streams <QF frames
    // on its own (no FETCH/FRAME polling).
    if (controller->expose_param.empty()) {
      logwrite(function, "ERROR EXPOSE_PARAM not defined in configuration");
      this->is_producer_error = true;
      return;
    }
    long e = controller->prep_parameter(controller->expose_param, nseq);
    if (e == NO_ERROR) e = controller->load_parameter(controller->expose_param, nseq);
    if (e != NO_ERROR) {
      logwrite(function, "ERROR failed to initiate exposure");
      this->is_producer_error = true;
      return;
    }

    // 64-bit: in freerun these count for the life of the session, and signed
    // overflow is undefined behaviour, not a harmless wrap.
    long long frames_read = 0;
    for (long long i = 0; freerun || i < nseq; ++i) {
      if (this->interface->is_aborted()) break;

      if (read_autofetch_frame(hispec, fetch_stats, archon_ts_deltas, prev_archon_ts) != NO_ERROR) {
        this->is_producer_error = true;
        break;
      }

      const auto idx = controller->frameinfo.index.load();

      // Windowed readout geometry (naxes), matching the <QF frame size.
      const uint32_t fw   = hispec->camera_info.naxes[0];
      const uint32_t fh   = hispec->camera_info.naxes[1];
      const uint32_t fbpp = 2;
      const size_t   nbytes = static_cast<size_t>(hispec->camera_info.section_size) * fbpp;

      auto imagebuffer = std::make_shared<ArchonImageBuffer>();
      try { imagebuffer->rawpixels = std::shared_ptr<char[]>(new char[nbytes]); }
      catch (const std::exception &ex) {
        logwrite(function, "ERROR allocating image buffer: " + std::string(ex.what()));
        this->is_producer_error = true;
        break;
      }
      std::memcpy(imagebuffer->rawpixels.get(),
                  controller->framebuf + AUTOFETCH_HEADER_LEN, nbytes);
      imagebuffer->width           = fw;
      imagebuffer->height          = fh;
      imagebuffer->bytes_per_pixel = fbpp;
      imagebuffer->bufframen_slice.push_back(controller->frameinfo.bufframen[idx]);
      imagebuffer->buftimestamp_slice.push_back(controller->frameinfo.buftimestamp[idx]);
      this->enqueue(std::move(imagebuffer));
      ++frames_read;

      // A freerun session has no end, so these sample vectors would grow
      // without bound. Report and reset them periodically instead.
      if (freerun && archon_ts_deltas.count() >= STATS_REPORT_FRAMES) {
        logwrite(function, archon_ts_deltas.summary("archon frame interval"));
        logwrite(function, fetch_stats.summary("host readout duration"));
        archon_ts_deltas.clear();
        fetch_stats.clear();
      }
    }

    if (!archon_ts_deltas.empty()) {
      logwrite(function, archon_ts_deltas.summary("archon frame interval"));
    }
    if (!fetch_stats.empty()) {
      logwrite(function, fetch_stats.summary("host readout duration"));
    }
    logwrite(function, freerun
             ? "freerun acquisition stopped after " + std::to_string(frames_read) + " frames"
             : "sequence complete: " + std::to_string(frames_read) + " of " +
               std::to_string(nseq) + " frames");
  }
  /***** Camera::ExposureModeHispecTrackingAutofetch::image_acquisition_thread */

}
