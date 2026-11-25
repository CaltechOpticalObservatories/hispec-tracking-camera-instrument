#include "archon_controller.h"
#include "archon_exposure_modes.h"
#include "common.h"
#include "fits_header_dictionary.h"
#include "fits_writer.h"
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
#include <optional>
#include <pthread.h>
#include <sched.h>
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
      const std::string &section = hispec->controller->selectedmode;
      if (hispec->is_windowed() || section.find("GUIDING") != std::string::npos) return "guiding";
      const auto &mode = hispec->controller->modemap[section];
      return (mode.geometry.linecount >= hispec->detector_rows()) ? "full_frame" : "roi";
    }

    std::string operational_mode_for(bool is_autofetch, bool is_freerun) {
      if (is_freerun) return is_autofetch ? "freerun+autofetch" : "freerun";
      return is_autofetch ? "autofetch" : "default";
    }

    // std::to_string(double) truncates to 6 decimals; round-trip full precision instead
    std::string precise(double value) {
      std::ostringstream oss;
      oss << std::setprecision(15) << value;
      return oss.str();
    }

    // Which tapline the reference amp reads out changes with the mode
    std::string refpix_channel_for(const ArchonController::modeinfo_t &mode,
                                   const std::string &refpix_amp) {
      for (int tap = 0; tap < mode.tapinfo.num_taps; ++tap) {
        if (mode.tapinfo.ampname[tap] == refpix_amp) return std::to_string(tap + 1);
      }
      return "";
    }
  }

  // Autofetch frame layout: 36-byte ASCII header followed by pixel data
  static constexpr int AUTOFETCH_HEADER_LEN = 36;
  // The Archon splits its on-chip memory into a 16k network write buffer per
  // connection and will not transmit an autofetch frame larger than that
  static constexpr size_t MAX_AUTOFETCH_FRAME_BYTES = 16384;

  // Field offsets within that header: <QF s ffffffff wwww hhhh tttttttttttttttt
  static constexpr int HEADER_WIDTH_OFFSET = 12;
  static constexpr int HEADER_HEIGHT_OFFSET = 16;
  static constexpr int HEADER_FIELD_LEN = 4;

  // How often a freerun session reports and resets its timing statistics.
  // A session has no end, so the sample vectors cannot simply grow.
  static constexpr size_t STATS_REPORT_FRAMES = 1000;

  // How many fetches in a row may fail before a freerun session gives up. A
  // single bad frame is survivable; a persistent fault should not spin forever.
  static constexpr int MAX_CONSECUTIVE_ERRORS = 5;

  // Allowed, on top of the exposure delay, for the sequencer to finish a
  // sequence, or an interleaved pass, once its last frame is complete. What is left by then
  // takes it microseconds, so this is all margin.
  static constexpr int PASS_SETTLE_MSEC = 20;

  // The sequencer runs one more exposure delay after a sequence's, or a pass's,
  // last frame before it counts it off; a parameter written meanwhile is
  // overwritten by that bookkeeping, which leaves the next pass a set short
  static std::chrono::steady_clock::time_point sequencer_idle_after(
      std::chrono::steady_clock::time_point last_frame, double exptime_sec) {
    return last_frame + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<double>(exptime_sec)) +
           std::chrono::milliseconds(PASS_SETTLE_MSEC);
  }

  // Used only when no geometry-derived deadline has been set
  static constexpr int DEFAULT_FRAME_WAIT_MSEC = 15000;
  static constexpr int FRAME_POLL_USEC = 1000;

  // Shown by ps -L and top -H; Linux caps thread names at 15 characters
  static constexpr const char* ACQUISITION_THREAD_NAME = "camerad:acq";
  static constexpr const char* PROCESSING_THREAD_NAME  = "camerad:proc";

  static size_t block_align(size_t n) {
    return ((n + BLOCK_LEN - 1) / BLOCK_LEN) * BLOCK_LEN;
  }


  static constexpr std::pair<InterleaveOutput, const char*> INTERLEAVE_OUTPUT_NAMES[] = {
    {InterleaveOutput::Raw, "raw"}, {InterleaveOutput::Split, "split"}, {InterleaveOutput::GuideOnly, "guide"}};

  /***** Camera::interleave_output_name ***************************************/
  const char* interleave_output_name(InterleaveOutput output) {
    for (const auto &[value, name] : INTERLEAVE_OUTPUT_NAMES) if (value == output) return name;
    return "raw";
  }
  /***** Camera::interleave_output_name ***************************************/


  /***** Camera::parse_interleave_output **************************************/
  InterleaveOutput parse_interleave_output(const std::string &text) {
    for (const auto &[value, name] : INTERLEAVE_OUTPUT_NAMES) if (text == name) return value;
    throw std::runtime_error("INTERLEAVE_OUTPUT="+text+": expected raw, split or guide");
  }
  /***** Camera::parse_interleave_output **************************************/


  /***** Camera::describe_placement *******************************************/
  std::string describe_placement(const ThreadPlacement &placement) {
    const std::string cpu = placement.cpu == ThreadPlacement::ANY_CPU
                          ? "any cpu" : "cpu " + std::to_string(placement.cpu);
    const std::string policy = placement.rt_priority == ThreadPlacement::NOT_REALTIME
                             ? "normal scheduling (SCHED_OTHER)"
                             : "real-time priority " + std::to_string(placement.rt_priority) + " (SCHED_FIFO)";
    return cpu + ", " + policy;
  }
  /***** Camera::describe_placement *******************************************/


  // pthread_* return the error number rather than setting errno
  static int pin_to_cpu(pthread_t thread, int cpu) {
    cpu_set_t cpus;
    CPU_ZERO(&cpus);
    CPU_SET(cpu, &cpus);
    return pthread_setaffinity_np(thread, sizeof(cpus), &cpus);
  }

  static int make_realtime(pthread_t thread, int priority) {
    sched_param param{};
    param.sched_priority = priority;
    return pthread_setschedparam(thread, SCHED_FIFO, &param);
  }

  // Read back from the kernel, so the log reports the outcome, not the request
  static ThreadPlacement current_placement(pthread_t thread) {
    ThreadPlacement placement;
    cpu_set_t cpus;
    if (pthread_getaffinity_np(thread, sizeof(cpus), &cpus) == 0 && CPU_COUNT(&cpus) == 1) {
      for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
        if (CPU_ISSET(cpu, &cpus)) { placement.cpu = cpu; break; }
      }
    }
    int policy = SCHED_OTHER;
    sched_param param{};
    if (pthread_getschedparam(thread, &policy, &param) == 0 && policy == SCHED_FIFO) {
      placement.rt_priority = param.sched_priority;
    }
    return placement;
  }


  /***** Camera::place_current_thread *****************************************/
  /**
   * @brief  name the calling thread and apply its CPU and scheduling
   * @details  Logs one line per thread: where it runs, plus why any part of
   *           the request was refused.
   */
  void place_current_thread(const char* name, const ThreadPlacement &placement) {
    const std::string function("Camera::place_current_thread");
    const pthread_t self = pthread_self();

    pthread_setname_np(self, name);
    if (!placement.is_configured()) return;

    std::string refusals;
    if (placement.cpu != ThreadPlacement::ANY_CPU) {
      if (const int error_number = pin_to_cpu(self, placement.cpu)) {
        refusals += "; cpu " + std::to_string(placement.cpu) + " refused: " +
                    std::strerror(error_number);
      }
    }
    if (placement.rt_priority != ThreadPlacement::NOT_REALTIME) {
      if (const int error_number = make_realtime(self, placement.rt_priority)) {
        refusals += "; real-time priority " + std::to_string(placement.rt_priority) +
                    " refused: " + std::strerror(error_number) +
                    " (grant camerad CAP_SYS_NICE, or raise the user's RLIMIT_RTPRIO to " +
                    std::to_string(placement.rt_priority) + ")";
      }
    }

    const std::string status = std::string(name) + " running on " +
                               describe_placement(current_placement(self));
    logwrite(function, refusals.empty() ? status : "WARNING " + status + refusals);
  }
  /***** Camera::place_current_thread *****************************************/
  // Long enough that a frame still in flight arrives, short enough that the
  // end of a stopped stream is not mistaken for a stall
  static constexpr int DRAIN_QUIET_MSEC = 500;

  /**
   * @brief  Read and throw away whatever the controller is still sending
   * @details Call only once the push has been stopped, otherwise the stream
   *          never goes quiet and this does not return
   * @return number of bytes discarded
   */
  static size_t drain_until_quiet(ArchonController* controller) {
    size_t dropped = controller->autofetch_carryover.size();
    controller->autofetch_carryover.clear();
    while (controller->archon.is_readable(DRAIN_QUIET_MSEC)) {
      const int got = controller->archon.Read(controller->framebuf,
                                              static_cast<int>(controller->framebuf_bytes));
      if (got <= 0) break;
      dropped += static_cast<size_t>(got);
    }
    return dropped;
  }


  /**
   * @brief  Read and throw away the next nbytes of the autofetch stream
   * @details Uses the frame buffer as scratch, so it never reads more than
   *          the buffer holds however large the frame being dropped is
   */
  static long discard_autofetch_bytes(ArchonController* controller, size_t nbytes) {
    const std::string function("Camera::discard_autofetch_bytes");
    while (nbytes > 0) {
      if (!controller->autofetch_carryover.empty()) {
        const size_t take = std::min(controller->autofetch_carryover.size(), nbytes);
        controller->autofetch_carryover.erase(0, take);
        nbytes -= take;
        continue;
      }
      if (!controller->archon.is_readable(1000)) {
        logwrite(function, "ERROR timeout discarding "+std::to_string(nbytes)+" bytes");
        return ERROR;
      }
      const int want = static_cast<int>(
          std::min(nbytes, static_cast<size_t>(controller->framebuf_bytes)));
      const int got = controller->archon.Read(controller->framebuf, want);
      if (got <= 0) {
        logwrite(function, "ERROR socket read failed while discarding");
        return ERROR;
      }
      nbytes -= static_cast<size_t>(got);
    }
    return NO_ERROR;
  }


  /***** read_autofetch_frame ***********************************************/
  /**
   * @brief  Read one autofetch frame: blocks until 36-byte header + pixels
   *         arrive on the Archon socket, then parses the header
   */
  static long read_autofetch_frame(HispecTrackingCamera* hispec,
                                   Utils::TimingStats &fetch_stats,
                                   Utils::TimingStats &archon_ts_deltas,
                                   uint64_t &prev_archon_ts,
                                   uint32_t &frame_width,
                                   uint32_t &frame_height) {
    const std::string function("Camera::read_autofetch_frame");
    const auto fetch_start = std::chrono::steady_clock::now();

    auto* controller = hispec->controller;
    // Only a starting guess; the header below carries the frame's real size
    const size_t pixel_bytes = static_cast<size_t>(hispec->camera_info.section_size) * 2;
    const size_t expected_size = AUTOFETCH_HEADER_LEN + pixel_bytes;
    // Rows identify the window: columns double because each frame carries the
    // reset image, and utr_rr doubles them again, but the row count is what
    // was configured. A frame with any other height belongs to an earlier run.
    const auto &roi = hispec->camera_info.region_of_interest;
    const uint32_t expected_rows = roi[3] - roi[2] + 1;
    size_t frame_size = expected_size;
    bool sized_from_header = false;

    char* buf = controller->framebuf;
    size_t total_read = 0;
    const auto timeout = std::chrono::seconds(10);

    while (total_read < frame_size || !sized_from_header) {
      if (hispec->is_aborted()) {
        logwrite(function, "aborted while reading autofetch frame");
        return ERROR;
      }
      if (std::chrono::steady_clock::now() - fetch_start > timeout) {
        logwrite(function, "ERROR overall timeout reading autofetch frame");
        return ERROR;
      }
      // The host's geometry can disagree with what the detector emits, and a
      // short read leaves the rest of the frame in the socket, desynchronising
      // every frame after it
      if (!sized_from_header && total_read >= AUTOFETCH_HEADER_LEN) {
        try {
          const auto header_width = std::stoul(
              std::string(buf+HEADER_WIDTH_OFFSET, HEADER_FIELD_LEN), nullptr, 16);
          const auto header_height = std::stoul(
              std::string(buf+HEADER_HEIGHT_OFFSET, HEADER_FIELD_LEN), nullptr, 16);
          if (header_width > 0 && header_height > 0) {
            const size_t header_size = AUTOFETCH_HEADER_LEN +
                static_cast<size_t>(header_width) * header_height * 2;
            // Enabling autofetch flushes frames the controller still held from
            // the previous run, at that run's geometry. Drop them whole and
            // resynchronise on the next header. The buffer bound is a second
            // check so an implausible header can never overrun it.
            if (header_height != expected_rows ||
                header_size > controller->framebuf_bytes) {
              logwrite(function, "dropping stale "+std::to_string(header_width)+"x"+
                                 std::to_string(header_height)+" frame, this run "
                                 "reads "+std::to_string(expected_rows)+" rows");
              if (discard_autofetch_bytes(controller, header_size - total_read) != NO_ERROR) {
                return ERROR;
              }
              frame_size = expected_size;
              total_read = 0;
              continue;
            }
            // Once per process: this path can run at thousands of frames a second
            static bool mismatch_logged = false;
            if (header_size != frame_size && !mismatch_logged) {
              mismatch_logged = true;
              logwrite(function, "frame is "+std::to_string(header_width)+"x"+
                                 std::to_string(header_height)+" ("+
                                 std::to_string(header_size)+" bytes), host expected "+
                                 std::to_string(frame_size));
            }
            frame_size = header_size;
            frame_width = static_cast<uint32_t>(header_width);
            frame_height = static_cast<uint32_t>(header_height);
            sized_from_header = true;
          }
        }
        catch (const std::exception &) {
          logwrite(function, "ERROR could not parse autofetch header geometry");
          return ERROR;
        }
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
        // Once the session is aborted the push has been stopped, so the
        // stream running dry is how it ends, not a fault worth an ERROR
        logwrite(function, hispec->is_aborted()
                 ? "stream ended after abort with "+std::to_string(total_read)+
                   " of "+std::to_string(frame_size)+" bytes of a partial frame"
                 : "ERROR timeout waiting for autofetch data: wanted "+
                   std::to_string(frame_size)+" bytes, got "+
                   std::to_string(total_read));
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
      controller->frameinfo.buftimestamp[controller->frameinfo.index] = timestamp;
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
  void ExposureModeHispecTrackingBase::enqueue(std::shared_ptr<HispecFrame> buf) {
    std::lock_guard<std::mutex> lock(this->queue_mutex);
    this->imagebuf_queue.push(std::move(buf));
    this->queue_cv.notify_one();
  }
  /***** Camera::ExposureModeHispecTrackingBase::enqueue *********************/


  /***** Camera::ExposureModeHispecTrackingBase::build_header_set ************/
  std::shared_ptr<Common::FitsKeys> ExposureModeHispecTrackingBase::build_header_set(
      bool is_autofetch, bool is_freerun) {
    auto* hispec = static_cast<HispecTrackingCamera*>(this->interface);
    auto* controller = hispec->controller;
    auto keys = std::make_shared<Common::FitsKeys>();

    for (const auto &e : hispec->camera_info.systemkeys.keydb) keys->keydb[e.first] = e.second;
    for (const auto &e : hispec->camera_info.userkeys.keydb)   keys->keydb[e.first] = e.second;

    set_dict_value(*keys, "operational_mode", operational_mode_for(is_autofetch, is_freerun));
    set_dict_value(*keys, "subframe_mode", subframe_mode_for(hispec));
    set_dict_value(*keys, "read_mode", hispec->readout_mode());
    set_dict_value(*keys, "FREERUN", is_freerun ? "TRUE" : "FALSE");
    if (hispec->is_interleave()) {
      // Sizes only: the guide window's position goes with each frame, as it can move
      const auto &interleave = hispec->interleave();
      const GuidePosition guide = hispec->guide_position();
      set_dict_value(*keys, "interleave_rows_per_set", std::to_string(interleave.rows_per_set));
      set_dict_value(*keys, "interleave_sets_per_pass", std::to_string(interleave.sets_per_pass()));
      set_dict_value(*keys, "interleave_science_start", std::to_string(interleave.science_start_row));
      set_dict_value(*keys, "interleave_science_rows", std::to_string(interleave.science_rows));
      set_dict_value(*keys, "interleave_science_pixels", std::to_string(interleave.science_pixels_per_tap));
      set_dict_value(*keys, "interleave_guide_rows", std::to_string(guide.vstop - guide.vstart + 1));
      set_dict_value(*keys, "interleave_guide_cols", std::to_string(guide.hstop - guide.hstart + 1));
      set_dict_value(*keys, "interleave_output", interleave_output_name(hispec->interleave_output_mode()));
    }
    set_dict_value(*keys, "file_type", "");
    set_dict_value(*keys, "bitpix", std::to_string(hispec->camera_info.bitpix));
    set_dict_value(*keys, "ref_channel_position", "");
    set_dict_value(*keys, "pixel_time", precise(hispec->effective_pixel_time_usec()));
    // Constant, so it is built once per exposure rather than per frame: the
    // per-frame set runs at the frame rate, which reaches 1 kHz in autofetch
    set_dict_value(*keys, "exposure_time_unit", "");
    set_dict_value(*keys, "FIRMWARE", controller->firmware);

    if (hispec->is_windowed()) {
      set_dict_value(*keys, "nskip_rows", std::to_string(hispec->window_vstart()));
      set_dict_value(*keys, "nskip_lines", std::to_string(hispec->window_hstart()));
    } else {
      set_dict_value(*keys, "nskip_rows", "0");
      set_dict_value(*keys, "nskip_lines", "0");
    }

    if (!controller->selectedmode.empty()) {
      const auto &mode = controller->modemap[controller->selectedmode];
      // The reference channel is not a detector channel, so it is not counted
      const int num_taps = mode.tapinfo.num_taps;
      set_dict_value(*keys, "n_channels", num_taps > 1 ? std::to_string(num_taps - 1) : "");
      set_dict_value(*keys, "refpix_channel", refpix_channel_for(mode, hispec->reference_amp()));
      // Same quantity the readout deadline is built from, so they cannot disagree
      const double frame_sec = hispec->frame_readout_sec();
      set_dict_value(*keys, "frame_time", frame_sec > 0.0 ? precise(frame_sec) : "");
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
    auto* hispec = static_cast<HispecTrackingCamera*>(this->interface);
    auto* controller = hispec->controller;

    const size_t needed = static_cast<size_t>(bufblocks) * BLOCK_LEN;
    if (needed > dest_bytes) {
      hispec->log_error(function, "frame larger than its buffer",
                 "the Archon will send "+std::to_string(bufblocks)+" blocks ("+
                 std::to_string(needed)+" bytes) but the buffer holds "+
                 std::to_string(dest_bytes)+"; the geometry changed after the buffer "
                 "was sized");
      return ERROR;
    }

    const int bufready = bufindex + 1;  // Archon buffers are 1-based
    if (bufready < 1 || bufready > controller->activebufs) {
      hispec->log_error(function, "invalid frame buffer",
                 "buffer "+std::to_string(bufready)+" is outside {1:"+
                 std::to_string(controller->activebufs)+"}; frameinfo.index was "+
                 std::to_string(bufindex));
      return ERROR;
    }

    if (controller->lock_buffer(bufready) == ERROR) {
      hispec->log_error(function, "buffer lock failed",
                 "the Archon rejected LOCK"+std::to_string(bufready)+
                 ", so the frame cannot be read without risking an overwrite");
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
      hispec->log_error(function, "fetch command rejected",
                 "the Archon refused FETCH for buffer "+std::to_string(bufready)+
                 " at address "+std::to_string(controller->frameinfo.bufbase[bufindex]));
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
        hispec->log_error(function, "short frame read",
                   "socket returned "+std::to_string(got)+" of "+
                   std::to_string(wire_bytes)+" expected bytes for buffer "+
                   std::to_string(bufready)+"; the connection closed or stalled mid-frame");
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
        hispec->log_error(function, "corrupt frame data",
                   "block "+std::to_string(block)+" of "+std::to_string(bufblocks)+
                   " did not start with the expected header "+std::string(check)+
                   "; the data stream is out of sync with the command stream");
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


  /***** Camera::ExposureModeHispecTrackingBase::wait_for_frame **************/
  /**
   * @brief  Wait for a completed Archon frame newer than baseline
   */
  long ExposureModeHispecTrackingBase::wait_for_frame(int &baseline, bool first_of_sequence) {
    const std::string function("Camera::ExposureModeHispecTrackingBase::wait_for_frame");
    auto* hispec = static_cast<HispecTrackingCamera*>(this->interface);
    auto* controller = hispec->controller;

    int budget_msec = controller->readout_time_msec > 0
                    ? controller->readout_time_msec : DEFAULT_FRAME_WAIT_MSEC;
    // The opening frame may have to wait out a frame already in flight from a
    // previous command before ours even starts
    if (first_of_sequence) budget_msec *= 2;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(budget_msec);

    while (!this->interface->is_aborted()) {
      if (controller->get_frame_status() == NO_ERROR) {
        // Newest buffer the controller has finished writing. Deliberately not
        // controller->lastframe, which is 0 whenever nothing is complete yet.
        int newest = 0, index = -1;
        for (size_t buf=0; buf < controller->frameinfo.bufframen.size(); ++buf) {
          if (controller->frameinfo.bufcomplete[buf] &&
              controller->frameinfo.bufframen[buf] > newest) {
            newest = controller->frameinfo.bufframen[buf];
            index  = static_cast<int>(buf);
          }
        }

        if (newest > baseline && index >= 0) {
          if (newest > baseline + 1) {
            const int missed = newest - baseline - 1;
            const std::string detail = std::to_string(missed)+" frame(s) were overwritten "
                       "before the host could fetch them: expected frame "+
                       std::to_string(baseline+1)+" but the newest complete buffer is "+
                       std::to_string(newest);
            this->frames_dropped += missed;
            logwrite(function, "WARNING dropped frames: "+detail+"; continuing from "+
                               std::to_string(newest)+" ("+
                               std::to_string(this->frames_dropped.load())+" this session)");
          }
          controller->frameinfo.index.store(index);
          baseline = newest;
          return NO_ERROR;
        }
      }

      if (std::chrono::steady_clock::now() > deadline) {
        hispec->log_error(function, "readout timeout",
                   "no buffer completed frame "+std::to_string(baseline+1)+" within "+
                   std::to_string(budget_msec)+" msec; either the sequencer is not "
                   "producing frames for this mode, or the deadline is short for the "
                   "geometry (raise READOUT_MARGIN_MSEC or PIXEL_TIME_USEC)");
        return ERROR;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(FRAME_POLL_USEC));
    }

    logwrite(function, "aborted waiting for frame "+std::to_string(baseline+1));
    return ERROR;
  }
  /***** Camera::ExposureModeHispecTrackingBase::wait_for_frame **************/
  /***** Camera::ExposureModeHispecTrackingBase::frame_metadata **************/
  /**
   * @brief  Metadata and per-frame FITS keys for one frame, as it was fetched
   */
  FrameMetadata ExposureModeHispecTrackingBase::frame_metadata(const HispecFrame &frame,
                                                               uint64_t sequence_number) const {
    auto* hispec = static_cast<HispecTrackingCamera*>(this->interface);

    Camera::FrameMetadata meta;
    meta.frame_number    = frame.bufframen_slice.empty()    ? 0 : static_cast<uint64_t>(frame.bufframen_slice[0]);
    meta.timestamp       = frame.buftimestamp_slice.empty() ? 0 : frame.buftimestamp_slice[0];
    meta.width           = frame.width;
    meta.height          = frame.height;
    meta.bytes_per_pixel = frame.bytes_per_pixel;
    meta.sequence_number = sequence_number;
    meta.header_set      = this->header_set;

    // No multi-read exposure mode exists yet (see FITS-HEADERS-AND-CUBE-PLAN.md D2.1);
    // one queued buffer is one whole exposure, so these are per-exposure, not summed.
    auto frame_keys = std::make_shared<Common::FitsKeys>();
    set_dict_value(*frame_keys, "mjd_start", precise(mjd_now()));
    set_dict_value(*frame_keys, "acq_time", get_timestamp());
    set_dict_value(*frame_keys, "exposure_time",
                   precise(hispec->camera_info.exposure_time->get()));
    set_dict_value(*frame_keys, "n_reads", "1");
    if (frame.interleave) {
      const auto &tag = *frame.interleave;
      const auto &interleave = hispec->interleave();
      set_dict_value(*frame_keys, "interleave_pass", std::to_string(tag.pass));
      set_dict_value(*frame_keys, "interleave_set_index", std::to_string(tag.set));
      set_dict_value(*frame_keys, "interleave_set_row",
                     std::to_string(interleave.science_start_row + tag.set * interleave.rows_per_set));
      set_dict_value(*frame_keys, "guide_vstart", std::to_string(tag.guide.vstart));
      set_dict_value(*frame_keys, "guide_vstop", std::to_string(tag.guide.vstop));
      set_dict_value(*frame_keys, "guide_hstart", std::to_string(tag.guide.hstart));
      set_dict_value(*frame_keys, "guide_hstop", std::to_string(tag.guide.hstop));
      set_dict_value(*frame_keys, "guide_moved", tag.is_guide_moved ? "TRUE" : "FALSE");
    }
    meta.frame_keys = std::move(frame_keys);
    return meta;
  }
  /***** Camera::ExposureModeHispecTrackingBase::frame_metadata **************/


  /***** Camera::ExposureModeHispecTrackingBase::dispatch_one ****************/
  /**
   * @brief  Build the metadata for one frame and fan it out to frame_outputs
   */
  void ExposureModeHispecTrackingBase::dispatch_one(const std::shared_ptr<HispecFrame> &buf,
                                                    uint64_t sequence_number) {
    const Camera::FrameMetadata meta = this->frame_metadata(*buf, sequence_number);
    const size_t frame_bytes = static_cast<size_t>(buf->width) * buf->height * buf->bytes_per_pixel;
    this->interface->dispatch_frame(buf->rawpixels.get(), frame_bytes, meta);
  }
  /***** Camera::ExposureModeHispecTrackingBase::dispatch_one ****************/


  namespace {
    // Stream the guide windows go out on: it names their SHM segment and FITS files
    constexpr const char* GUIDE_STREAM = "guide";

    // Header properties that describe one set, so not an image built from many
    constexpr const char* SET_KEYS[] = {
      "interleave_set_index", "interleave_set_row", "guide_vstart", "guide_vstop",
      "guide_hstart", "guide_hstop", "guide_moved" };

    // FRAMEMODE value with which the Archon stores a frame last line first, so
    // the first row read ends up at the bottom. The ACF uses it for every mode.
    constexpr int FRAMEMODE_BOTTOM_FIRST = 1;

    bool is_stored_bottom_first(const HispecTrackingCamera &hispec) {
      const auto &modes = hispec.controller->modemap;
      const auto mode = modes.find(hispec.controller->selectedmode);
      return mode != modes.end() && mode->second.geometry.framemode == FRAMEMODE_BOTTOM_FIRST;
    }

    /**
     * @brief  Publishes interleaved frames as guide windows and a science image
     * @details  Every frame carries one set: a few science lines and a whole
     *           guide window. The guide window goes out at once on its own
     *           stream. The science lines are gathered into one image, which
     *           goes out on the primary stream when its pass is complete.
     *
     *           A set is read science lines first, and the Archon may store
     *           the frame in that order or reversed. Both outputs keep the
     *           stored order, so the science image comes out the way a frame of
     *           the same rows read in one go would. Measured on the detector on
     *           2026-11-23: bottom first, science in the last lines.
     *
     *           Owned by the consumer thread, so it needs no locking.
     */
    class InterleaveSplitter {
      public:
        explicit InterleaveSplitter(HispecTrackingCamera &hispec)
          : camera(hispec),
            science_lines(static_cast<uint32_t>(hispec.interleave().rows_per_set)),
            science_rows(static_cast<uint32_t>(hispec.interleave().science_rows)),
            is_bottom_first(is_stored_bottom_first(hispec)),
            is_guide_only(hispec.interleave_output_mode() == InterleaveOutput::GuideOnly),
            is_guide_in_fits(hispec.writes_guide_fits()) { }

        void dispatch(const HispecFrame &frame, FrameMetadata meta);

      private:
        void write(const char* data, size_t bytes, const FrameMetadata &meta, bool to_fits);

        HispecTrackingCamera &camera;

        // Frozen while an exposure runs, so read once
        const uint32_t science_lines;   //!< science lines in every frame
        const uint32_t science_rows;    //!< rows of the assembled image
        const bool is_bottom_first;     //!< frames are stored last line first
        const bool is_guide_only;
        const bool is_guide_in_fits;

        std::vector<char> image;        //!< science image of the pass being gathered
        int pass{-1};
        int sets_gathered{0};           //!< sets copied into that image so far
        uint64_t guide_count{0};
        uint64_t image_count{0};
        bool is_layout_reported{false};
    };

    void InterleaveSplitter::dispatch(const HispecFrame &frame, FrameMetadata meta) {
      const size_t line_bytes = static_cast<size_t>(frame.width) * frame.bytes_per_pixel;
      const uint32_t sets = this->science_rows / this->science_lines;

      // A frame that is not laid out as expected goes out whole: better an
      // unsplit frame than a wrong split
      const int set = frame.interleave ? frame.interleave->set : -1;
      if (frame.height <= this->science_lines || set < 0 || static_cast<uint32_t>(set) >= sets) {
        if (!this->is_layout_reported) {
          this->is_layout_reported = true;
          logwrite("Camera::InterleaveSplitter::dispatch",
                   "WARNING frame "+std::to_string(frame.width)+"x"+std::to_string(frame.height)+
                   " (set "+std::to_string(set)+" of "+std::to_string(sets)+") does not hold "+
                   std::to_string(this->science_lines)+" science lines and a guide window; "
                   "such frames are published unsplit");
        }
        this->write(frame.rawpixels.get(), frame.height * line_bytes, meta, true);
        return;
      }

      const uint32_t guide_lines = frame.height - this->science_lines;
      const size_t science_bytes = this->science_lines * line_bytes;
      const size_t guide_bytes   = guide_lines * line_bytes;
      const char* science = frame.rawpixels.get() + (this->is_bottom_first ? guide_bytes : 0);
      const char* guide   = frame.rawpixels.get() + (this->is_bottom_first ? 0 : science_bytes);

      FrameMetadata guide_meta = meta;
      guide_meta.stream          = GUIDE_STREAM;
      guide_meta.height          = guide_lines;
      guide_meta.sequence_number = this->guide_count++;
      this->write(guide, guide_bytes, guide_meta, this->is_guide_in_fits);

      if (this->is_guide_only) return;

      // Started blank for every pass, so the rows of a set whose frame the
      // controller overwrote read as zero, not as the previous pass
      const size_t image_bytes = this->science_rows * line_bytes;
      if (frame.interleave->pass != this->pass || this->image.size() != image_bytes) {
        this->image.assign(image_bytes, 0);
        this->pass = frame.interleave->pass;
        this->sets_gathered = 0;
      }
      // Stored bottom first, the first set read belongs at the end of the image
      const size_t image_set = this->is_bottom_first ? sets - 1 - set : set;
      std::memcpy(this->image.data() + image_set * science_bytes, science, science_bytes);
      ++this->sets_gathered;

      if (static_cast<uint32_t>(set) == sets - 1) {
        // The image is the whole pass, so it drops the keys that describe one
        // set and says how much of the pass it holds
        auto image_keys = std::make_shared<Common::FitsKeys>(*meta.frame_keys);
        for (const char* property : SET_KEYS) {
          if (const auto* entry = find_header_entry(property)) image_keys->keydb.erase(entry->keyword);
        }
        set_dict_value(*image_keys, "interleave_sets_gathered", std::to_string(this->sets_gathered));
        meta.frame_keys      = std::move(image_keys);
        meta.height          = this->science_rows;
        meta.sequence_number = this->image_count++;
        this->write(this->image.data(), image_bytes, meta, true);
      }
    }

    void InterleaveSplitter::write(const char* data, size_t bytes, const FrameMetadata &meta,
                                   bool to_fits) {
      for (auto &output : this->camera.frame_outputs) {
        if (!to_fits && dynamic_cast<const FitsWriter*>(output.get()) != nullptr) continue;
        output->write(data, bytes, meta);
      }
    }
  }


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
    place_current_thread(PROCESSING_THREAD_NAME,
                         static_cast<HispecTrackingCamera*>(this->interface)->processing_placement());
    if (this->is_debug) { logwrite(function, continuous ? "enter (continuous)" : "enter"); }

    // whichever flag ends this loop, in this mode
    auto finished = [this, continuous] {
      return continuous ? this->stop_consumer.load() : this->is_producer_finished.load();
    };

    uint64_t sequence_number = 0;

    // Decided once: what a frame is published as cannot change under a running exposure
    auto* hispec = static_cast<HispecTrackingCamera*>(this->interface);
    std::optional<InterleaveSplitter> splitter;
    if (hispec->is_interleave() && hispec->interleave_output_mode() != InterleaveOutput::Raw) {
      splitter.emplace(*hispec);
    }

    while (true) {
      std::shared_ptr<HispecFrame> buf;
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

      if (splitter) splitter->dispatch(*buf, this->frame_metadata(*buf, sequence_number++));
      else          this->dispatch_one(buf, sequence_number++);
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

    // Named rather than merged: a bare failure here reads as "the abort
    // failed" when the abort worked and a thread had flagged something else
    const std::string function("Camera::ExposureModeHispecTrackingBase::stop_freerun");
    if (this->is_producer_error) logwrite(function, "ERROR the acquisition thread flagged an error");
    if (this->is_consumer_error) logwrite(function, "ERROR the processing thread flagged an error");

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
    place_current_thread(ACQUISITION_THREAD_NAME, hispec->acquisition_placement());

    if (controller->selectedmode.empty()) {
      hispec->log_error(function, "no camera mode selected",
                 "controller->selectedmode is empty, so the frame geometry is unknown; "
                 "run \"load\" or \"mode <name>\" first");
      this->is_producer_error = true;
      return;
    }
    if (controller->expose_param.empty()) {
      hispec->log_error(function, "cannot trigger an exposure",
                 "EXPOSE_PARAM is absent from the .cfg, so there is no Archon "
                 "parameter to write the frame count to");
      this->is_producer_error = true;
      return;
    }

    // Read once so this thread has one fixed behaviour for its whole life
    const bool freerun = this->is_freerun.load();
    const bool interleaved = hispec->is_interleave();

    this->header_set = build_header_set(false, freerun);

    // One trigger yields the whole sequence. The interleaved readout is the
    // exception: there a trigger yields a single pass, a frame per set, so
    // nseq counts passes and each one is triggered separately.
    const int nseq = this->nseq.load();
    const int trigger_count = interleaved ? 1 : nseq;
    const int sets_per_pass = hispec->interleave().sets_per_pass();
    const long long frames_wanted = hispec->frames_for(nseq);

    auto* mode = &controller->modemap[controller->selectedmode];
    const int num_detect = mode->geometry.num_detect;
    const int fallback_bpp = (mode->samplemode == 1) ? 4 : 2;

    const auto await_sequencer_idle = [this]() {
      while (std::chrono::steady_clock::now() < this->sequencer_settled && !this->interface->is_aborted()) {
        std::this_thread::sleep_for(std::chrono::microseconds(FRAME_POLL_USEC));
      }
    };

    // A trigger written while the sequencer still runs the previous sequence's
    // last exposure delay is overwritten by its own count down, and nothing is
    // read. Waited out here, ahead of the baseline below: the idle frames read
    // meanwhile must not count as this sequence's.
    if (const auto wait = this->sequencer_settled - std::chrono::steady_clock::now(); wait.count() > 0) {
      logwrite(function, "waiting "+std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(wait).count())+
                         " msec for the previous sequence's exposure delay");
    }
    await_sequencer_idle();

    // Newest frame the controller has, taken before the trigger. Taken after it,
    // a frame the sequencer has already begun looks like one that predates the
    // request, and the sequence comes up a frame short. The ACF idles by reading
    // frames out (reset_while_idling), so this can be an idle frame rather than
    // ours; that costs one stale frame at the head instead of a missing one at
    // the tail. Never derived from lastframe, which get_frame_status() reports
    // as 0 whenever no buffer is complete.
    int baseline = 0;
    if (controller->get_frame_status() == NO_ERROR) {
      const auto &bufframen = controller->frameinfo.bufframen;
      baseline = *std::max_element(bufframen.begin(), bufframen.end());
    }

    // A pass leaves the sequencer at the end of the science window, so the
    // interleaved readout is pointed back at its start before every trigger
    const auto trigger = [&]() -> long {
      if (interleaved && hispec->arm_interleave_pass() != NO_ERROR) {
        hispec->log_error(function, "pass not armed",
                   "the Archon refused H2RG_rows_skip or interleave_nsets; check that "
                   "both exist in the loaded ACF");
        return ERROR;
      }
      long e = controller->prep_parameter(controller->expose_param, trigger_count);
      if (e == NO_ERROR) e = controller->load_parameter(controller->expose_param, trigger_count);
      if (e != NO_ERROR) {
        hispec->log_error(function, "exposure trigger rejected",
                   "the Archon refused FASTPREPPARAM/FASTLOADPARAM "+
                   controller->expose_param+"="+std::to_string(trigger_count)+
                   "; check that this parameter name matches the ACF exactly, it is "
                   "case sensitive");
      }
      return e;
    };

    // 64-bit: in freerun these count for the life of the session, and signed
    // overflow is undefined behaviour, not a harmless wrap.
    // Instantiate all variables possible so we don't have to do it in the loop, save time and avoid memory fragmentation.
    long long frames_read = 0;
    this->frames_acquired.store(0);
    this->frames_dropped.store(0);
    int consecutive_errors = 0;  //!< reset by every good frame; see MAX_CONSECUTIVE_ERRORS
    int pass = -1;
    int next_set = 0;  //!< interleaved: the set the next frame belongs to
    uint64_t guide_changes = hispec->guide_position().changes;
    std::chrono::steady_clock::time_point last_frame_at;

    for (long long i = 0; freerun || i < frames_wanted; ++i) {
      // An interleaved pass is re-armed only once the sequencer has finished the last
      const bool triggering = (i == 0) || (interleaved && next_set >= sets_per_pass);
      if (triggering && interleaved) await_sequencer_idle();
      // Checked before the trigger, so a stopped exposure triggers nothing more
      if (this->interface->is_aborted()) break;

      if (triggering) {
        if (trigger() != NO_ERROR) {
          this->is_producer_error = true;
          return;
        }
        next_set = 0;
        ++pass;
      }

      const long long dropped_before = this->frames_dropped.load();
      if (this->wait_for_frame(baseline, triggering) != NO_ERROR) {
        // wait_for_frame reports an abort the same way it reports a timeout,
        // so without this a deliberate stop is returned to the caller as a
        // failed abort
        if (!this->interface->is_aborted()) this->is_producer_error = true;
        return;
      }
      if (this->interface->is_aborted()) break;
      // Skipped frames still belong to the sequence, so it ends where the controller's does
      const long long skipped = this->frames_dropped.load() - dropped_before;
      i += skipped;
      int set = 0;
      if (interleaved) {
        // Counted with the skipped ones, so the set stays right across a gap
        set = next_set + static_cast<int>(skipped);
        next_set = set + 1;
        if (next_set >= sets_per_pass) {
          this->sequencer_settled = sequencer_idle_after(std::chrono::steady_clock::now(), controller->get_exptime());
        }
      }

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

      auto imagebuffer = std::make_shared<HispecFrame>();
      try { imagebuffer->rawpixels = std::shared_ptr<char[]>(new char[nbytes]); }
      catch (const std::exception &ex) {
        hispec->log_error(function, "out of memory",
                   "could not allocate "+std::to_string(nbytes)+" bytes for frame "+
                   std::to_string(i+1)+" of "+std::to_string(frames_wanted)+": "+ex.what());
        this->is_producer_error = true;
        break;
      }
      // A failed fetch ends a one-shot sequence, but in freerun it is just a
      // lost frame: one bad block header should not tear down the session.
      if (this->fetch_frame(idx, bufblocks, imagebuffer->rawpixels.get(), nbytes) != NO_ERROR) {
        if (!freerun) { this->is_producer_error = true; break; }
        if (++consecutive_errors > MAX_CONSECUTIVE_ERRORS) {
          hispec->log_error(function, "freerun ended",
                     std::to_string(consecutive_errors)+" consecutive fetch failures "
                     "exceeded the limit of "+std::to_string(MAX_CONSECUTIVE_ERRORS)+
                     "; the preceding fetch_frame errors give the cause");
          this->is_producer_error = true;
          break;
        }
        continue;
      }
      consecutive_errors = 0;

      imagebuffer->width           = fw;
      imagebuffer->height          = fh;
      imagebuffer->bytes_per_pixel = fbpp;
      if (interleaved) {
        // A change count that moved on, or is odd, means the window was being
        // rewritten at some point while this frame was read out
        const GuidePosition guide = hispec->guide_position();
        const bool moved = (guide.changes != guide_changes) || (guide.changes % 2 != 0);
        guide_changes = guide.changes;
        imagebuffer->interleave = InterleaveTag{pass, set, guide, moved};
      }
      imagebuffer->bufframen_slice.push_back(controller->frameinfo.bufframen[idx]);
      imagebuffer->buftimestamp_slice.push_back(controller->frameinfo.buftimestamp[idx]);
      this->enqueue(std::move(imagebuffer));
      ++frames_read;
      this->frames_acquired.store(frames_read);
      last_frame_at = std::chrono::steady_clock::now();
    }

    // So the command returns with the sequencer idle, ready for the next one.
    // Other readouts return at once and leave the wait to the next sequence,
    // since nothing is written between the two that the sequencer could undo.
    if (interleaved) {
      await_sequencer_idle();
    }
    else if (!freerun && frames_read > 0 && !this->interface->is_aborted()) {
      this->sequencer_settled = sequencer_idle_after(last_frame_at, controller->get_exptime());
    }

    logwrite(function, freerun
             ? "freerun acquisition stopped after " + std::to_string(frames_read) +
               " frames, " + std::to_string(this->frames_dropped.load()) + " dropped"
             : "sequence complete: " + std::to_string(frames_read) + " of " +
               std::to_string(frames_wanted) + " frames, " +
               std::to_string(this->frames_dropped.load()) + " dropped");
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
    place_current_thread(ACQUISITION_THREAD_NAME, hispec->acquisition_placement());

    if (controller->selectedmode.empty()) {
      logwrite(function, "ERROR no mode selected");
      this->is_producer_error = true;
      return;
    }

    // Read once so this thread has one fixed behaviour for its whole life
    const bool freerun = this->is_freerun.load();

    this->header_set = build_header_set(true, freerun);

    int nseq = 1;
    const std::string args = this->get_args_string();
    if (!args.empty()) {
      try { nseq = std::stoi(args); }
      catch (const std::exception &e) {
        logwrite(function, "ERROR invalid sequence count: " + args);
        this->is_producer_error = true;
        return;
      }
    }

    auto* mode = &controller->modemap[controller->selectedmode];
    const int bpp = (mode->samplemode == 1) ? 4 : 2;

    Utils::TimingStats fetch_stats;
    Utils::TimingStats archon_ts_deltas;
    uint64_t prev_archon_ts = 0;

    // The Archon's maximum rather than this window's size. image_memory comes
    // from detector_pixels, which the guiding path leaves at zero, so sizing
    // from it gives one block and every frame above 1024 bytes is discarded.
    const uint32_t bufsize = static_cast<uint32_t>(
        block_align(MAX_AUTOFETCH_FRAME_BYTES));
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
    // Over the control connection, not the primary one: enabling autofetch
    // pushes whatever frames the controller already held, so a reply on the
    // streaming socket arrives behind them and never matches. stop_autofetch
    // sends its own commands this way for the same reason.
    const std::string trigger = controller->expose_param + " " + std::to_string(nseq);
    long e = controller->send_control_cmd("FASTPREPPARAM " + trigger);
    if (e == NO_ERROR) e = controller->send_control_cmd("FASTLOADPARAM " + trigger);
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

      uint32_t frame_width = 0;
      uint32_t frame_height = 0;
      if (read_autofetch_frame(hispec, fetch_stats, archon_ts_deltas, prev_archon_ts,
                               frame_width, frame_height) != NO_ERROR) {
        // Disarming aborts the session, so a read failing here is the stream
        // ending rather than a fault; the drain below handles what is left
        if (!this->interface->is_aborted()) this->is_producer_error = true;
        break;
      }

      const auto idx = controller->frameinfo.index.load();

      // From the frame itself: host geometry does not account for taps
      const uint32_t fw   = frame_width;
      const uint32_t fh   = frame_height;
      const uint32_t fbpp = 2;
      const size_t   nbytes = static_cast<size_t>(fw) * fh * fbpp;

      auto imagebuffer = std::make_shared<HispecFrame>();
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

    // A session cut short leaves frames in flight. The disarm has already
    // stopped the push, so this terminates, and reading them here on the
    // thread that owns the socket is what keeps the teardown orderly: without
    // it the process aborts on every interrupted run.
    if (this->interface->is_aborted()) {
      logwrite(function, "drained "+std::to_string(drain_until_quiet(controller))+
                         " bytes left in flight");
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
