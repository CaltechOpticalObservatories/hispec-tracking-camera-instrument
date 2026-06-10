/**
 * @file     Instruments/hispec_tracking_camera/hispec_tracking_camera_exposure_modes.cpp
 * @brief    implements HISPEC Tracking Camera-specific exposure modes
 * @author   Michael Langmayr <langmayr@astro.caltech.edu>
 *
 */

#include "archon_controller.h"
#include "archon_exposure_modes.h"
#include "hispec_tracking_camera_exposure_modes.h"
#include "hispec_tracking_camera_instrument.h"
#include "timing_stats.h"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>

namespace Camera {

  // Autofetch frame layout: 36-byte ASCII header followed by pixel data
  static constexpr int AUTOFETCH_HEADER_LEN = 36;


  /***** Camera::ExposureModeFullFrame::read_autofetch_frame ******************/
  /**
   * @brief  Read one autofetch frame: blocks until 36-byte header + pixels
   *         arrive on the Archon socket, then parses the header
   */
  static long read_autofetch_frame(HispecTrackingCamera* hispec,
                                   Utils::TimingStats &fetch_stats,
                                   Utils::TimingStats &archon_ts_deltas,
                                   uint64_t &prev_archon_ts) {
    const std::string function("Camera::ExposureModeFullFrame::read_autofetch_frame");
    const auto fetch_start = std::chrono::steady_clock::now();

    auto* controller = hispec->controller;
    auto* mode = &controller->modemap[controller->selectedmode];
    const int num_detect = mode->geometry.num_detect;
    const auto pixel_bytes = static_cast<size_t>(hispec->camera_info.image_memory * num_detect);
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
  /***** Camera::ExposureModeFullFrame::read_autofetch_frame ******************/


  /***** Camera::ExposureModeFullFrame::image_acquisition_thread **************/
  /**
   * @brief  Producer thread: validate, prep (non-autofetch), loop fetching
   *         frames and dispatching each to the instrument's frame_outputs
   */
  void ExposureModeFullFrame::image_acquisition_thread() {
    const std::string function("Camera::ExposureModeFullFrame::image_acquisition_thread");
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
    const int num_detect = mode->geometry.num_detect;
    const int bpp = (mode->samplemode == 1) ? 4 : 2;
    const auto pixel_bytes = static_cast<size_t>(hispec->camera_info.image_memory * num_detect);

    Utils::TimingStats fetch_stats;
    Utils::TimingStats archon_ts_deltas;
    uint64_t prev_archon_ts = 0;

    // Non-autofetch path: allocate framebuf and trigger an exposure on the Archon
    if (!hispec->is_autofetch_mode) {
      const uint32_t bufsize = static_cast<uint32_t>(
          std::ceil(static_cast<double>(hispec->camera_info.image_memory * num_detect + BLOCK_LEN - 1) / BLOCK_LEN) * BLOCK_LEN);
      if (controller->allocate_framebuf(bufsize) != NO_ERROR) {
        logwrite(function, "ERROR unable to allocate frame buffer");
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
    }

    int frames_read = 0;
    for (int i = 0; i < nseq; ++i) {
      if (this->interface->is_aborted()) break;

      long e;
      if (hispec->is_autofetch_mode) {
        e = read_autofetch_frame(hispec, fetch_stats, archon_ts_deltas, prev_archon_ts);
      }
      else {
        e = controller->wait_for_readout();
        if (e == NO_ERROR) {
          // read_frame mutates its second arg; pass a local copy to preserve framebuf
          char* imagebuf = controller->framebuf;
          e = controller->read_frame(ArchonController::FRAME_IMAGE, imagebuf);
        }
        if (e != NO_ERROR) logwrite(function, "ERROR reading frame from controller");
      }
      if (e != NO_ERROR) {
        this->is_producer_error = true;
        break;
      }

      const char* pixel_data = hispec->is_autofetch_mode
        ? controller->framebuf + AUTOFETCH_HEADER_LEN
        : controller->framebuf;

      const auto idx = controller->frameinfo.index.load();
      Camera::FrameMetadata meta;
      meta.frame_number    = controller->frameinfo.bufframen[idx];
      meta.timestamp       = controller->frameinfo.buftimestamp[idx];
      meta.width           = mode->geometry.pixelcount;
      meta.height          = mode->geometry.linecount;
      meta.bytes_per_pixel = bpp;

      hispec->dispatch_frame(pixel_data, pixel_bytes, meta);
      ++frames_read;
    }

    if (!archon_ts_deltas.empty()) {
      logwrite(function, archon_ts_deltas.summary("archon frame interval"));
    }
    if (!fetch_stats.empty()) {
      logwrite(function, fetch_stats.summary("host readout duration"));
    }
    logwrite(function, "sequence complete: " + std::to_string(frames_read) +
             " of " + std::to_string(nseq) + " frames");
  }
  /***** Camera::ExposureModeFullFrame::image_acquisition_thread **************/

}
