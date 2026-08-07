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
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

namespace Camera {

  // Autofetch frame layout: 36-byte ASCII header followed by pixel data
  static constexpr int AUTOFETCH_HEADER_LEN = 36;

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


  /***** Camera::ExposureModeHispecTrackingBase::image_processing_thread *****/
  /**
   * @brief  Consumer thread: pop each queued frame and fan it out to the
   *         instrument's frame_outputs
   */
  void ExposureModeHispecTrackingBase::image_processing_thread() {
    const std::string function("Camera::ExposureModeHispecTrackingBase::image_processing_thread");
    logwrite(function, "enter");

    while (!this->interface->is_aborted()) {
      std::shared_ptr<ArchonImageBuffer> buf;
      {
        std::unique_lock<std::mutex> lock(this->queue_mutex);
        this->queue_cv.wait(lock, [this] {
            return !this->imagebuf_queue.empty() || this->is_producer_finished || this->interface->is_aborted();
            });
        if (this->interface->is_aborted()) break;
        if (this->imagebuf_queue.empty()) {
          if (this->is_producer_finished) {
            logwrite(function, "queue empty and producer finished");
            break;
          }
          continue;
        }
        buf = this->imagebuf_queue.front();
        this->imagebuf_queue.pop();
      }

      Camera::FrameMetadata meta;
      meta.frame_number    = buf->bufframen_slice.empty()    ? 0 : static_cast<uint64_t>(buf->bufframen_slice[0]);
      meta.timestamp       = buf->buftimestamp_slice.empty() ? 0 : buf->buftimestamp_slice[0];
      meta.width           = buf->width;
      meta.height          = buf->height;
      meta.bytes_per_pixel = buf->bytes_per_pixel;
      const size_t frame_bytes = static_cast<size_t>(buf->width) * buf->height * buf->bytes_per_pixel;
      this->interface->dispatch_frame(buf->rawpixels.get(), frame_bytes, meta);
    }

    logwrite(function, "exit");
  }
  /***** Camera::ExposureModeHispecTrackingBase::image_processing_thread *****/

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
    const int fallback_bpp = (mode->samplemode == 1) ? 4 : 2;

    long e = controller->prep_parameter(controller->expose_param, nseq);
    if (e == NO_ERROR) e = controller->load_parameter(controller->expose_param, nseq);
    if (e != NO_ERROR) {
      logwrite(function, "ERROR failed to initiate exposure");
      this->is_producer_error = true;
      return;
    }

    int frames_read = 0;
    for (int i = 0; i < nseq; ++i) {
      if (this->interface->is_aborted()) break;

      if (controller->wait_for_readout() == ERROR) {
        this->is_producer_error = true;
        break;
      }

      const auto idx = controller->frameinfo.index.load();

      // Geometry from the Archon-reported buffer dimensions (BUFnWIDTH/HEIGHT).
      uint32_t fw   = static_cast<uint32_t>(controller->frameinfo.bufwidth[idx]);
      uint32_t fh   = static_cast<uint32_t>(controller->frameinfo.bufheight[idx]);
      uint32_t fbpp = (controller->frameinfo.bufsample[idx] == 1) ? 4u : 2u;
      if (fw == 0 || fh == 0) {
        fw   = hispec->camera_info.detector_pixels[0];
        fh   = hispec->camera_info.detector_pixels[1];
        fbpp = static_cast<uint32_t>(fallback_bpp);
      }
      if (i == 0) {
        logwrite(function, "frame geometry " + std::to_string(fw) + "x" +
                 std::to_string(fh) + " bpp=" + std::to_string(fbpp));
      }

      // Size the buffer to the whole block-aligned frame and read straight into it.
      const size_t raw    = static_cast<size_t>(fw) * fh * fbpp * num_detect;
      const size_t nbytes = block_align(raw);
      auto imagebuffer = std::make_shared<ArchonImageBuffer>();
      try { imagebuffer->rawpixels = std::shared_ptr<char[]>(new char[nbytes]); }
      catch (const std::exception &ex) {
        logwrite(function, "ERROR allocating image buffer: " + std::string(ex.what()));
        this->is_producer_error = true;
        break;
      }
      char* p = imagebuffer->rawpixels.get();
      if (controller->read_frame(ArchonController::FRAME_IMAGE, p) == ERROR) {
        logwrite(function, "ERROR reading frame from controller");
        this->is_producer_error = true;
        break;
      }
      imagebuffer->width           = fw;
      imagebuffer->height          = fh;
      imagebuffer->bytes_per_pixel = fbpp;
      imagebuffer->bufframen_slice.push_back(controller->frameinfo.bufframen[idx]);
      imagebuffer->buftimestamp_slice.push_back(controller->frameinfo.buftimestamp[idx]);
      this->enqueue(std::move(imagebuffer));
      ++frames_read;
    }

    logwrite(function, "sequence complete: " + std::to_string(frames_read) +
             " of " + std::to_string(nseq) + " frames");
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

    int frames_read = 0;
    for (int i = 0; i < nseq; ++i) {
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
  /***** Camera::ExposureModeHispecTrackingAutofetch::image_acquisition_thread */

}
