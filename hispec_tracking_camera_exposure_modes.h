/**
 * @file     Instruments/hispec_tracking_camera/hispec_tracking_camera_exposure_modes.h
 * @brief    declares HISPEC Tracking Camera-specific exposure mode classes
 * @author   Michael Langmayr <langmayr@astro.caltech.edu>
 *
 */

#pragma once

#include "exposure_modes.h"        // ExposureMode base class
#include "archon_exposure_modes.h" // ArchonImageBuffer

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <thread>
#include <vector>

namespace Common { class FitsKeys; }

namespace Camera {

  namespace HispecTrackingCameraExposureMode {
    constexpr const char* DEFAULT   = "DEFAULT";
    constexpr const char* AUTOFETCH = "AUTOFETCH";
    constexpr const char* ALLMODES[] = {DEFAULT};
  }

  class HispecTrackingCamera;

  // Shared base: owns the frame queue and the consumer; subclasses implement
  // only the producer (image_acquisition_thread).
  class ExposureModeHispecTrackingBase : public ExposureModeTemplate<Camera::ArchonInterface> {
    public:
      ExposureModeHispecTrackingBase(Camera::ArchonInterface* iface)
        : ExposureModeTemplate<Camera::ArchonInterface>(iface) { }

      ~ExposureModeHispecTrackingBase() override;

      /**
       * @brief  one-shot consumer, spawned and joined by do_expose()
       * @details  Always terminates when the producer is finished and the queue
       *           is drained, whatever is_freerun says. do_expose() joins this
       *           thread unconditionally, so it must never be the endless one.
       */
      void image_processing_thread() override;

      /**
       * @brief  start a freerun session and return immediately
       * @return ERROR|NO_ERROR
       * @details  Does not go through do_expose(), which spawns and joins a
       *           thread pair per call -- per frame, in freerun. This spawns one
       *           producer and one consumer for the whole session and returns,
       *           so no thread is created per frame and no caller is parked.
       *           Both are real thread objects, so priority and CPU affinity can
       *           be set on each.
       *
       *           Ends on abort, or on a producer error. Reap with stop_freerun().
       */
      long start_freerun();

      /**
       * @brief  stop a freerun session and join both threads
       * @return ERROR|NO_ERROR accumulated from the producer and consumer
       * @details  Idempotent, and safe when no session was ever started. Blocks
       *           only as long as the in-flight frame read takes. Called from
       *           abort(), and from the destructor as a safety net.
       */
      long stop_freerun();

      /** @brief  true while the freerun producer is still running */
      bool is_freerun_running() const { return this->session_running.load(); }

      void set_args(const std::vector<std::string> &a) { this->args = a; }
      void set_debug(bool d) { this->is_debug = d; }
      void set_take_stats(bool t) { this->take_stats = t; }

      // Written by the command thread, read by the producer/consumer threads,
      // so these have to be atomic: a plain bool here is a data race, and the
      // compiler may hoist the load out of a loop so a change is never seen.
      std::atomic<bool> is_debug{false};    //!< true to log per-frame debug info
      std::atomic<bool> is_freerun{false};  //!< true: producer loops continuously
      std::atomic<bool> take_stats{false};  //!< true to collect timing statistics

    protected:
      void enqueue(std::shared_ptr<ArchonImageBuffer> buf);

      /**
       * @brief  resolve how many frames this exposure should read
       * @return the frame count, or nullopt if the mode args do not parse
       * @details  The count is written straight into the ACF's Expose
       *           parameter, which the Archon limits to 20 bits, so an
       *           out-of-range value is rejected here rather than left to
       *           throw from prep_parameter mid-exposure.
       */
      std::optional<int> sequence_count();

      // Built once at the start of image_acquisition_thread(), read by the
      // consumer, so the values cannot shift mid-session.
      std::shared_ptr<Common::FitsKeys> build_header_set(const std::string &operational_mode,
                                                         const std::string &subframe_mode,
                                                         bool is_freerun,
                                                         int n_reads);

      /**
       * @brief  the consumer loop
       * @param[in]  continuous  false: exit when the producer finishes (one exposure)
       *                         true:  exit only on abort or stop_consumer (session)
       * @details  The mode is a parameter, not a flag read from the object, so a
       *           given consumer thread has one fixed behaviour for its whole
       *           life and cannot change identity mid-loop.
       */
      void process_frames(bool continuous);

      /** @brief  build the metadata for one frame and fan it out to frame_outputs */
      void dispatch_one(const std::shared_ptr<ArchonImageBuffer> &buf, uint64_t sequence_number);

      /**
       * @brief  tell the session consumer to drain the queue and exit
       * @details  Both flags are set under queue_mutex and followed by
       *           notify_all. Set outside the lock, a consumer that has just
       *           evaluated its wait predicate sleeps forever: the producer is
       *           gone, so nothing will ever notify it again.
       */
      void signal_consumer_stop();

      /**
       * @brief      fetch one frame from an Archon buffer into dest
       * @param[in]  bufindex    0-based frameinfo index of the buffer to read
       * @param[in]  bufblocks   number of BLOCK_LEN blocks the Archon will send
       * @param[out] dest        destination for the pixel data
       * @param[in]  dest_bytes  capacity of dest, in bytes
       * @return     ERROR|NO_ERROR
       *
       * @details    Moves bytes and nothing else: the caller decides how big a
       *             frame is, so exactly one place in the code sizes a frame.
       *             dest_bytes is passed so that relationship is checked here
       *             rather than assumed -- an undersized dest is an early
       *             return, not a heap overflow.
       *
       *             Equivalent to ArchonController::read_frame() minus its
       *             unconditional per-call log, and reading the fetch in bulk
       *             instead of a poll(), an ioctl spin and two socket reads per
       *             kilobyte (~1.5us/KB, ~12ms on an 8MB frame).
       *
       *             Returning a status rather than aborting lets the caller
       *             decide whether a bad frame ends the run: in freerun a
       *             single failed fetch should not tear down the session.
       */
      long fetch_frame(int bufindex, unsigned bufblocks, char* dest, size_t dest_bytes);

      std::queue<std::shared_ptr<ArchonImageBuffer>> imagebuf_queue;

      // Staging buffer for one FETCH, reused for the life of the object: the
      // Archon interleaves a 4-byte header every BLOCK_LEN bytes, so the wire
      // image is read whole and the payload compacted out afterwards. Grows
      // once, then never reallocates.
      std::vector<char> fetch_buf;

      // Freerun session threads. Kept joinable so they can be reaped; a thread
      // still running against a destroyed object is undefined behaviour.
      std::thread producer_thread;
      std::thread consumer_thread;
      std::atomic<bool> stop_consumer{false};   //!< ask the consumer to drain and exit
      std::atomic<bool> session_running{false}; //!< true while the producer is alive
      std::shared_ptr<const Common::FitsKeys> header_set;
  };

  // Default: one frame per readout, read straight into its own buffer.
  class ExposureModeHispecTrackingDefault : public ExposureModeHispecTrackingBase {
    public:
      ExposureModeHispecTrackingDefault(Camera::ArchonInterface* iface)
        : ExposureModeHispecTrackingBase(iface) {
          type=HispecTrackingCameraExposureMode::DEFAULT;
        }
      void image_acquisition_thread() override;
  };

  // Autofetch: continuous streaming from the Archon via framebuf.
  class ExposureModeHispecTrackingAutofetch : public ExposureModeHispecTrackingBase {
    public:
      ExposureModeHispecTrackingAutofetch(Camera::ArchonInterface* iface)
        : ExposureModeHispecTrackingBase(iface) {
          type=HispecTrackingCameraExposureMode::AUTOFETCH;
        }
      void image_acquisition_thread() override;
  };

}
