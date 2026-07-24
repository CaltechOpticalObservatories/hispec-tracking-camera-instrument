/**
 * @file     Instruments/hispec_tracking_camera/hispec_tracking_camera_exposure_modes.h
 * @brief    declares HISPEC Tracking Camera-specific exposure mode classes
 * @author   Michael Langmayr <langmayr@astro.caltech.edu>
 *
 */

#pragma once

#include "exposure_modes.h"        // ExposureMode base class
#include "archon_exposure_modes.h" // ArchonImageBuffer

#include <memory>
#include <queue>

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

      void image_processing_thread() override;   // consumer: dispatches queued frames
      void set_args(const std::vector<std::string> &a) { this->args = a; }

    protected:
      void enqueue(std::shared_ptr<ArchonImageBuffer> buf);
      std::queue<std::shared_ptr<ArchonImageBuffer>> imagebuf_queue;
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
