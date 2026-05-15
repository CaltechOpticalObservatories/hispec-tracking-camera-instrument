/**
 * @file     Instruments/hispec_tracking_camera/hispec_tracking_camera_exposure_modes.h
 * @brief    declares HISPEC Tracking Camera-specific exposure mode classes
 * @details  Declares classes that implement exposure modes supported by
 *           HISPEC Tracking Camera. These classes override virtual functions
 *           in the ExposureMode base class to provide mode-specific behavior.
 * @author   Michael Langmayr <langmayr@astro.caltech.edu>
 *
 */

#pragma once

#include "exposure_modes.h"  // ExposureMode base class

namespace Camera {

  /**
   * @namespace  all recognized exposure modes for HispecTrackingCamera
   */
  namespace HispecTrackingCameraExposureMode {
    constexpr const char* FULLFRAME = "FULLFRAME";
    constexpr const char* ALLMODES[] = {FULLFRAME};
  }

  class HispecTrackingCamera;

  class ExposureModeFullFrame : public ExposureModeTemplate<Camera::ArchonInterface> {
    public:
      ExposureModeFullFrame(Camera::ArchonInterface* iface)
        : ExposureModeTemplate<Camera::ArchonInterface>(iface) {
          type=HispecTrackingCameraExposureMode::FULLFRAME;
        }

    void image_acquisition_thread() override;
  };

}
