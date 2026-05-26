/**
 * @file     Instruments/hispec_tracking_camera/hispec_tracking_camera_Readout_modes.h
 * @brief    declares HISPEC Tracking Camera-specific Readout mode classes
 * @details  Declares classes that implement Readout modes supported by
 *           HISPEC Tracking Camera. These classes override virtual functions
 *           in the ReadoutMode base class to provide mode-specific behavior.
 * @author   Michael Langmayr <langmayr@astro.caltech.edu>
 *
 */

#pragma once

#include "exposure_modes.h"  // ReadoutMode base class

namespace Camera {

  /**
   * @namespace  all recognized Readout modes for HispecTrackingCamera
   */
  namespace HispecTrackingCameraReadoutMode {
    constexpr const char* RXR = "RXR";
    constexpr const char* RX = "RX";
    constexpr const char* UTR_RR = "UTR_RR";
    constexpr const char* UTR_GR = "UTR_GR";
    constexpr const char* ALLMODES[] = {
      RXR,
      RX,
      UTR_RR,
      UTR_GR
    };
  }

  class HispecTrackingCamera;

  class ReadoutModeUTR_RR : public ReadoutModeTemplate<Camera::ArchonInterface> {
    public:
      ReadoutModeUTR_RR(Camera::ArchonInterface* iface)
        : ReadoutModeTemplate<Camera::ArchonInterface>(iface) {
          type=HispecTrackingCameraReadoutMode::UTR_RR;
        }

    long expose() override;
  };

  class ReadoutModeUTR_GR : public ReadoutModeTemplate<Camera::ArchonInterface> {
    public:
      ReadoutModeUTR_GR(Camera::ArchonInterface* iface)
        : ReadoutModeTemplate<Camera::ArchonInterface>(iface) {
          type=HispecTrackingCameraReadoutMode::UTR_GR;
        }

    long expose() override;
  };

  class ReadoutModeRX : public ReadoutModeTemplate<Camera::ArchonInterface> {
    public:
      ReadoutModeRX(Camera::ArchonInterface* iface)
        : ReadoutModeTemplate<Camera::ArchonInterface>(iface) {
          type=HispecTrackingCameraReadoutMode::RX;
        }

    long expose() override;
  };

  class ReadoutModeRXR : public ReadoutModeTemplate<Camera::ArchonInterface> {
    public:
      ReadoutModeRXR(Camera::ArchonInterface* iface)
        : ReadoutModeTemplate<Camera::ArchonInterface>(iface) {
          type=HispecTrackingCameraReadoutMode::RXR;
        }

    long expose() override;
  };

}
