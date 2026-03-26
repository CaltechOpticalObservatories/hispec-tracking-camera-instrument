/**
 * @file     Instruments/hispec_tracking_camera/hispec_tracking_camera_exposure_modes.cpp
 * @brief    implements HISPEC Tracking Camera-specific exposure modes
 * @author   Michael Langmayr <langmayr@astro.caltech.edu>
 *
 */

#include "archon_exposure_modes.h"
#include "hispec_tracking_camera_exposure_modes.h"
#include "hispec_tracking_camera_instrument.h"

namespace Camera {

  /***** Camera::ExposureModeTracking::expose *********************************/
  /**
   * @brief  implementation of hispec tracking camera-specific expose for Tracking mode
   * @details Delegates to HispecTrackingCamera::expose() which owns the
   *          acquisition loop and autofetch-aware readout logic.
   *
   */
  long ExposureModeTracking::expose() {
    const std::string function("Camera::ExposureModeTracking::expose");

    auto* tracking_camera = static_cast<HispecTrackingCamera*>(this->interface);
    std::string retstring;
    std::string args = this->get_args_string();
    return tracking_camera->instrument_cmd("expose", args, retstring);
  }
  /***** Camera::ExposureModeTracking::expose *********************************/

}
