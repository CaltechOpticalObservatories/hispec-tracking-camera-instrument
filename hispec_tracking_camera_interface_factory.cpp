#include "hispec_tracking_camera_instrument.h"
#include "camera_interface.h"

namespace Camera {

  /***** Camera::Interface::create ********************************************/
  /**
   * @brief      factory function to create pointer to HispecTrackingCamera
   * @return     unique_ptr<HispecTrackingCamera>
   *
   */
  std::unique_ptr<Interface> Interface::create() {
    return std::make_unique<HispecTrackingCamera>();
  }
  /***** Camera::Interface::create ********************************************/

}
