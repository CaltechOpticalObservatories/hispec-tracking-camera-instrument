/**
 * @file    Instruments/hispec_tracking_camera/hispec_tracking_camera_instrument.h
 * @brief   HISPEC Tracking Camera instrument properties and acquisition logic
 * @author  Michael Langmayr <langmayr@astro.caltech.edu>
 *
 */
#pragma once

#include "archon_interface.h"
#include "frame_output_factory.h"
#include "hispec_tracking_camera_exposure_modes.h"

#include <string>
#include <unordered_map>

namespace Camera {

  class HispecTrackingCamera : public ArchonInterface {
    public:
      long instrument_cmd(const std::string &cmd,
                          const std::string &args,
                          std::string &retstring) override;

      bool is_instrument_command(const std::string &cmd) override;

      void configure_instrument() override;

      std::vector<std::string> get_exposure_modes() override;
      long set_exposure_mode(const std::string &modein, const std::vector<std::string> &modeargs) override;

      std::string default_exposure_mode_name() const override {
        return std::string(HispecTrackingCameraExposureMode::FULLFRAME);
      }

    private:
      using CmdHandler = long (HispecTrackingCamera::*)(const std::string&, std::string&);
      static const std::unordered_map<std::string, CmdHandler> command_handlers_;
      static const std::unordered_map<std::string, std::string> exposure_modes;

      // H2RG detector commands
      long h2rg_init(const std::string &args, std::string &retstring);
      long exposure_mode(const std::string &args, std::string &retstring);
      long roi(const std::string &args, std::string &retstring);
      long roi_exec(const std::string &args, std::string &retstring);
      long fullframe(const std::string &args, std::string &retstring);
      long validate_roi(const std::string &args, std::string &retstring);
      long window_mode(const std::string &args, std::string &retstring);
      long guiding_roi(const std::string &args, std::string &retstring);

      // Helper to send an INREG command and optionally clock it to the detector
      long send_inreg(int module, int inreg, int value);
      long send_inreg_clocked(int module, int inreg, int value);

      // Window mode state
      bool is_window{false};
      int win_vstart{0};
      int win_vstop{2047};
      int win_hstart{0};
      int win_hstop{2047};
      int taplines_store{0};
      std::string tapline0_store;
      std::string cur_exposure_mode;

      // Set in configure_instrument()
      int lvds_module{0};
      int h2rg_max_pixel{0};
  };

}
