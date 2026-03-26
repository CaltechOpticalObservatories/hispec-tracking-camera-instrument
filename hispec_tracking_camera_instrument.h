/**
 * @file    Instruments/hispec_tracking_camera/hispec_tracking_camera_instrument.h
 * @brief   HISPEC Tracking Camera instrument properties and acquisition logic
 * @author  Michael Langmayr <langmayr@astro.caltech.edu>
 *
 */
#pragma once

#include "archon_interface.h"
#include "timing_stats.h"

namespace Camera {

  class HispecTrackingCamera : public ArchonInterface {
    public:
      long instrument_cmd(const std::string &cmd,
                          const std::string &args,
                          std::string &retstring) override;

      void configure_instrument() override;

      std::vector<std::string> get_exposure_modes() override;
      long set_exposure_mode(const std::string &modein, const std::vector<std::string> &modeargs) override;

    private:
      long readout(const std::string &args, std::string &retstring);
      long expose(const std::string &args, std::string &retstring);

      // Autofetch frame reading — reads a single frame from the Archon socket
      // when autofetch mode is active. Returns frame data in the controller's framebuf.
      long readout_autofetch();

      // H2RG detector commands
      long h2rg_init(const std::string &args, std::string &retstring);
      long window_mode(const std::string &args, std::string &retstring);
      long window_roi(const std::string &args, std::string &retstring);

      // Helper to send an INREG command and optionally clock it to the detector
      long send_inreg(int module, int inreg, int value);
      long send_inreg_clocked(int module, int inreg, int value);

      Utils::TimingStats fetch_stats;
      Utils::TimingStats archon_ts_deltas;
      uint64_t prev_archon_ts{0};

      // Window mode state
      bool is_window{false};
      int win_vstart{0};
      int win_vstop{2047};
      int win_hstart{0};
      int win_hstop{2047};
      int taplines_store{0};
      std::string tapline0_store;

      static constexpr int AUTOFETCH_HEADER_LEN = 36;

      // Set in configure_instrument()
      int lvds_module{0};
      int h2rg_max_pixel{0};
  };

}
