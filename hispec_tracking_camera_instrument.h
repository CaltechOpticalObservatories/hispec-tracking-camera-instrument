/**
 * @file    Instruments/hispec_tracking_camera/hispec_tracking_camera_instrument.h
 * @brief   HISPEC Tracking Camera instrument properties and acquisition logic
 * @author  Michael Langmayr <langmayr@astro.caltech.edu>
 *
 */
#pragma once

#include "archon_interface.h"
#include "hispec_tracking_camera_exposure_modes.h"

#include <atomic>
#include <string>
#include <unordered_map>

namespace Camera {

  class HispecTrackingCamera : public ArchonInterface {
    public:
      long instrument_cmd(const std::string &cmd,
                          const std::string &args,
                          std::string &retstring) override;

      bool is_instrument_command(const std::string &cmd);

      std::vector<std::string> instrument_commands() const override;

      void configure_instrument() override;

      std::vector<std::string> get_exposure_modes() override;
      long set_exposure_mode(const std::string &modein, const std::vector<std::string> &modeargs) override;
      long expose(const std::string args, std::string &retstring) override;

      /**
       * @brief  abort, and tear down a freerun session if one is running
       * @details  The base abort sets the abort state, which both freerun loops
       *           watch. This then joins them, so when abort returns the session
       *           is fully stopped and freerun can be restarted immediately.
       */
      long abort(const std::string args, std::string &retstring) override;

      std::string default_exposure_mode_name() const override {
        return std::string(HispecTrackingCameraExposureMode::DEFAULT);
      }

      /**
       * @brief      one-line snapshot of what the camera was doing
       * @details    Appended to every error log so a failure carries the state
       *             that produced it, rather than leaving it to be reconstructed
       *             from surrounding lines that a concurrent command may have
       *             interleaved.
       */
      std::string state_summary() const;

      /**
       * @brief      log an error with its root cause and the camera state
       * @param[in]  brief   what failed, in a few words
       * @param[in]  detail  why it failed, as specifically as the call site knows
       * @details    For the acquisition threads, which have no caller to answer.
       */
      void log_error(const std::string &function, const std::string &brief,
                     const std::string &detail) const;

      bool is_windowed() const { return is_window; }
      int detector_rows() const { return h2rg_max_pixel + 1; }
      int window_vstart() const { return win_vstart; }
      int window_hstart() const { return win_hstart; }

    private:
      using CmdHandler = long (HispecTrackingCamera::*)(const std::string&, std::string&);
      static const std::unordered_map<std::string, CmdHandler> command_handlers_;
      static const std::unordered_map<std::string, std::string> _exposure_modes;

      // H2RG detector commands
      long h2rg_init(const std::string &args, std::string &retstring);
      long _exposure_mode(const std::string &args, std::string &retstring);
      long _autofetch_mode(const std::string &args, std::string &retstring);
      long mode(const std::string &args, std::string &retstring);
      long roi(const std::string &args, std::string &retstring);
      long roi_exec(const std::string &args, std::string &retstring);
      long fullframe(const std::string &args, std::string &retstring);
      long validate_roi(const std::string &args, std::string &retstring);
      long window_mode(const std::string &args, std::string &retstring);
      long guiding_roi(const std::string &args, std::string &retstring);

      long freerun(const std::string &args, std::string &retstring);
      long _debug(const std::string &args, std::string &retstring);
      long _take_stats(const std::string &args, std::string &retstring);

      // Acquire a whole sequence from one Archon trigger, see the .cpp for why
      long run_exposure_sequence(const std::string &args, std::string &retstring);

      // Per-frame readout deadline for the current geometry and exposure time
      int readout_timeout_msec() const;

      // Short reason to the caller, root cause plus state to the log
      long fail_detailed(const std::string &function, std::string &retstring,
                         const std::string &brief, const std::string &detail) const;

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

      // Readout deadline model, from the .cfg since it varies per system
      double pixel_time_usec{0.0};
      double readout_margin_msec{0.0};

      std::atomic<bool> exposure_in_progress{false};
      bool is_freerunning{false};
      std::atomic<bool> is_freerun_active{false};  //!< true while the background freerun loop is running
      bool is_debug{false};
      bool take_stats{false};
  };

}
