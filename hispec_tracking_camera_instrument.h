#pragma once

#include "archon_interface.h"
#include "hispec_tracking_camera_exposure_modes.h"

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>
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
      long load_firmware(const std::string &args, std::string &retstring) override;
      long expose(const std::string args, std::string &retstring) override;

      /**
       * @brief  abort, and tear down a freerun session if one is running
       * @details  The base abort sets the abort state, which both freerun loops
       *           watch. This then joins them, so when abort returns the session
       *           is fully stopped and freerun can be restarted immediately.
       */
      long abort(const std::string args, std::string &retstring) override;

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

      const std::string& reference_amp() const { return refpix_amp; }
      const std::string& readout_mode() const { return cur_exposure_mode; }

      bool is_interleave() const { return cur_exposure_mode == "interleave"; }
      const InterleaveGeometry& interleave() const { return interleave_geometry; }
      InterleaveOutput interleave_output_mode() const { return interleave_output; }
      bool writes_guide_fits() const { return is_guide_fits_enabled; }

      /**
       * @brief      where the guide window is now, and whether it is moving
       * @details    For the acquisition threads, which tag every frame with it.
       *             Never waits on a move in progress.
       */
      GuidePosition guide_position() const;

      /**
       * @brief      point the sequencer at the start of the science window
       * @details    A pass leaves H2RG_rows_skip at the window's last row and
       *             interleave_nsets at zero, so both are rewritten before
       *             every trigger.
       * @return     ERROR|NO_ERROR
       */
      long arm_interleave_pass();

      /** @brief  pixel time in force, from the ACF when it carries one */
      double effective_pixel_time_usec() const;

      /** @brief  seconds to clock out one amplifier region in the selected mode */
      double frame_readout_sec() const;

      // Applied by each thread to itself on entry, so freerun and do_expose()
      // threads are placed the same way
      const ThreadPlacement& acquisition_placement() const { return acquisition_thread_placement; }
      const ThreadPlacement& processing_placement() const { return processing_thread_placement; }

    private:
      // The reference channel's tapline moves with the mode, its amp does not
      static constexpr const char* DEFAULT_REFPIX_AMP = "AM52";

      using CmdHandler = long (HispecTrackingCamera::*)(const std::string&, std::string&);
      static const std::unordered_map<std::string, CmdHandler> command_handlers_;
      static const std::unordered_map<std::string, std::string> _exposure_modes;

      // H2RG detector commands
      long h2rg_init(const std::string &args, std::string &retstring);
      long _exposure_mode(const std::string &args, std::string &retstring);
      long _autofetch_mode(const std::string &args, std::string &retstring);
      long stop_autofetch_stream(std::string &retstring);
      long set_parameter_offstream(const std::string &name, int value);
      long mode(const std::string &args, std::string &retstring);
      long roi(const std::string &args, std::string &retstring);
      long roi_exec(const std::string &args, std::string &retstring);
      long fullframe(const std::string &args, std::string &retstring);
      long validate_roi(const std::string &args, std::string &retstring);
      long window_mode(const std::string &args, std::string &retstring);
      long guiding_roi(const std::string &args, std::string &retstring);

      // Interleaved readout: science rows alternating with a guide window
      long interleave_science(const std::string &args, std::string &retstring);
      long interleave_guide(const std::string &args, std::string &retstring);
      long interleave_set(const std::string &args, std::string &retstring);
      long write_guide_window(int vstart, int vstop, int hstart, int hstop);

      // Send what the interleave commands recorded to the controller, ahead of a trigger
      long apply_interleave(std::string &retstring);

      // The geometry an interleaved exposure started with has to hold until it ends
      bool is_interleave_frozen() const {
        return is_interleave() && (exposes_in_progress.load() > 0 || is_freerun_active.load());
      }

      /** H2RG_columns and H2RG_rows_skip, which the detector reads and the host cannot read back */
      struct H2rgWindow {
        int columns{0};
        int rows_skip{0};
      };

      /** What apply_interleave() overwrites, kept so that leaving the readout can put it back */
      struct SavedReadout {
        std::string camera_mode;
        int pixelcount{0};
        int linecount{0};
        std::optional<H2rgWindow> h2rg_window;
        std::array<uint32_t, 4> region_of_interest{};
        std::array<uint32_t, 2> detector_pixels{};
      };

      void record_h2rg_window(int columns, int rows_skip);
      void discard_pre_interleave_readout();
      std::optional<H2rgWindow> current_h2rg_window() const;
      long restore_pre_interleave_readout(std::string &retstring);

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

      // The same write, kept clear of frames arriving on the primary connection
      long send_inreg_offstream(int module, int inreg, int value);

      // Window mode state
      bool is_window{false};
      int win_vstart{0};
      int win_vstop{2047};
      int win_hstart{0};
      int win_hstop{2047};
      int taplines_store{0};
      std::string tapline0_store;
      std::string cur_exposure_mode;

      InterleaveGeometry interleave_geometry;
      // Guards interleave_geometry, and is held across the register writes of a
      // guide window move so that two moves cannot mix on the wire
      std::mutex interleave_mutex;
      // Held only to copy the guide window's bounds, so a reader never waits on
      // the register writes that interleave_mutex is held across
      mutable std::mutex guide_position_mutex;
      std::atomic<uint64_t> guide_changes{0};   //!< see GuidePosition::changes
      std::atomic<int> exposes_in_progress{0};   //!< expose() calls that have not yet returned
      // Both guarded by interleave_mutex. h2rg_window is empty until a command
      // writes it; pre_interleave_readout is held from the first interleaved
      // exposure until the readout is switched away or a new geometry is set.
      std::optional<H2rgWindow> h2rg_window;
      std::optional<SavedReadout> pre_interleave_readout;
      InterleaveOutput interleave_output{InterleaveOutput::Raw};
      bool is_guide_fits_enabled{false};

      // Set in configure_instrument()
      int lvds_module{0};
      int h2rg_max_pixel{0};
      std::string refpix_amp{DEFAULT_REFPIX_AMP};

      // Readout deadline model, from the .cfg since it varies per system
      double pixel_time_usec{0.0};
      double readout_margin_msec{0.0};

      ThreadPlacement acquisition_thread_placement;
      ThreadPlacement processing_thread_placement;

      bool is_freerunning{false};
      std::atomic<bool> is_freerun_active{false};  //!< true while the background freerun loop is running
      bool is_debug{false};
      bool take_stats{false};
  };

}
