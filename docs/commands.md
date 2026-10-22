# Commands

The tracking camera adds these to the {core}`base command set <commands/>`. They are reached through
the normal command interface, and from Python via `instrument_cmd()`.

| Command | Arguments | Purpose |
|---|---|---|
| `mode` | `[<name>]` | Select or report the camera mode, see [modes](modes.md) |
| `exposure` | `[utr_rr\|utr_gr\|rx\|rxr]` | Select or report the readout mode |
| `autofetch_mode` | `[true\|false]` | Have the controller stream frames as they are read |
| `freerun` | `1\|0` | Arm or disarm continuous exposure |
| `roi` | see below | Set or report the region of interest |
| `window_mode` | `[true\|false]` | Enter or leave windowed (guiding) readout |
| `debug` | `[true\|false]` | Per-frame logging in the acquisition threads |
| `take_stats` | `[true\|false]` | Per-frame timing statistics, logged at the end of each exposure |

`debug` and `take_stats` are off by default because both slow acquisition. They can be toggled while
an exposure or freerun loop is running. Errors are logged either way.

`instrument_commands()` lists the commands of a given build.

## Region of interest

| Form | Meaning |
|---|---|
| `roi` | Report the current window as `vstart vstop hstart hstop` |
| `roi <height> <width>` | A centred region, at most 2048 by 1024 |
| `roi <vstart> <vstop> <hstart> <hstop>` | An explicit window, puts the detector in window mode |
| `roi fullframe` | Return to the full 2048x2048 frame and the `FULLFRAME` camera mode |

Bounds are inclusive and in detector pixels, `0` to `2047`, with `start < stop`.

`window_mode true` saves the current tapline layout, switches to the `GUIDING` camera mode and reads
a single tapline over the current window. `window_mode false` restores the saved taplines, switches
back to `DEFAULT` to reset the controller's buffer geometry, and issues `Abort`. Leaving is
therefore not the inverse of entering, and it needs the ACF to define both modes.

## Error reporting

A failing command returns a short reason. The log gets the root cause and a one-line snapshot of
the camera state: connection, power, firmware, camera and readout mode, geometry, window, exposure
time, last frame and readout deadline. The snapshot is taken when the error is logged, so it is not
confused by log lines a concurrent command interleaved.
