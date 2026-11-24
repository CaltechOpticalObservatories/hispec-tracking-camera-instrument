# Commands

The tracking camera adds these to the {core}`base command set <commands/>`. They are reached through
the normal command interface, and from Python via `instrument_cmd()`.

| Command | Arguments | Purpose |
|---|---|---|
| `mode` | `[<name>]` | Select or report the camera mode, see [modes](modes.md) |
| `exposure` | `[utr_rr\|utr_gr\|rx\|rxr\|interleave]` | Select or report the readout mode |
| `autofetch_mode` | `[true\|false]` | Have the controller stream frames as they are read |
| `freerun` | `1\|0` | Arm or disarm continuous exposure |
| `roi` | see below | Set or report the region of interest |
| `window_mode` | `[true\|false]` | Enter or leave windowed (guiding) readout |
| `interleave_science` | `[<height> <width>]` | Set or report the interleaved readout's science window, see below |
| `interleave_guide` | `[<vstart> <vstop> <hstart> <hstop>]` | Set or report the interleaved readout's guide window |
| `interleave_set` | `[<rows>]` | Set or report the science rows read before each guide window |
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

## Interleaved readout

The `interleave` readout mode reads a science window a few rows at a time and a whole guide window
after each group of rows. Three commands describe it. Each reports its current value with no
argument and prints its usage with `?` or `help`, and they can be given in any order.

| Command | Meaning |
|---|---|
| `interleave_science <height> <width>` | The science window, centred on the detector like `roi <height> <width>`, at most 2048 by 1024 |
| `interleave_guide <vstart> <vstop> <hstart> <hstop>` | The guide window, with the same inclusive bounds as the explicit `roi` form |
| `interleave_set <rows>` | Science rows read before each guide window, `1` to `2048` |

They start as a 128 by 128 science window, `interleave_set 4` and a 10 by 10 guide window at the
centre of the detector (`1019 1028 1019 1028`), so `exposure interleave` followed by `expose` works
without setting any of them.

`<width>` is split across two amplifiers, so it must be even. The reference channel adds a third
block of `<width>/2` columns to every line, so a 128 by 128 window is stored 192 columns wide.

`interleave_guide` writes the detector's window bound registers straight away, the same ones the
explicit `roi` form writes. Unlike `roi` it does not put the detector in window mode: the firmware
switches into window mode for each guide read and back out for the science rows. The guide window
may sit anywhere on the detector, inside the science window or not.

It needs the controller connected, firmware loaded and the detector powered, and is refused
otherwise. A power cycle clears the detector's registers, so every `expose` in the `interleave`
readout mode writes the window again.

It can be sent while an exposure is running, to move the window as the guide star drifts, though
not to resize it:

- The register writes keep clear of the frame data. They wait out a frame fetch on the shared
  connection, and go over the controller's control connection while autofetch is armed.
- Only one move runs at a time.
- A move that fails part way writes the previous window back, and the log says whether that
  worked. Until a move succeeds, `interleave_guide` keeps reporting the previous window.

A guide frame being read out while the window moves can hold rows from both positions.

`interleave_science` and `interleave_set` only record their values, which are sent to the
controller at the next `expose`. Neither can be changed while an interleaved exposure or freerun
session is running.

### At expose

In the `interleave` readout mode, `expose` first checks the three settings against each other and
refuses to start if:

- the science height is not a whole multiple of `interleave_set`
- the guide window is wider than the `<width>/2` columns each amplifier reads per line. A narrower
  one is fine: the sequencer clocks every guide line out to the full width
- no camera mode is selected, since the frame is laid out across its taplines

It then clears any abort left set in the controller, sends the settings and sets the frame
geometry. One Archon frame holds one set:
`interleave_set` science lines followed by the guide window's lines, each `<width>/2` pixels per
tap. With a 128 by 128 science window, `interleave_set 4` and a 10 by 10 guide window, a frame is
192 by 14 and a pass over the science window is 32 frames.

`expose <n>` runs `n` passes, and with `freerun 1` passes repeat until `abort`. Each pass is one
trigger. Before every one the sequencer is pointed back at the start of the science window, and an
`abort` stops the passes that have not started. The sequencer finishes a pass one `exptime` delay
after its last frame, so the next pass is not armed, and `expose` does not return, until that delay
has run out. What is published from the frames of a pass is set
by `INTERLEAVE_OUTPUT`, see [configuration](configuration.md#interleaved-output). This is on the
polled pipeline; the autofetch pipeline does not yet run passes.

The sets of a pass follow each other as fast as the detector reads them unless `exptime` is set,
which adds that delay after every set. With no delay the polled pipeline cannot fetch each frame
before the controller overwrites it, and `expose` reports how many were dropped.

None of these commands changes the window `roi` reports.

### Leaving the interleaved readout

Selecting another readout with `exposure` puts back the frame geometry from before the first
interleaved exposure: `H2RG_columns`, `H2RG_rows_skip`, `PIXELCOUNT`, `LINECOUNT` and the region of
interest. The next `rx` frame is then the same shape as one taken before the interleaved readout.
It is refused with `exposure in progress` while an interleaved exposure or freerun session runs.

A `roi`, `mode`, `fullframe` or `window_mode` given in the meantime sets its own geometry, and that
is what the next readout keeps.

## Error reporting

A failing command returns a short reason. The log gets the root cause and a one-line snapshot of
the camera state: connection, power, firmware, camera and readout mode, geometry, window, exposure
time, last frame and readout deadline. The snapshot is taken when the error is logged, so it is not
confused by log lines a concurrent command interleaved.
