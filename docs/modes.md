# Modes

Three unrelated settings are all called a mode. They are selected by different commands and do
different things.

Camera mode (`mode`)
: Names a `[MODE_*]` section of the loaded ACF. Selecting one loads that section's geometry
  (`PIXELCOUNT`, `LINECOUNT`), its parameters and its tapline layout (`TAPLINES`, `TAPLINE0..N`)
  into the controller, then sets `H2RG_rows` and `H2RG_columns` so the detector clocks the same
  window the CDS expects. This changes the shape of the data coming back. Firmware must already be
  loaded.

Readout mode (`exposure`)
: Selects the H2RG readout scheme by setting the matching `mode_*` ACF parameter to 1 and the
  others to 0, so the ACF must define all five.

    | Argument | ACF parameter | Scheme |
    |---|---|---|
    | `utr_rr` | `mode_UTR_RR` | Up the ramp, reset-read |
    | `utr_gr` | `mode_UTR_GR` | Up the ramp, guided read |
    | `rx` | `mode_RX` | Reset-execute |
    | `rxr` | `mode_RXR` | Reset-execute-read |
    | `interleave` | `mode_InterleavedGuiding` | Science rows alternating with a guide window, see [commands](commands.md#interleaved-readout) |

  `rxr` reads every pixel twice, so selecting it sets `PIXELCOUNT` to twice `H2RG_columns`, and
  selecting another readout sets it back.

Exposure mode (`exposuremode`)
: The base command, selecting which acquisition pipeline drives an exposure. This module adds
  `DEFAULT`, which picks its autofetch or polled pipeline from the current `autofetch_mode`. See
  {core}`architecture <architecture/>` for what an exposure mode is.

With no argument, `mode` and `exposure` report the current value and change nothing.

## Acquisition

How `expose` behaves depends on two switches, and the combination is recorded in `OPSMODE`.

| `autofetch_mode` | `freerun` | `expose <n>` | `OPSMODE` |
|---|---|---|---|
| off | `0` | Acquires `n` frames from a single Archon trigger, resetting only before the first, so one call is one ramp | `default` |
| on | `0` | The controller pushes `n` frames as they are read | `autofetch` |
| either | `1` | Starts a background loop and returns immediately; `abort` stops it | `freerun` or `freerun+autofetch` |

In the `interleave` readout mode the count is passes, not frames. `expose <n>` runs `n` passes over
the science window, each one a trigger that yields a frame per set, and with `freerun 1` passes
repeat until `abort`. This runs on the polled pipeline only. See
[commands](commands.md#interleaved-readout) for the settings and
[configuration](configuration.md#interleaved-output) for what is published.

`autofetch_mode` re-selects the `DEFAULT` pipeline whenever it changes, so the polled and autofetch
pipelines never need selecting by hand.

While autofetch or a freerun readout is streaming, the main Archon connection carries frames. Writes
this module makes in that state, including `freerun`, `abort` and disabling autofetch, go over the
controller's control connection instead, so their replies are not lost behind frame data.

`abort` during freerun returns once both threads have joined, so freerun can be restarted straight
away.

Every `expose` first clears the controller's `ABORT_PARAM`: the sequencer never consumes an `abort`
sent while it is idle, and it would otherwise cut the next exposure short. A sequence is not
triggered until the previous one's last `exptime` delay has run out, since the sequencer ignores a
trigger written during it.

If the host falls behind and the controller overwrites frames before they are fetched, acquisition
logs a warning, counts them, and continues from the newest frame; the count is in the line logged
when it stops. A counted `expose` still writes every frame that arrived, then returns an error
naming how many were dropped, since the sequence has gaps; each file's `FRAMENO` shows where.
