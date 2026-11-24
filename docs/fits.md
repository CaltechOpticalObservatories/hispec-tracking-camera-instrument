# FITS keywords

{source}`fits_header_dictionary.cpp` maps each header property to a keyword, comment, type and
default. It carries defaults for two cameras, ATC and SPEC; this module writes the ATC set. A
property with no value and no default, or `N/A`, is left out of the header.

These are added to every file on top of the {core}`core keywords <fits/>`:

| Keyword | Meaning |
|---|---|
| `OPSMODE` | `default`, `autofetch`, `freerun` or `freerun+autofetch`, see [modes](modes.md#acquisition) |
| `SUBFRAME` | `guiding` in window mode or a `GUIDING` camera mode, otherwise `full_frame` or `roi` by row count |
| `READOUT` | Readout mode: `rx`, `rxr`, `utr_rr`, `utr_gr` or `interleave` |
| `FREERUN` | Acquired in freerun |
| `EXPTIME`, `EXPTUNIT` | Exposure time, always in seconds |
| `EXPMJDST`, `ACQTIME` | MJD and UTC timestamp of the frame |
| `NREADS` | Reads in the frame, `1` |
| `DETBITS` | Detector bit depth |
| `PIXTIME` | Pixel time in force, in microseconds, from the ACF's `tpix` (see [configuration](configuration.md#readout-deadline)) |
| `FRAMETME` | Seconds to read out one amplifier region, the same figure the readout deadline uses |
| `NCHANLS` | Detector channels, not counting the reference channel |
| `REFPXAMP` | Channel read by `REFPIX_AMP` in the current camera mode |
| `REFCHPOS`, `CHANVERT` | Reference channel position and channel orientation |
| `SKIPROWS`, `SKIPLNES` | Window start row and column, `0` outside window mode |
| `LVLC_V1`..`LVLC_V3` | Gate, bias and diode substrate voltages from the ACF |
| `FIRMWARE`, `CAMD_VER` | ACF in use and the `camerad` build date |
| `FILETYPE` | `FITS` |

`EXPMJDST`, `ACQTIME`, `EXPTIME` and `NREADS` are set per frame; the rest once per exposure.

## Interleave readout

The `interleave` readout mode adds these. Rows and columns are detector pixels, and passes and
sets count from 0.

Once per exposure:

| Keyword | Meaning |
|---|---|
| `ILVSET` | Science rows read before each guide window |
| `ILVNSETS` | Sets in one pass over the science window |
| `ILVSROW0`, `ILVSROWS` | First row of the science window, and how many rows it has |
| `ILVSPIX` | Science pixels each amplifier reads per row |
| `ILVGROWS`, `ILVGCOLS` | Rows and columns of the guide window |
| `ILVOUT` | `INTERLEAVE_OUTPUT` in force: `raw`, `split` or `guide` |

With every frame that holds one set, so every raw frame and every guide window:

| Keyword | Meaning |
|---|---|
| `ILVPASS` | Pass over the science window the set belongs to |
| `ILVSETN` | Set within that pass |
| `ILVROW0` | First detector row of the set's science rows |
| `ILVGV0`, `ILVGV1` | First and last row of the guide window when the frame was fetched |
| `ILVGH0`, `ILVGH1` | First and last column of the guide window |
| `ILVGMOVE` | The guide window was being moved at some point while the frame was read, so it may hold rows from both positions |

A science image assembled in `split` carries `ILVPASS` and `ILVGOT`, the number of sets gathered
into it. `ILVGOT` below `ILVNSETS` means some rows are missing and read as zero. Its other keys,
such as `FRAMENO`, are those of the last set of the pass.

Guide windows are written to files ending in `_guide`, and with `datacube true` to a cube of their
own, apart from the science images.
