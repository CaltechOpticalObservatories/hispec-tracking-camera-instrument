# FITS keywords

{source}`fits_header_dictionary.cpp` maps each header property to a keyword, comment, type and
default. It carries defaults for two cameras, ATC and SPEC; this module writes the ATC set. A
property with no value and no default, or `N/A`, is left out of the header.

These are added to every file on top of the {core}`core keywords <fits/>`:

| Keyword | Meaning |
|---|---|
| `OPSMODE` | `default`, `autofetch`, `freerun` or `freerun+autofetch`, see [modes](modes.md#acquisition) |
| `SUBFRAME` | `guiding` in window mode or a `GUIDING` camera mode, otherwise `full_frame` or `roi` by row count |
| `READOUT` | Readout mode: `rx`, `rxr`, `utr_rr` or `utr_gr` |
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
