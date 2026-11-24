# Configuration

The shipped configuration is {source}`config/hispecatc.cfg`, installed to
`share/hispec_tracking_camera`. Its `DEFAULT_FIRMWARE`, `IMDIR`, `FITS_OUTPUT_DIR` and `LOGPATH` are
paths on the development machine; set them for yours.

Alongside the {core}`core keys <configuration/>`, this module reads its own:

| Key | Default | Meaning |
|---|---|---|
| `REFPIX_AMP` | `AM52` | Amplifier reading the reference channel. Its tapline moves with the camera mode, so `REFPXAMP` is looked up per mode. |
| `PIXEL_TIME_USEC` | `7.0` | Pixel time, used only when the ACF has neither `tpix` nor `SHD2` |
| `READOUT_MARGIN_MSEC` | `5000` | Slack added to the per-frame readout deadline |
| `ACQUISITION_THREAD` | unset | `<cpu\|any>[,<priority>]` for the Archon reader thread |
| `PROCESSING_THREAD` | unset | `<cpu\|any>[,<priority>]` for the thread feeding SHM and FITS |
| `INTERLEAVE_OUTPUT` | `raw` | What the `interleave` readout mode publishes: `raw`, `split` or `guide`, see below |
| `INTERLEAVE_GUIDE_FITS` | `no` | Whether guide windows are also written to FITS, `yes` or `no` |

## Readout deadline

Each frame must arrive within exposure time + pixels per tap x pixel time + `READOUT_MARGIN_MSEC`,
recomputed per `expose` so a full frame and a small ROI share one setting. A frame that misses it
is reported lost. The pixel time comes from the ACF, in 10 ns ticks: the `tpix` parameter of the
selected mode, else the global `tpix`, else the CDS sample end `SHD2`, which falls short of the full
pixel. Without any of them it is `PIXEL_TIME_USEC`. `PIXEL_TIME_USEC` must be positive and `READOUT_MARGIN_MSEC` non-negative; a
non-numeric value fails startup.

## Interleaved output

In the `interleave` readout mode every Archon frame holds one set: a few science lines and a whole
guide window. `INTERLEAVE_OUTPUT` chooses what is published from it. Any other value fails startup.

| Value | Published |
|---|---|
| `raw` | Every frame as read, science and guide lines together |
| `split` | Each guide window as it arrives, plus one science image per pass, assembled from the science lines of every set |
| `guide` | Each guide window and nothing else |

Guide windows go out on a stream of their own, so they get a shared-memory segment named
`<SHM_SEGMENT_NAME>_guide` and FITS files ending in `_guide`. They are the full width of the frame,
every tap included. The science image goes out on the usual stream. A set whose frame the controller
overwrote before it was fetched leaves its rows of the science image at zero.

Both keep the line order the controller stores, which the ACF's `FRAMEMODE` sets, so the science
image comes out the way a frame of the same rows read in one go would. With `FRAMEMODE=1`, as the
ACF has it, a frame is stored last line first: the guide window is the first lines of each frame,
the science lines are the last, and the first set read ends up at the bottom of the science image.

A guide window arrives with every set, far more often than a science image, so guide windows stay
out of FITS unless `INTERLEAVE_GUIDE_FITS=yes`. Shared memory gets them either way. The setting has
no effect in `raw`.

The other readout modes ignore both keys.

## Thread placement

A priority runs the thread `SCHED_FIFO`, which needs `CAP_SYS_NICE`
(`sudo setcap cap_sys_nice+ep bin/camerad`) or a non-zero `RLIMIT_RTPRIO`. Out-of-range values fail
startup. If the kernel refuses at runtime, the refusal is logged and the thread runs unplaced.

## Fixed values

Set in `configure_instrument()` rather than configured: the LVDS module is 10 and the detector's
maximum pixel index is 2047. The Archon socket is tuned there for streaming, with `TCP_NODELAY` and
1 MB buffers.

`WRITE_TAPINFO_TO_FITS` appears in the shipped `hispecatc.cfg` but nothing reads it.
