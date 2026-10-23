# HISPEC Tracking Camera Instrument

Camerad 2.0 instrument module for the HISPEC acquisition and tracking camera: a 2048x2048 H2RG on an
Archon controller.

This project owns the instrument and pulls camera-interface in as a dependency, so it
builds on its own.

Full documentation: <https://caltechopticalobservatories.github.io/hispec-tracking-camera-instrument/>.
Shared `camerad` behaviour is in the [camera-interface docs](https://caltechopticalobservatories.github.io/camera-interface/).

## Building

```bash
cmake -S . -B build && cmake --build build
```

That produces `build/camerad`. Add `-DBUILD_PYTHON_MODULE=ON` for the Python module,
which needs `pybind11` in the interpreter CMake finds.

## Python module

```bash
pip install .
```

Installs the `camera_interface_tracking` package, so the build can share an environment
with other instruments:

```python
import camera_interface_tracking as camera
assert camera.instrument_name() == "hispec_tracking_camera"
```

Pass any CMake option through pip, for example
`--config-settings=cmake.define.CMAKE_BUILD_TYPE=Debug`.

### Shared memory

`ENABLE_SHM_OUTPUT` is on by default here, because the tracking camera
publishes frames to shared memory. It needs
[ImageStreamIO](https://github.com/milk-org/ImageStreamIO), and the build stops
if CMake cannot find it rather than producing a module that logs "compiled
without SHM support" only once a camera is running.

CMake looks for the package config under `<prefix>/lib/cmake/ImageStreamIO/`.
An install that leaves it directly in `<prefix>/lib/cmake/` has to be pointed
at explicitly:

```bash
pip install . --config-settings=cmake.define.ImageStreamIO_DIR=/usr/local/lib/cmake
```

## Commands

Added to the base `camerad` command set:

| Command | Purpose |
|---|---|
| `mode [<name>]` | Select or report the ACF camera mode (`[MODE_*]` section) |
| `exposure [utr_rr\|utr_gr\|rx\|rxr]` | Select or report the H2RG readout mode |
| `autofetch_mode [true\|false]` | Have the controller stream frames as they are read |
| `freerun 1\|0` | Arm or disarm continuous exposure; `abort` stops a running loop |
| `roi [...]` | Set or report the region of interest |
| `window_mode [true\|false]` | Enter or leave windowed (guiding) readout |
| `debug [true\|false]` | Per-frame logging in the acquisition threads |
| `take_stats [true\|false]` | Per-frame timing statistics |

The shipped configuration is `config/hispecatc.cfg`; its paths point at the development machine.

## Documentation

```bash
pip install -r docs/requirements.txt
sphinx-build -W docs docs/_build/html
```
