# HISPEC Tracking Camera Instrument

Camerad 2.0 instrument module for the HISPEC tracking camera using the Archon controller.

This project owns the instrument and pulls camera-interface in as a dependency, so it
builds on its own.

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
