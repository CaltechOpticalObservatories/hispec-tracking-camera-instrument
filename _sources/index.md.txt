# HISPEC tracking camera

The `camerad` instrument module for the HISPEC acquisition and tracking camera (ATC): a 2048x2048
H2RG read out by an STA Archon controller.

This repository owns the instrument and pulls
[camera-interface](https://github.com/CaltechOpticalObservatories/camera-interface) in with
`FetchContent`. Everything shared by every instrument (the server, the base commands, the core
`.cfg` keys, frame outputs, the Python bindings) is documented in the
{core}`camera-interface docs <>`. These pages cover only what the tracking camera adds.

## Building

```bash
cmake -S . -B build
cmake --build build
```

That produces `build/camerad`. For the Python module, install the package, which builds it:

```bash
pip install .
```

```python
import camera_interface_tracking as camera
assert camera.instrument_name() == "hispec_tracking_camera"
```

The package is named `camera_interface_tracking` so it can share an environment with other
instruments' builds. Any CMake option passes through pip, for example
`--config-settings=cmake.define.CMAKE_BUILD_TYPE=Debug`.

`ENABLE_SHM_OUTPUT` is on by default, because the tracking camera publishes frames to shared memory.
It needs [ImageStreamIO](https://github.com/milk-org/ImageStreamIO), and the build stops if CMake
cannot find it. An install under `/usr/local` is found with no flags; for another prefix, pass
`--config-settings=cmake.define.ImageStreamIO_DIR=<prefix>/lib/cmake`.

```{toctree}
:hidden:

modes
commands
configuration
fits
```
