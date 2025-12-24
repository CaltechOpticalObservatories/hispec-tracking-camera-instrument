# HISPEC Tracking Camera Instrument

Camerad 2.0 instrument module for the HISPEC tracking camera using the Archon controller.

## Usage

This repository is intended to be used as a git submodule within [camera-interface](https://github.com/CaltechOpticalObservatories/camera-interface).

```bash
# Initialize this submodule in an existing camera-interface clone
git submodule update --init camerad/Instruments/hispec_tracking_camera
```

## Building

```bash
cd camera-interface/build
cmake -DINSTRUMENT=hispec_tracking_camera -DCONTROLLER=archon ..
make
```
