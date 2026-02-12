# SteamVR driver for Mercury hand tracking!

A maintained version of [moshimeow's mercury_steamvr_driver](https://github.com/moshimeow/mercury_steamvr_driver) Mercury SteamVR drivers, enabling hand-tracking in HMDs with Cameras by bringing [Monado's Mercury Pipeline](https://monado.freedesktop.org/handtracking) to SteamVR.
This fork includes patches to improve camera stability, no longer causing it to be unusable due to hanging processes.

## Build Instructions

Prerequisites:
- vcpkg
- CMake
- Ninja
- Visual Studio 2022 or 2026

### Clone the repository
```
git clone https://github.com/moshimeow/mercury_steamvr_driver.git --recursive
cd mercury_steamvr_driver
git submodule init
```

### Download onnx
```
powershell .\attic\moshi_get_onnxruntime.ps1
```

### Set your vcpkg path
Edit the `vcpkgdir` variable in [attic/moshi_build.ps1](attic/moshi_build.ps1) to reflect your vcpkg path.

### Build the project
```
powershell .\attic\moshi_build.ps1
```

Once this has completed, copy `onnxruntime.dll` from the `deps` folder to `build/mercury/bin/win64/`.

