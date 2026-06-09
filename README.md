# tcxOrbbec

Orbbec depth-camera support for [TrussC](https://github.com/TrussC-org/TrussC),
implementing the [`tcxDepthCamera`](https://github.com/TrussC-org/TrussC/tree/dev/addons/tcxDepthCamera)
interface. Drive an Orbbec camera (Femto Bolt, Gemini 335L, Astra, …) through the
same `DepthCamera` API as any other depth sensor.

> **Status: runs on real hardware** — `example-basic` renders a live point cloud
> from a **Gemini 335L** (macOS needs `sudo`, see below). Color-format coverage,
> the SDK point-cloud path, and Linux/Windows runs are still being validated; see
> `TODO.md`.

## Requirements

- **Orbbec SDK v2 (libobsensor)** installed. Unlike k4a, it is **cross-platform:
  Windows, Linux, and macOS** (arm64/x86_64) — so this addon can build and run on
  a Mac with a connected device.
  - Download from [OrbbecSDK_v2 releases](https://github.com/orbbec/OrbbecSDK_v2/releases).
  - Point CMake at its package config:
    `-DOrbbecSDK_DIR="<sdk>/lib/cmake/OrbbecSDK"` (defines `ob::OrbbecSDK`).
  - The SDK runtime shared library is bundled next to the app binary at build
    time; its `extensions/` folder currently needs to be copied by hand (see
    `TODO.md`).
- The `tcxDepthCamera` addon (this addon depends on it).

## Running on macOS — needs `sudo`

```bash
sudo ./bin/example-basic.app/Contents/MacOS/example-basic
```

Otherwise you get `uvc_open ... -3 (access)`: macOS's `UVCAssistant` grabs UVC
cameras exclusively and only root can override it ([details](https://github.com/orbbec/OrbbecSDK_v2/issues/124)).
It's a macOS-wide limitation, not this addon (Orbbec's own samples fail the same).
Linux and Windows don't need `sudo`.

## Usage

```cpp
#include <tcxOrbbec.h>
using namespace tcx;

shared_ptr<DepthCamera> cam = make_shared<Orbbec>();  // first device
cam->setThreaded(true);     // grab on a background thread (call before setup)
cam->enableDepth();
cam->enableColor();
cam->setup();
// ...
cam->update();
if (cam->isFrameNew()) {
    Mesh cloud = cam->toMesh({.colors = true});
    cloud.draw();
}
```

Color and IR are part of the canonical `DepthFrame`, read directly on the camera:

```cpp
if (cam->hasColor())    cam->getColorImage().draw(0, 0);     // cached preview Image
if (cam->hasInfrared()) cam->getInfraredImage().draw(0, 0);
const DepthFrame& f = cam->currentFrame();   // depth / color / ir / intrinsics
```

## Notes

- This backend just **fills the canonical `DepthFrame`** in `captureInto()`; the
  `tcxDepthCamera` base provides all the accessors, meshing and threading.
- **Units:** depth distance and world coordinates are in **meters**. Depth is
  stored as uint16; `depthScale` = `depthFrame->getValueScale() * 0.001` (the SDK
  value scale is mm per unit).
- **Color** is kept at its NATIVE full resolution — NOT registered/downsampled to
  depth. The base computes the depth→color mapping on demand from the color
  intrinsics + depth→color extrinsic (cached from the SDK stream profiles).
- **World coordinates** use the base's intrinsics deprojection (Brown-Conrady
  aware, from the depth intrinsics + distortion). Using the SDK's point-cloud
  filter for an exact transform is a planned follow-up.
- **Stream enables are honored:** `enableDepth()/enableColor()/enableInfrared()`
  (all off by default) decide which streams the pipeline starts.
- **Sensor type** is guessed from the device name (Femto → ToF, Gemini → Stereo,
  Astra → StructuredLight), informational only.

## License

MIT. See `LICENSES.md`. Depends on the Orbbec SDK v2 (Apache-2.0, Orbbec) which
must be installed separately.
