#pragma once

// =============================================================================
// tcxOrbbec.h - Orbbec backend for tcxDepthCamera
// =============================================================================
//
// Drives an Orbbec depth camera (Femto Bolt = ToF, Gemini 355L = stereo, Astra,
// ...) through the unified tcxDepthCamera interface, built on the Orbbec SDK v2
// (cross-platform: Windows / Linux / macOS).
//
// It fills the canonical DepthFrame: depth (uint16 + value scale), native
// full-resolution color, IR, plus the depth/color intrinsics and the depth->
// color extrinsic, so the base can deproject to world coordinates and project
// color on demand. World coordinates use the base's intrinsics deprojection
// (Brown-Conrady aware); an SDK point-cloud path is a planned follow-up.
//
//   #include <tcxOrbbec.h>
//   using namespace tcx;
//   shared_ptr<DepthCamera> cam = make_shared<Orbbec>();   // first device
//   cam->setThreaded(true);
//   cam->enableDepth(); cam->enableColor();
//   cam->setup();
//   ...
//   cam->update();
//   if (cam->isFrameNew()) cam->toMesh({.colors = true}).draw();
//
// Orbbec SDK (ob::) handles are hidden behind a pImpl so this header stays
// SDK-free.
//
// =============================================================================

#include <tcxDepthCamera.h>
#include <cstdint>
#include <memory>
#include <string>

namespace tcx {

using namespace tc;

class Orbbec : public DepthCamera {
public:
    // deviceIndex selects among connected Orbbec devices (0 = first).
    explicit Orbbec(uint32_t deviceIndex = 0);
    ~Orbbec() override;

    // Detected from the device when known (Femto -> ToF, Gemini -> Stereo),
    // else Unknown. Informational only.
    DepthSensorType getSensorType() const override { return sensorType_; }

    // Device name reported by the SDK (e.g. "Orbbec Femto Bolt"), empty until setup().
    const std::string& getDeviceName() const { return deviceName_; }

    // Telemetry from the last captured depth frame (Orbbec SDK), for diagnostics
    // / OrbbecViewer-style readouts. Timestamps are microseconds.
    uint64_t getFrameIndex()        const { return frameIndex_; }
    uint64_t getDeviceTimestampUs() const { return deviceTsUs_; }
    uint64_t getGlobalTimestampUs() const { return globalTsUs_; }  // 0 unless supported
    uint64_t getSystemTimestampUs() const { return systemTsUs_; }

protected:
    bool openDevice() override;
    void closeDevice() override;
    StreamFreshness captureInto(DepthFrame& dst) override;

private:
    struct Impl;                    // hides ob:: handles
    std::unique_ptr<Impl> impl_;
    uint32_t deviceIndex_;
    std::string deviceName_;
    DepthSensorType sensorType_ = DepthSensorType::Unknown;

    DepthIntrinsics depthIntrinsics_{};  // cached from the depth profile in openDevice
    DepthIntrinsics colorIntrinsics_{};  // color camera intrinsics (native res)
    Mat4 depthToColor_{};                // depth-cam space -> color-cam space
    bool haveColorCalib_ = false;        // colorIntrinsics_/depthToColor_ valid

    // Last depth-frame telemetry (set in captureInto).
    uint64_t frameIndex_ = 0;
    uint64_t deviceTsUs_ = 0;
    uint64_t globalTsUs_ = 0;
    uint64_t systemTsUs_ = 0;
};

} // namespace tcx
