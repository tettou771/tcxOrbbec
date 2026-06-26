// =============================================================================
// tcxOrbbec.cpp - Orbbec backend implementation (Orbbec SDK v2)
// =============================================================================
//
// SCAFFOLD - NOT YET HARDWARE-VERIFIED. Written against the Orbbec SDK v2 C++
// API (libobsensor / namespace ob) ahead of the hardware arriving. The exact
// method names and color-format handling still need to be checked against the
// installed SDK headers and a real Femto Bolt / Gemini 355L. See TODO.md.
//
// Fills the canonical DepthFrame in captureInto(): depth (uint16 * valueScale),
// native full-resolution color, IR, depth/color intrinsics + depth->color
// extrinsic. World coordinates are left to the base's intrinsics deprojection
// (an SDK point-cloud path is a follow-up).
//
// =============================================================================

#include "tcxOrbbec.h"

#include <libobsensor/ObSensor.hpp>
#include <libobsensor/hpp/Utils.hpp>   // CoordinateTransformHelper (XY tables)

#include <cstring>

using namespace std;

namespace tcx {

// waitForFrames timeout before the worker re-checks running (ms).
static constexpr int kFrameTimeoutMs = 1000;

struct Orbbec::Impl {
    shared_ptr<ob::Context>  context;
    shared_ptr<ob::Device>   device;
    shared_ptr<ob::Pipeline> pipeline;
    shared_ptr<ob::Config>   config;

    // Active stream profiles (kept for intrinsics / extrinsics).
    shared_ptr<ob::VideoStreamProfile> depthProfile;
    shared_ptr<ob::VideoStreamProfile> colorProfile;

    // Converts encoded color (MJPG/YUYV/NV12/...) to RGB. Reused per frame.
    ob::FormatConvertFilter colorConv;

    // SDK lens-model deprojection (XY tables). xTable/yTable alias xyTableData,
    // so that buffer must outlive them — keep both here.
    std::vector<float>   xyTableData;
    OBXYTables           xyTables{};
    bool                 xyReady = false;
    std::vector<uint8_t> pointBuf;   // OBPoint output (w*h * sizeof(OBPoint))

    bool warnedColorFormat = false;
};

Orbbec::Orbbec(uint32_t deviceIndex)
    : impl_(make_unique<Impl>()), deviceIndex_(deviceIndex) {}

Orbbec::~Orbbec() = default;

// -----------------------------------------------------------------------------
// Copy an OBCameraIntrinsic (+ optional distortion) into our DepthIntrinsics.
static void copyIntrinsics(const OBCameraIntrinsic& in, DepthIntrinsics& out) {
    out.width  = in.width;
    out.height = in.height;
    out.fx = in.fx; out.fy = in.fy;
    out.cx = in.cx; out.cy = in.cy;
}
static void copyDistortion(const OBCameraDistortion& d, DepthIntrinsics& out) {
    // The base undistort uses the Brown-Conrady k1..k3 / p1,p2 terms; the
    // rational k4..k6 are dropped (fine for the typically low depth distortion).
    out.k1 = d.k1; out.k2 = d.k2; out.k3 = d.k3;
    out.p1 = d.p1; out.p2 = d.p2;
}

// -----------------------------------------------------------------------------
bool Orbbec::openDevice() {
    try {
        impl_->context = make_shared<ob::Context>();
        auto devList = impl_->context->queryDeviceList();
        if (!devList || devList->getCount() == 0) {
            logError("tcxOrbbec") << "no Orbbec device found";
            return false;
        }
        if (deviceIndex_ >= static_cast<uint32_t>(devList->getCount())) {
            logError("tcxOrbbec") << "device index " << deviceIndex_
                                  << " out of range (" << devList->getCount() << " device(s))";
            return false;
        }
        impl_->device = devList->getDevice(deviceIndex_);

        // Device name -> a friendly sensor-type guess (informational only).
        auto info = impl_->device->getDeviceInfo();
        deviceName_ = info ? info->getName() : "";
        if (deviceName_.find("Femto") != string::npos) sensorType_ = DepthSensorType::ToF;
        else if (deviceName_.find("Gemini") != string::npos) sensorType_ = DepthSensorType::Stereo;
        else if (deviceName_.find("Astra") != string::npos) sensorType_ = DepthSensorType::StructuredLight;

        impl_->pipeline = make_shared<ob::Pipeline>(impl_->device);
        impl_->config   = make_shared<ob::Config>();

        // Enable only the streams the caller asked for (all off by default).
        if (isDepthEnabled()) {
            auto list = impl_->pipeline->getStreamProfileList(OB_SENSOR_DEPTH);
            auto prof = list->getProfile(OB_PROFILE_DEFAULT);
            impl_->depthProfile = prof->as<ob::VideoStreamProfile>();
            impl_->config->enableStream(impl_->depthProfile);
        } else {
            logWarning("tcxOrbbec") << "depth stream not enabled; call enableDepth() before setup()";
        }
        if (isColorEnabled()) {
            auto list = impl_->pipeline->getStreamProfileList(OB_SENSOR_COLOR);
            auto prof = list->getProfile(OB_PROFILE_DEFAULT);
            impl_->colorProfile = prof->as<ob::VideoStreamProfile>();
            impl_->config->enableStream(impl_->colorProfile);
        }
        if (isInfraredEnabled()) {
            auto list = impl_->pipeline->getStreamProfileList(OB_SENSOR_IR);
            impl_->config->enableStream(list->getProfile(OB_PROFILE_DEFAULT));
        }

        impl_->pipeline->start(impl_->config);

        // Cache depth intrinsics + distortion.
        if (impl_->depthProfile) {
            copyIntrinsics(impl_->depthProfile->getIntrinsic(), depthIntrinsics_);
            copyDistortion(impl_->depthProfile->getDistortion(), depthIntrinsics_);
        }
        // Cache color intrinsics + depth->color extrinsic.
        if (impl_->depthProfile && impl_->colorProfile) {
            copyIntrinsics(impl_->colorProfile->getIntrinsic(), colorIntrinsics_);
            copyDistortion(impl_->colorProfile->getDistortion(), colorIntrinsics_);
            const OBExtrinsic ex = impl_->depthProfile->getExtrinsicTo(impl_->colorProfile);
            Mat4 m;  // row-major affine, identity by default
            m.m[0]  = ex.rot[0]; m.m[1]  = ex.rot[1]; m.m[2]  = ex.rot[2];
            m.m[4]  = ex.rot[3]; m.m[5]  = ex.rot[4]; m.m[6]  = ex.rot[5];
            m.m[8]  = ex.rot[6]; m.m[9]  = ex.rot[7]; m.m[10] = ex.rot[8];
            m.m[3]  = ex.trans[0] * 0.001f;   // Orbbec extrinsic translation is mm
            m.m[7]  = ex.trans[1] * 0.001f;
            m.m[11] = ex.trans[2] * 0.001f;
            depthToColor_ = m;
            haveColorCalib_ = true;
        }
        logNotice("tcxOrbbec") << "opened " << (deviceName_.empty() ? "Orbbec device" : deviceName_);
        return true;
    } catch (const ob::Error& e) {
        logError("tcxOrbbec") << "openDevice failed: " << e.getMessage();
        return false;
    }
}

void Orbbec::closeDevice() {
    try {
        if (impl_->pipeline) impl_->pipeline->stop();
    } catch (const ob::Error& e) {
        logWarning("tcxOrbbec") << "stop failed: " << e.getMessage();
    }
    impl_->pipeline.reset();
    impl_->config.reset();
    impl_->device.reset();
    impl_->context.reset();
    impl_->depthProfile.reset();
    impl_->colorProfile.reset();
}

// -----------------------------------------------------------------------------
StreamFreshness Orbbec::captureInto(DepthFrame& dst) {
    StreamFreshness fresh;
    if (!impl_->pipeline) return fresh;

    shared_ptr<ob::FrameSet> fs;
    try {
        fs = impl_->pipeline->waitForFrames(kFrameTimeoutMs);
    } catch (const ob::Error&) {
        return fresh;
    }
    if (!fs) return fresh;

    // --- Depth (uint16; meters = value * valueScale * 0.001) ----------------
    if (auto depth = fs->depthFrame()) {
        const int w = depth->getWidth();
        const int h = depth->getHeight();
        dst.w = w;
        dst.h = h;
        // getValueScale() is mm per unit; depthScale is meters per unit.
        dst.depthScale = depth->getValueScale() * 0.001f;
        dst.intrinsics = depthIntrinsics_;
        dst.timestamp  = depth->getTimeStampUs() * 1e-6;

        const uint16_t* db = reinterpret_cast<const uint16_t*>(depth->getData());
        dst.depth.assign(db, db + static_cast<size_t>(w) * h);

        // SDK-exact world cloud via XY tables (TODO #3): models the full lens
        // (incl. distortion), so the cloud matches OrbbecViewer — pinhole stretches
        // the edges on a wide-FOV ToF camera. Fills DepthFrame.world[], which the
        // base prefers over its own intrinsics deprojection. The tables are rebuilt
        // only when the depth resolution changes; xTable/yTable alias xyTableData.
        if (!impl_->xyReady ||
            impl_->xyTables.width != w || impl_->xyTables.height != h) {
            try {
                OBCalibrationParam calib = impl_->pipeline->getCalibrationParam(impl_->config);
                impl_->xyTableData.assign(static_cast<size_t>(w) * h * 2, 0.0f);
                uint32_t bytes = static_cast<uint32_t>(impl_->xyTableData.size() * sizeof(float));
                impl_->xyReady = ob::CoordinateTransformHelper::transformationInitXYTables(
                    calib, OB_SENSOR_DEPTH, impl_->xyTableData.data(), &bytes, &impl_->xyTables);
            } catch (const ob::Error&) {
                impl_->xyReady = false;
            }
        }
        if (impl_->xyReady) {
            const size_t n = static_cast<size_t>(w) * h;
            impl_->pointBuf.resize(n * sizeof(OBPoint));
            ob::CoordinateTransformHelper::transformationDepthToPointCloud(
                &impl_->xyTables, depth->getData(), impl_->pointBuf.data());
            const OBPoint* pts = reinterpret_cast<const OBPoint*>(impl_->pointBuf.data());
            dst.world.resize(n);
            for (size_t i = 0; i < n; ++i)         // OBPoint mm -> meters
                dst.world[i] = Vec3(pts[i].x * 0.001f, pts[i].y * 0.001f, pts[i].z * 0.001f);
        } else {
            dst.world.clear();   // fall back to the base's pinhole deprojection
        }
        fresh.depth = true;
    }

    // --- Color (native full resolution; common formats only for now) --------
    if (auto color = fs->colorFrame()) {
        const int cw = color->getWidth();
        const int ch = color->getHeight();
        const OBFormat fmt = color->getFormat();
        const uint8_t* cb = reinterpret_cast<const uint8_t*>(color->getData());
        const size_t n = static_cast<size_t>(cw) * ch;

        bool ok = true;
        dst.color.allocate(cw, ch, 4);
        uint8_t* out = dst.color.getData();
        if (fmt == OB_FORMAT_RGB) {
            for (size_t i = 0; i < n; ++i) {
                out[i*4+0] = cb[i*3+0]; out[i*4+1] = cb[i*3+1];
                out[i*4+2] = cb[i*3+2]; out[i*4+3] = 255;
            }
        } else if (fmt == OB_FORMAT_BGR) {
            for (size_t i = 0; i < n; ++i) {
                out[i*4+0] = cb[i*3+2]; out[i*4+1] = cb[i*3+1];
                out[i*4+2] = cb[i*3+0]; out[i*4+3] = 255;
            }
        } else if (fmt == OB_FORMAT_RGBA) {
            memcpy(out, cb, n * 4);
        } else if (fmt == OB_FORMAT_BGRA) {
            for (size_t i = 0; i < n; ++i) {
                out[i*4+0] = cb[i*4+2]; out[i*4+1] = cb[i*4+1];
                out[i*4+2] = cb[i*4+0]; out[i*4+3] = cb[i*4+3];
            }
        } else {
            // Encoded formats (MJPG / YUYV / NV12 / Y8/Y16 / ...) -> RGB via the
            // SDK's FormatConvertFilter, then packed to RGBA. Femto Bolt / Gemini
            // color usually arrives as MJPG or YUYV, so this is the common path.
            OBConvertFormat ct = FORMAT_MJPG_TO_RGB;
            bool convertible = true;
            switch (fmt) {
                case OB_FORMAT_MJPG: ct = FORMAT_MJPG_TO_RGB; break;
                case OB_FORMAT_YUYV: ct = FORMAT_YUYV_TO_RGB; break;
                case OB_FORMAT_UYVY: ct = FORMAT_UYVY_TO_RGB; break;
                case OB_FORMAT_NV12: ct = FORMAT_NV12_TO_RGB; break;
                case OB_FORMAT_NV21: ct = FORMAT_NV21_TO_RGB; break;
                case OB_FORMAT_Y16:  ct = FORMAT_Y16_TO_RGB;  break;
                case OB_FORMAT_Y8:   ct = FORMAT_Y8_TO_RGB;   break;
                default: convertible = false; break;
            }
            shared_ptr<ob::VideoFrame> rgb;
            if (convertible) {
                impl_->colorConv.setFormatConvertType(ct);
                if (auto o = impl_->colorConv.process(color)) rgb = o->as<ob::VideoFrame>();
            }
            if (rgb) {
                const uint8_t* rb = reinterpret_cast<const uint8_t*>(rgb->getData());
                for (size_t i = 0; i < n; ++i) {
                    out[i*4+0] = rb[i*3+0]; out[i*4+1] = rb[i*3+1];
                    out[i*4+2] = rb[i*3+2]; out[i*4+3] = 255;
                }
            } else {
                ok = false;
                if (!impl_->warnedColorFormat) {
                    impl_->warnedColorFormat = true;
                    logWarning("tcxOrbbec")
                        << "unsupported color format " << static_cast<int>(fmt)
                        << "; color skipped.";
                }
                dst.color = Pixels{};
            }
        }
        if (ok) {
            if (haveColorCalib_) {
                dst.colorIntrinsics = colorIntrinsics_;
                dst.depthToColor    = depthToColor_;
            }
            fresh.color = true;
        }
    }

    // --- IR (active brightness -> 1-channel F32) -----------------------------
    if (auto ir = fs->irFrame()) {
        const int w = ir->getWidth();
        const int h = ir->getHeight();
        dst.ir.allocate(w, h, 1, PixelFormat::F32);
        float* irOut = dst.ir.getDataF32();
        const size_t n = static_cast<size_t>(w) * h;
        // IR is typically uint16; if a device delivers uint8 this needs adjusting.
        const uint16_t* ib = reinterpret_cast<const uint16_t*>(ir->getData());
        for (size_t i = 0; i < n; ++i) irOut[i] = static_cast<float>(ib[i]);
        fresh.infrared = true;
    }

    return fresh;
}

} // namespace tcx
