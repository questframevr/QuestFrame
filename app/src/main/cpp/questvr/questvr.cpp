//
//      A/B/X/Y, Menu, haptics.
//
//
//
//

#include <jni.h>
#include <android/log.h>
#include <EGL/egl.h>
#include <GLES3/gl3.h>

#include "steamvr_link.h"

#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#define TAG "QuestVR"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN, TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)

namespace {

constexpr int MODE_HOME = 0;
constexpr int MODE_THEATER = 1;
constexpr int MODE_STEAMVR = 2;

constexpr int BTN_A = 0, BTN_B = 1, BTN_X = 2, BTN_Y = 3, BTN_LB = 4, BTN_RB = 5, BTN_START = 7, BTN_L3 = 8, BTN_R3 = 9;

constexpr int POINTER_NONE = 0, POINTER_HOVER = 1, POINTER_DOWN = 2, POINTER_UP = 3;

// ---------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------
struct Mat4 {
    float m[16];
};

Mat4 identity() {
    Mat4 r{};
    r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1.0f;
    return r;
}

Mat4 mul(const Mat4& a, const Mat4& b) {
    Mat4 r{};
    for (int c = 0; c < 4; c++)
        for (int row = 0; row < 4; row++) {
            float s = 0;
            for (int k = 0; k < 4; k++) s += a.m[k * 4 + row] * b.m[c * 4 + k];
            r.m[c * 4 + row] = s;
        }
    return r;
}

Mat4 translation(float x, float y, float z) {
    Mat4 r = identity();
    r.m[12] = x;
    r.m[13] = y;
    r.m[14] = z;
    return r;
}

Mat4 scale(float x, float y, float z) {
    Mat4 r = identity();
    r.m[0] = x;
    r.m[5] = y;
    r.m[10] = z;
    return r;
}

Mat4 rotationY(float a) {
    Mat4 r = identity();
    r.m[0] = std::cos(a);
    r.m[2] = -std::sin(a);
    r.m[8] = std::sin(a);
    r.m[10] = std::cos(a);
    return r;
}

Mat4 fromPose(const XrPosef& p) {
    const float x = p.orientation.x, y = p.orientation.y, z = p.orientation.z, w = p.orientation.w;
    Mat4 r = identity();
    r.m[0] = 1 - 2 * (y * y + z * z);
    r.m[1] = 2 * (x * y + z * w);
    r.m[2] = 2 * (x * z - y * w);
    r.m[4] = 2 * (x * y - z * w);
    r.m[5] = 1 - 2 * (x * x + z * z);
    r.m[6] = 2 * (y * z + x * w);
    r.m[8] = 2 * (x * z + y * w);
    r.m[9] = 2 * (y * z - x * w);
    r.m[10] = 1 - 2 * (x * x + y * y);
    r.m[12] = p.position.x;
    r.m[13] = p.position.y;
    r.m[14] = p.position.z;
    return r;
}

Mat4 invertRigid(const Mat4& a) {
    Mat4 r = identity();
    for (int c = 0; c < 3; c++)
        for (int row = 0; row < 3; row++) r.m[c * 4 + row] = a.m[row * 4 + c];
    for (int row = 0; row < 3; row++)
        r.m[12 + row] = -(r.m[0 + row] * a.m[12] + r.m[4 + row] * a.m[13] + r.m[8 + row] * a.m[14]);
    return r;
}

Mat4 projectionFov(const XrFovf& fov, float nearZ, float farZ) {
    const float l = std::tan(fov.angleLeft), r = std::tan(fov.angleRight);
    const float u = std::tan(fov.angleUp), d = std::tan(fov.angleDown);
    const float w = r - l, h = u - d;
    Mat4 m{};
    m.m[0] = 2.0f / w;
    m.m[5] = 2.0f / h;
    m.m[8] = (r + l) / w;
    m.m[9] = (u + d) / h;
    m.m[10] = -(farZ + nearZ) / (farZ - nearZ);
    m.m[11] = -1.0f;
    m.m[14] = -(2.0f * farZ * nearZ) / (farZ - nearZ);
    return m;
}

XrVector3f add(XrVector3f a, XrVector3f b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
XrVector3f sub(XrVector3f a, XrVector3f b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
XrVector3f scaled(XrVector3f a, float s) { return {a.x * s, a.y * s, a.z * s}; }

XrVector3f rotate(const XrQuaternionf& q, XrVector3f v) {
    const XrVector3f u{q.x, q.y, q.z};
    const float s = q.w;
    const XrVector3f c1{u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z, u.x * v.y - u.y * v.x};
    const XrVector3f c2{u.y * c1.z - u.z * c1.y, u.z * c1.x - u.x * c1.z, u.x * c1.y - u.y * c1.x};
    return add(v, add(scaled(c1, 2.0f * s), scaled(c2, 2.0f)));
}

XrQuaternionf conjugate(const XrQuaternionf& q) { return {-q.x, -q.y, -q.z, q.w}; }

XrQuaternionf qmul(const XrQuaternionf& a, const XrQuaternionf& b) {
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

XrPosef compose(const XrPosef& a, const XrPosef& b) {
    return {qmul(a.orientation, b.orientation), add(a.position, rotate(a.orientation, b.position))};
}

qdvr_pose toWire(const XrPosef& p) {
    return {p.position.x, p.position.y, p.position.z, p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w};
}
XrPosef fromWire(const qdvr_pose& p) { return {{p.qx, p.qy, p.qz, p.qw}, {p.px, p.py, p.pz}}; }

XrQuaternionf yawQuat(float yaw) { return {0.0f, std::sin(yaw * 0.5f), 0.0f, std::cos(yaw * 0.5f)}; }

float yawOf(const XrQuaternionf& q) {
    const XrVector3f f = rotate(q, {0, 0, -1});
    return std::atan2(-f.x, -f.z);
}

// ---------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------
JavaVM* gVm = nullptr;

struct Callbacks {
    jobject activity = nullptr;
    jmethodID onError = nullptr, onExit = nullptr, onTheaterSurface = nullptr, onPanelSurface = nullptr,
              onSurfacesGone = nullptr, onPanelPointer = nullptr, onPad = nullptr;
    jclass compositor = nullptr;  // global ref
    jmethodID compositorVsync = nullptr;
};

// ---------------------------------------------------------------------------------------------
// The session.
// ---------------------------------------------------------------------------------------------
struct Vertex {
    float x, y, z, r, g, b;
};

struct EyeSwapchain {
    XrSwapchain handle = XR_NULL_HANDLE;
    int32_t width = 0, height = 0;
    std::vector<XrSwapchainImageOpenGLESKHR> images;
    std::vector<GLuint> fbos;
    GLuint depth = 0;
};

struct SurfaceQuad {
    XrSwapchain handle = XR_NULL_HANDLE;
    jobject surface = nullptr;  // global ref
    bool ownsSurfaceRef = false;
    int32_t width = 0, height = 0;
    XrPosef pose{{0, 0, 0, 1}, {0, 0, 0}};
    XrExtent2Df size{1, 1};
    bool visible = false;
};

class VrSession {
public:
    VrSession(int mode, int theaterW, int theaterH) : mode_(mode), theaterW_(theaterW), theaterH_(theaterH) {}

    std::atomic<bool> quit{false};
    std::atomic<bool> finished{false};
    std::atomic<bool> recenterAsked{false};
    std::atomic<float> screenDistance{2.6f};
    std::atomic<float> screenWidth{3.2f};
    Callbacks cb;

    void run() {
        JNIEnv* env = nullptr;
        if (gVm->AttachCurrentThread(&env, nullptr) != JNI_OK) {
            LOGE("could not attach the session thread to the JVM");
            finished = true;
            return;
        }
        env_ = env;
        std::string error;
        if (!init(error)) {
            LOGE("startup failed: %s", error.c_str());
            surfacesGone();
            teardown();
            callString(cb.onError, error);
        } else {
            loop();
            surfacesGone();
            teardown();
            callVoid(cb.onExit);
        }
        if (cb.activity) env_->DeleteGlobalRef(cb.activity);
        if (cb.compositor) env_->DeleteGlobalRef(cb.compositor);
        if (padArray_) env_->DeleteGlobalRef(padArray_);
        cb = Callbacks();
        padArray_ = nullptr;
        env_ = nullptr;
        gVm->DetachCurrentThread();
        finished = true;
    }

private:
    const int mode_;
    const int theaterW_, theaterH_;
    JNIEnv* env_ = nullptr;
    jfloatArray padArray_ = nullptr;

    XrInstance instance_ = XR_NULL_HANDLE;
    XrSystemId systemId_ = XR_NULL_SYSTEM_ID;
    XrSession session_ = XR_NULL_HANDLE;
    XrSpace appSpace_ = XR_NULL_HANDLE;
    bool stageSpace_ = false;
    XrSessionState state_ = XR_SESSION_STATE_UNKNOWN;
    bool sessionRunning_ = false;
    EyeSwapchain eyes_[2];
    XrViewConfigurationView viewConfig_[2]{};
    int64_t colorFormat_ = 0;
    bool surfaceSwapchains_ = false;
    bool flipLayers_ = false;
    XrCompositionLayerImageLayoutFB flipLayout_{XR_TYPE_COMPOSITION_LAYER_IMAGE_LAYOUT_FB, nullptr,
                                                XR_COMPOSITION_LAYER_IMAGE_LAYOUT_VERTICAL_FLIP_BIT_FB};
    PFN_xrCreateSwapchainAndroidSurfaceKHR createSurfaceSwapchain_ = nullptr;
    SurfaceQuad theater_, panel_;
    static constexpr int PANEL_W = 1600, PANEL_H = 200;

    // --- Input --------------------------------------------------------------------------
    XrActionSet actionSet_ = XR_NULL_HANDLE;
    XrAction aimPose_ = XR_NULL_HANDLE, gripPose_ = XR_NULL_HANDLE, trigger_ = XR_NULL_HANDLE,
             squeeze_ = XR_NULL_HANDLE, thumbstick_ = XR_NULL_HANDLE, stickClick_ = XR_NULL_HANDLE,
             haptic_ = XR_NULL_HANDLE, btnA_ = XR_NULL_HANDLE, btnB_ = XR_NULL_HANDLE, btnX_ = XR_NULL_HANDLE,
             btnY_ = XR_NULL_HANDLE, menu_ = XR_NULL_HANDLE;
    XrPath hand_[2]{};
    XrSpace aimSpace_[2]{XR_NULL_HANDLE, XR_NULL_HANDLE};
    XrSpace gripSpace_[2]{XR_NULL_HANDLE, XR_NULL_HANDLE};
    struct Hand {
        bool aimValid = false, gripValid = false;
        XrPosef aim{}, grip{};
        float trigger = 0, squeeze = 0, stickX = 0, stickY = 0;
        bool stickClick = false;
        bool active = false;
        bool squeezeWasDown = false;
        bool onPanel = false, wasOnPanel = false, panelDown = false;
        float panelU = 0, panelV = 0, rayLength = 3.0f;
    } hands_[2];
    bool a_ = false, b_ = false, x_ = false, y_ = false, menu_down_ = false;
    int pointerHand_ = -1;

    EGLDisplay display_ = EGL_NO_DISPLAY;
    EGLContext context_ = EGL_NO_CONTEXT;
    EGLSurface surface_ = EGL_NO_SURFACE;
    EGLConfig config_ = nullptr;
    GLuint program_ = 0, staticVbo_ = 0, dynamicVbo_ = 0;
    GLint uMvp_ = -1;
    int gridVertexCount_ = 0, cubeFirst_ = 0, cubeVertexCount_ = 0;

    SteamVrLink link_;
    bool linkUp_ = false;
    XrSpace viewSpace_ = XR_NULL_HANDLE;
    qdvr_hello hello_{};
    XrFovf helloFov_[2]{};
    XrPosef eyeToHead_[2]{};
    bool haveEyes_ = false;
    uint32_t trackingSeq_ = 0;
    GLuint blitProgram_ = 0, blitVao_ = 0;
    GLint uBlitX0_ = -1, uBlitDecode_ = -1, uBlitTex_ = -1;
    bool showingSteamVr_ = false;
    std::vector<qdvr_haptic> haptics_;

    float spin_ = 0;
    float spinSpeed_ = 0.6f;
    int64_t lastFrameTime_ = 0;
    int framesRendered_ = 0;
    bool anchored_ = false;
    XrVector3f anchorPos_{0, 0, 0};
    float anchorYaw_ = 0;
    std::chrono::steady_clock::time_point lastStatus_ = std::chrono::steady_clock::now();

    void clearException() {
        if (env_->ExceptionCheck()) {
            env_->ExceptionDescribe();
            env_->ExceptionClear();
        }
    }
    void callVoid(jmethodID m) {
        if (!cb.activity || !m) return;
        env_->CallVoidMethod(cb.activity, m);
        clearException();
    }
    void callString(jmethodID m, const std::string& s) {
        if (!cb.activity || !m) return;
        jstring js = env_->NewStringUTF(s.c_str());
        env_->CallVoidMethod(cb.activity, m, js);
        env_->DeleteLocalRef(js);
        clearException();
    }
    void callSurface(jmethodID m, jobject surface, int w, int h) {
        if (!cb.activity || !m) return;
        env_->CallVoidMethod(cb.activity, m, surface, w, h);
        clearException();
    }
    void surfacesGone() {
        if (theater_.surface || panel_.surface) callVoid(cb.onSurfacesGone);
    }

    bool xrOk(XrResult r, const char* what) {
        if (XR_SUCCEEDED(r)) return true;
        char buf[XR_MAX_RESULT_STRING_SIZE] = "?";
        if (instance_ != XR_NULL_HANDLE) xrResultToString(instance_, r, buf);
        LOGE("%s failed: %s (%d)", what, buf, static_cast<int>(r));
        return false;
    }

    XrPath path(const char* s) {
        XrPath p = XR_NULL_PATH;
        xrStringToPath(instance_, s, &p);
        return p;
    }

    // ------------------------------------------------------------------------------------
    bool init(std::string& error) {
        PFN_xrInitializeLoaderKHR initLoader = nullptr;
        if (!xrOk(xrGetInstanceProcAddr(XR_NULL_HANDLE, "xrInitializeLoaderKHR",
                                        reinterpret_cast<PFN_xrVoidFunction*>(&initLoader)), "xrGetInstanceProcAddr(xrInitializeLoaderKHR)") ||
            !initLoader) {
            error = "The OpenXR loader is not available on this device.";
            return false;
        }
        XrLoaderInitInfoAndroidKHR loaderInfo{XR_TYPE_LOADER_INIT_INFO_ANDROID_KHR};
        loaderInfo.applicationVM = gVm;
        loaderInfo.applicationContext = cb.activity;
        if (!xrOk(initLoader(reinterpret_cast<const XrLoaderInitInfoBaseHeaderKHR*>(&loaderInfo)), "xrInitializeLoaderKHR")) {
            error = "The OpenXR loader could not be initialised.";
            return false;
        }

        uint32_t extCount = 0;
        xrEnumerateInstanceExtensionProperties(nullptr, 0, &extCount, nullptr);
        std::vector<XrExtensionProperties> exts(extCount, {XR_TYPE_EXTENSION_PROPERTIES});
        xrEnumerateInstanceExtensionProperties(nullptr, extCount, &extCount, exts.data());
        auto has = [&](const char* name) {
            for (auto& e : exts)
                if (std::strcmp(e.extensionName, name) == 0) return true;
            return false;
        };
        LOGI("runtime offers %u instance extensions", extCount);
        if (!has(XR_KHR_OPENGL_ES_ENABLE_EXTENSION_NAME)) {
            error = "This OpenXR runtime does not support OpenGL ES (XR_KHR_opengl_es_enable).";
            return false;
        }
        std::vector<const char*> enabled{XR_KHR_OPENGL_ES_ENABLE_EXTENSION_NAME};
        const bool androidCreate = has(XR_KHR_ANDROID_CREATE_INSTANCE_EXTENSION_NAME);
        if (androidCreate) enabled.push_back(XR_KHR_ANDROID_CREATE_INSTANCE_EXTENSION_NAME);
        surfaceSwapchains_ = has(XR_KHR_ANDROID_SURFACE_SWAPCHAIN_EXTENSION_NAME);
        if (surfaceSwapchains_) enabled.push_back(XR_KHR_ANDROID_SURFACE_SWAPCHAIN_EXTENSION_NAME);
        else LOGW("no %s: no theater screen and no panel", XR_KHR_ANDROID_SURFACE_SWAPCHAIN_EXTENSION_NAME);
        flipLayers_ = has(XR_FB_COMPOSITION_LAYER_IMAGE_LAYOUT_EXTENSION_NAME);
        if (flipLayers_) enabled.push_back(XR_FB_COMPOSITION_LAYER_IMAGE_LAYOUT_EXTENSION_NAME);
        else LOGW("no %s: Surface layers are flipped by their producers instead", XR_FB_COMPOSITION_LAYER_IMAGE_LAYOUT_EXTENSION_NAME);
        if (mode_ == MODE_THEATER && !surfaceSwapchains_) {
            error = "This headset's OpenXR runtime cannot show an Android Surface, which the VR screen needs.";
            return false;
        }

        XrInstanceCreateInfoAndroidKHR androidInfo{XR_TYPE_INSTANCE_CREATE_INFO_ANDROID_KHR};
        androidInfo.applicationVM = gVm;
        androidInfo.applicationActivity = cb.activity;

        XrInstanceCreateInfo ci{XR_TYPE_INSTANCE_CREATE_INFO};
        std::strncpy(ci.applicationInfo.applicationName, "QuestDeck VR", XR_MAX_APPLICATION_NAME_SIZE - 1);
        std::strncpy(ci.applicationInfo.engineName, "QuestDeck", XR_MAX_ENGINE_NAME_SIZE - 1);
        ci.applicationInfo.applicationVersion = 1;
        ci.applicationInfo.engineVersion = 1;
        ci.enabledExtensionCount = static_cast<uint32_t>(enabled.size());
        ci.enabledExtensionNames = enabled.data();
        if (androidCreate) ci.next = &androidInfo;
        ci.applicationInfo.apiVersion = XR_API_VERSION_1_1;
        XrResult r = xrCreateInstance(&ci, &instance_);
        if (r == XR_ERROR_API_VERSION_UNSUPPORTED) {
            LOGW("runtime refused OpenXR 1.1; asking for 1.0");
            ci.applicationInfo.apiVersion = XR_API_VERSION_1_0;
            r = xrCreateInstance(&ci, &instance_);
        }
        if (!xrOk(r, "xrCreateInstance")) {
            error = (r == XR_ERROR_RUNTIME_UNAVAILABLE)
                        ? "No OpenXR runtime is available. This needs a Meta Quest headset."
                        : "OpenXR instance creation failed (" + std::to_string(static_cast<int>(r)) + ").";
            return false;
        }
        XrInstanceProperties props{XR_TYPE_INSTANCE_PROPERTIES};
        if (xrOk(xrGetInstanceProperties(instance_, &props), "xrGetInstanceProperties")) {
            LOGI("runtime: %s %u.%u.%u (api %u.%u)", props.runtimeName, XR_VERSION_MAJOR(props.runtimeVersion),
                 XR_VERSION_MINOR(props.runtimeVersion), XR_VERSION_PATCH(props.runtimeVersion),
                 XR_VERSION_MAJOR(ci.applicationInfo.apiVersion), XR_VERSION_MINOR(ci.applicationInfo.apiVersion));
        }
        if (surfaceSwapchains_ &&
            !xrOk(xrGetInstanceProcAddr(instance_, "xrCreateSwapchainAndroidSurfaceKHR",
                                        reinterpret_cast<PFN_xrVoidFunction*>(&createSurfaceSwapchain_)), "xrGetInstanceProcAddr(xrCreateSwapchainAndroidSurfaceKHR)")) {
            createSurfaceSwapchain_ = nullptr;
        }

        XrSystemGetInfo sysInfo{XR_TYPE_SYSTEM_GET_INFO};
        sysInfo.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
        if (!xrOk(xrGetSystem(instance_, &sysInfo, &systemId_), "xrGetSystem")) {
            error = "No headset was found by the OpenXR runtime.";
            return false;
        }
        XrSystemProperties sys{XR_TYPE_SYSTEM_PROPERTIES};
        if (xrOk(xrGetSystemProperties(instance_, systemId_, &sys), "xrGetSystemProperties")) {
            LOGI("system: %s, orientation tracking %d, position tracking %d, max layers %u", sys.systemName,
                 sys.trackingProperties.orientationTracking, sys.trackingProperties.positionTracking, sys.graphicsProperties.maxLayerCount);
        }

        PFN_xrGetOpenGLESGraphicsRequirementsKHR getReq = nullptr;
        if (!xrOk(xrGetInstanceProcAddr(instance_, "xrGetOpenGLESGraphicsRequirementsKHR",
                                        reinterpret_cast<PFN_xrVoidFunction*>(&getReq)), "xrGetInstanceProcAddr(xrGetOpenGLESGraphicsRequirementsKHR)") || !getReq) {
            error = "The runtime does not report its OpenGL ES requirements.";
            return false;
        }
        XrGraphicsRequirementsOpenGLESKHR req{XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_ES_KHR};
        if (!xrOk(getReq(instance_, systemId_, &req), "xrGetOpenGLESGraphicsRequirementsKHR")) {
            error = "The runtime would not report its OpenGL ES requirements.";
            return false;
        }
        LOGI("runtime wants OpenGL ES %u.%u to %u.%u", XR_VERSION_MAJOR(req.minApiVersionSupported),
             XR_VERSION_MINOR(req.minApiVersionSupported), XR_VERSION_MAJOR(req.maxApiVersionSupported),
             XR_VERSION_MINOR(req.maxApiVersionSupported));

        if (!initEgl(error)) return false;

        uint32_t viewCount = 0;
        xrEnumerateViewConfigurationViews(instance_, systemId_, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 0, &viewCount, nullptr);
        if (viewCount != 2) {
            error = "The headset did not report a stereo view configuration.";
            return false;
        }
        for (auto& v : viewConfig_) v = {XR_TYPE_VIEW_CONFIGURATION_VIEW};
        xrEnumerateViewConfigurationViews(instance_, systemId_, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 2, &viewCount, viewConfig_);
        LOGI("recommended eye size %ux%u (max %ux%u)", viewConfig_[0].recommendedImageRectWidth, viewConfig_[0].recommendedImageRectHeight,
             viewConfig_[0].maxImageRectWidth, viewConfig_[0].maxImageRectHeight);

        XrGraphicsBindingOpenGLESAndroidKHR binding{XR_TYPE_GRAPHICS_BINDING_OPENGL_ES_ANDROID_KHR};
        binding.display = display_;
        binding.config = config_;
        binding.context = context_;
        XrSessionCreateInfo sci{XR_TYPE_SESSION_CREATE_INFO};
        sci.next = &binding;
        sci.systemId = systemId_;
        if (!xrOk(xrCreateSession(instance_, &sci, &session_), "xrCreateSession")) {
            error = "The OpenXR session could not be created.";
            return false;
        }

        if (!createSpaces(error)) return false;
        if (!createSwapchains(error)) return false;
        if (!createActions(error)) return false;
        if (!createScene(error)) return false;
        if (!createSurfaceQuads(error)) return false;
        if (mode_ == MODE_STEAMVR && !startSteamVr(error)) return false;
        padArray_ = static_cast<jfloatArray>(env_->NewGlobalRef(env_->NewFloatArray(8)));
        LOGI("session created (mode %s); waiting for the runtime to make it ready", modeName());
        return true;
    }

    const char* modeName() const { return mode_ == MODE_THEATER ? "theater" : mode_ == MODE_STEAMVR ? "SteamVR" : "home"; }

    bool startSteamVr(std::string& error) {
        XrReferenceSpaceCreateInfo rs{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
        rs.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
        rs.poseInReferenceSpace.orientation.w = 1.0f;
        if (!xrOk(xrCreateReferenceSpace(session_, &rs, &viewSpace_), "xrCreateReferenceSpace(VIEW)")) {
            error = "The head's tracking space could not be created.";
            return false;
        }
        const char* vs =
            "#version 300 es\n"
            "uniform float uX0;\n"
            "out vec2 vUv;\n"
            "void main(){ vec2 p = vec2(gl_VertexID == 1 ? 3.0 : -1.0, gl_VertexID == 2 ? 3.0 : -1.0);\n"
            "  vec2 uv = p * 0.5 + 0.5; vUv = vec2(uX0 + uv.x * 0.5, 1.0 - uv.y); gl_Position = vec4(p, 0.0, 1.0); }\n";
        const char* fs =
            "#version 300 es\n"
            "precision mediump float;\n"
            "uniform sampler2D uTex;\n"
            "uniform int uDecode;\n"
            "in vec2 vUv;\n"
            "out vec4 outColor;\n"
            "vec3 lin(vec3 c){ return mix(c / 12.92, pow((c + 0.055) / 1.055, vec3(2.4)), step(0.04045, c)); }\n"
            "void main(){ vec3 c = texture(uTex, vUv).rgb; outColor = vec4(uDecode == 1 ? lin(c) : c, 1.0); }\n";
        GLuint v = compile(GL_VERTEX_SHADER, vs), f = compile(GL_FRAGMENT_SHADER, fs);
        if (!v || !f) {
            error = "The SteamVR view's shaders did not compile.";
            return false;
        }
        blitProgram_ = glCreateProgram();
        glAttachShader(blitProgram_, v);
        glAttachShader(blitProgram_, f);
        glLinkProgram(blitProgram_);
        glDeleteShader(v);
        glDeleteShader(f);
        uBlitX0_ = glGetUniformLocation(blitProgram_, "uX0");
        uBlitDecode_ = glGetUniformLocation(blitProgram_, "uDecode");
        uBlitTex_ = glGetUniformLocation(blitProgram_, "uTex");
        glGenVertexArrays(1, &blitVao_);
        linkUp_ = link_.start(display_);
        if (!linkUp_) {
            error = "The link SteamVR's driver connects to could not be opened.";
            return false;
        }
        return true;
    }

    void steamVrTick(const XrFrameState& fs) {
        if (!linkUp_) return;
        haptics_.clear();
        link_.poll(haptics_);
        for (const qdvr_haptic& h : haptics_)
            if (h.hand < 2) buzz(int(h.hand), std::min(1.0f, std::max(0.0f, h.amplitude)), int64_t(std::max(0.01f, h.durationSec) * 1e9f));
        if (!link_.connected()) return;
        if (!haveEyes_) {
            XrView views[2] = {{XR_TYPE_VIEW}, {XR_TYPE_VIEW}};
            XrViewState vstate{XR_TYPE_VIEW_STATE};
            XrViewLocateInfo li{XR_TYPE_VIEW_LOCATE_INFO};
            li.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
            li.displayTime = fs.predictedDisplayTime;
            li.space = viewSpace_;
            uint32_t n = 0;
            if (!XR_SUCCEEDED(xrLocateViews(session_, &li, &vstate, 2, &n, views)) || n != 2) return;
            for (int e = 0; e < 2; e++) {
                eyeToHead_[e] = views[e].pose;
                helloFov_[e] = views[e].fov;
                hello_.fov[e] = {std::tan(views[e].fov.angleLeft), std::tan(views[e].fov.angleRight), std::tan(views[e].fov.angleUp),
                                 std::tan(views[e].fov.angleDown)};
                hello_.eyeToHead[e] = toWire(views[e].pose);
            }
            hello_.type = QDVR_MSG_HELLO;
            hello_.version = QDVR_PROTOCOL_VERSION;
            hello_.eyeWidth = viewConfig_[0].recommendedImageRectWidth;
            hello_.eyeHeight = viewConfig_[0].recommendedImageRectHeight;
            hello_.refreshHz = fs.predictedDisplayPeriod > 0 ? float(1e9 / double(fs.predictedDisplayPeriod)) : 72.0f;
            haveEyes_ = true;
            LOGI("SteamVR headset: %ux%u per eye, %.0f Hz, IPD %.1f mm", hello_.eyeWidth, hello_.eyeHeight, hello_.refreshHz,
                 1000.0f * std::fabs(eyeToHead_[1].position.x - eyeToHead_[0].position.x));
        }
        if (!link_.helloSent()) link_.sendHello(hello_);

        qdvr_tracking t{};
        t.type = QDVR_MSG_TRACKING;
        t.seq = ++trackingSeq_;
        t.displayTimeNs = fs.predictedDisplayTime;
        t.frameIntervalSec = float(fs.predictedDisplayPeriod) * 1e-9f;
        XrSpaceLocation head{XR_TYPE_SPACE_LOCATION};
        const XrSpaceLocationFlags need = XR_SPACE_LOCATION_POSITION_VALID_BIT | XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;
        t.headValid = XR_SUCCEEDED(xrLocateSpace(viewSpace_, appSpace_, fs.predictedDisplayTime, &head)) && (head.locationFlags & need) == need;
        if (t.headValid) t.head = toWire(head.pose);
        for (int h = 0; h < 2; h++) {
            const Hand& hd = hands_[h];
            qdvr_hand& w = t.hands[h];
            if (hd.active) w.flags |= QDVR_HAND_ACTIVE;
            if (hd.gripValid && hd.aimValid) w.flags |= QDVR_HAND_POSE_VALID;
            w.gripPose = toWire(hd.grip);
            w.aimPose = toWire(hd.aim);
            w.trigger = hd.trigger;
            w.grip = hd.squeeze;
            w.stickX = hd.stickX;
            w.stickY = hd.stickY;
            const bool first = h == 0 ? x_ : a_, second = h == 0 ? y_ : b_;
            if (first) w.buttons |= QDVR_BTN_A;
            if (second) w.buttons |= QDVR_BTN_B;
            if (h == 0 && menu_down_) w.buttons |= QDVR_BTN_SYSTEM;
            if (hd.stickClick) w.buttons |= QDVR_BTN_STICK;
            if (hd.trigger > 0.75f) w.buttons |= QDVR_BTN_TRIGGER;
            if (hd.squeeze > 0.75f) w.buttons |= QDVR_BTN_GRIP;
        }
        link_.sendTracking(t);
    }

    void drawSteamVrEye(int eye, uint32_t imageIndex, const SteamVrLink::Frame& f) {
        EyeSwapchain& sc = eyes_[eye];
        glBindFramebuffer(GL_FRAMEBUFFER, sc.fbos[imageIndex]);
        glViewport(0, 0, sc.width, sc.height);
        glDisable(GL_DEPTH_TEST);
        glUseProgram(blitProgram_);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, f.texture);
        glUniform1i(uBlitTex_, 0);
        glUniform1f(uBlitX0_, eye == 0 ? 0.0f : 0.5f);
        glUniform1i(uBlitDecode_, (f.info.srgb && colorFormat_ == GL_SRGB8_ALPHA8) ? 1 : 0);
        glBindVertexArray(blitVao_);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glBindVertexArray(0);
        glBindTexture(GL_TEXTURE_2D, 0);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    bool initEgl(std::string& error) {
        display_ = eglGetDisplay(EGL_DEFAULT_DISPLAY);
        EGLint major = 0, minor = 0;
        if (display_ == EGL_NO_DISPLAY || !eglInitialize(display_, &major, &minor)) {
            error = "EGL could not be initialised.";
            return false;
        }
        const EGLint attribs[] = {EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT, EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
                                  EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_ALPHA_SIZE, 8, EGL_NONE};
        EGLint n = 0;
        if (!eglChooseConfig(display_, attribs, &config_, 1, &n) || n < 1) {
            error = "No suitable EGL configuration for OpenGL ES 3.";
            return false;
        }
        const EGLint ctxAttribs[] = {EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE};
        context_ = eglCreateContext(display_, config_, EGL_NO_CONTEXT, ctxAttribs);
        const EGLint pbuf[] = {EGL_WIDTH, 16, EGL_HEIGHT, 16, EGL_NONE};
        surface_ = eglCreatePbufferSurface(display_, config_, pbuf);
        if (context_ == EGL_NO_CONTEXT || surface_ == EGL_NO_SURFACE || !eglMakeCurrent(display_, surface_, surface_, context_)) {
            error = "The OpenGL ES 3 context could not be created.";
            return false;
        }
        LOGI("EGL %d.%d, GL renderer: %s", major, minor, reinterpret_cast<const char*>(glGetString(GL_RENDERER)));
        return true;
    }

    bool createSpaces(std::string& error) {
        uint32_t n = 0;
        xrEnumerateReferenceSpaces(session_, 0, &n, nullptr);
        std::vector<XrReferenceSpaceType> types(n);
        xrEnumerateReferenceSpaces(session_, n, &n, types.data());
        for (auto t : types)
            if (t == XR_REFERENCE_SPACE_TYPE_STAGE) stageSpace_ = true;
        XrReferenceSpaceCreateInfo rs{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
        rs.referenceSpaceType = stageSpace_ ? XR_REFERENCE_SPACE_TYPE_STAGE : XR_REFERENCE_SPACE_TYPE_LOCAL;
        rs.poseInReferenceSpace.orientation.w = 1.0f;
        if (!xrOk(xrCreateReferenceSpace(session_, &rs, &appSpace_), "xrCreateReferenceSpace")) {
            error = "The tracking space could not be created.";
            return false;
        }
        LOGI("tracking space: %s", stageSpace_ ? "STAGE (floor level)" : "LOCAL (head start; the floor is assumed 1.5 m down)");
        return true;
    }

    bool createSwapchains(std::string& error) {
        uint32_t fmtCount = 0;
        xrEnumerateSwapchainFormats(session_, 0, &fmtCount, nullptr);
        std::vector<int64_t> formats(fmtCount);
        xrEnumerateSwapchainFormats(session_, fmtCount, &fmtCount, formats.data());
        colorFormat_ = 0;
        for (int64_t f : formats)
            if (f == GL_SRGB8_ALPHA8) colorFormat_ = f;
        if (!colorFormat_)
            for (int64_t f : formats)
                if (f == GL_RGBA8) colorFormat_ = f;
        if (!colorFormat_ && !formats.empty()) colorFormat_ = formats[0];
        if (!colorFormat_) {
            error = "The runtime offered no swapchain formats.";
            return false;
        }
        LOGI("swapchain format 0x%llx (%s)", static_cast<long long>(colorFormat_), colorFormat_ == GL_SRGB8_ALPHA8 ? "sRGB" : "linear");
        for (int e = 0; e < 2; e++) {
            EyeSwapchain& eye = eyes_[e];
            eye.width = static_cast<int32_t>(viewConfig_[e].recommendedImageRectWidth);
            eye.height = static_cast<int32_t>(viewConfig_[e].recommendedImageRectHeight);
            XrSwapchainCreateInfo sc{XR_TYPE_SWAPCHAIN_CREATE_INFO};
            sc.usageFlags = XR_SWAPCHAIN_USAGE_SAMPLED_BIT | XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT;
            sc.format = colorFormat_;
            sc.sampleCount = 1;
            sc.width = static_cast<uint32_t>(eye.width);
            sc.height = static_cast<uint32_t>(eye.height);
            sc.faceCount = 1;
            sc.arraySize = 1;
            sc.mipCount = 1;
            if (!xrOk(xrCreateSwapchain(session_, &sc, &eye.handle), "xrCreateSwapchain")) {
                error = "A swapchain could not be created.";
                return false;
            }
            uint32_t count = 0;
            xrEnumerateSwapchainImages(eye.handle, 0, &count, nullptr);
            eye.images.assign(count, {XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_ES_KHR});
            xrEnumerateSwapchainImages(eye.handle, count, &count, reinterpret_cast<XrSwapchainImageBaseHeader*>(eye.images.data()));

            glGenRenderbuffers(1, &eye.depth);
            glBindRenderbuffer(GL_RENDERBUFFER, eye.depth);
            glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, eye.width, eye.height);
            eye.fbos.assign(count, 0);
            glGenFramebuffers(static_cast<GLsizei>(count), eye.fbos.data());
            for (uint32_t i = 0; i < count; i++) {
                glBindFramebuffer(GL_FRAMEBUFFER, eye.fbos[i]);
                glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, eye.images[i].image, 0);
                glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, eye.depth);
                if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
                    error = "An eye framebuffer is incomplete.";
                    return false;
                }
            }
            LOGI("eye %d: %dx%d, %u images", e, eye.width, eye.height, count);
        }
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        return true;
    }

    bool createSurfaceQuad(SurfaceQuad& q, int w, int h, const char* what) {
        if (!createSurfaceSwapchain_) return false;
        struct Shape {
            XrSwapchainUsageFlags usage;
            uint32_t samples, faces, layers, mips;
            const char* name;
        };
        const Shape shapes[] = {
            {XR_SWAPCHAIN_USAGE_SAMPLED_BIT | XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT, 0, 0, 0, 0, "zeros"},
            {XR_SWAPCHAIN_USAGE_SAMPLED_BIT, 0, 0, 0, 0, "zeros, sampled only"},
            {XR_SWAPCHAIN_USAGE_SAMPLED_BIT, 1, 1, 1, 1, "ones, sampled only"},
            {XR_SWAPCHAIN_USAGE_SAMPLED_BIT | XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT, 1, 1, 1, 1, "ones"},
        };
        jobject surface = nullptr;
        bool made = false;
        for (const Shape& s : shapes) {
            XrSwapchainCreateInfo sc{XR_TYPE_SWAPCHAIN_CREATE_INFO};
            sc.usageFlags = s.usage;
            sc.format = 0;
            sc.sampleCount = s.samples;
            sc.width = static_cast<uint32_t>(w);
            sc.height = static_cast<uint32_t>(h);
            sc.faceCount = s.faces;
            sc.arraySize = s.layers;
            sc.mipCount = s.mips;
            q.handle = XR_NULL_HANDLE;
            surface = nullptr;
            const XrResult r = createSurfaceSwapchain_(session_, &sc, &q.handle, &surface);
            if (XR_SUCCEEDED(r) && surface) {
                LOGI("%s: Android-surface swapchain accepted with %s", what, s.name);
                made = true;
                break;
            }
            LOGW("%s: Android-surface swapchain refused with %s (%d)", what, s.name, static_cast<int>(r));
            if (q.handle) xrDestroySwapchain(q.handle);
        }
        if (!made) {
            q.handle = XR_NULL_HANDLE;
            return false;
        }
        if (env_->GetObjectRefType(surface) == JNILocalRefType) {
            q.surface = env_->NewGlobalRef(surface);
            env_->DeleteLocalRef(surface);
            q.ownsSurfaceRef = true;
        } else {
            q.surface = surface;
            q.ownsSurfaceRef = false;
        }
        q.width = w;
        q.height = h;
        LOGI("%s: %dx%d Android-surface swapchain", what, w, h);
        return true;
    }

    bool createSurfaceQuads(std::string& error) {
        if (!createSurfaceSwapchain_) {
            if (mode_ == MODE_THEATER) {
                error = "The VR screen could not be created on this runtime.";
                return false;
            }
            return true;
        }
        if (mode_ == MODE_THEATER) {
            if (!createSurfaceQuad(theater_, theaterW_, theaterH_, "theater screen")) {
                error = "The VR screen could not be created.";
                return false;
            }
            callSurface(cb.onTheaterSurface, theater_.surface, theater_.width, theater_.height);
        }
        if (createSurfaceQuad(panel_, PANEL_W, PANEL_H, "panel")) callSurface(cb.onPanelSurface, panel_.surface, panel_.width, panel_.height);
        return true;
    }

    bool createActions(std::string& error) {
        hand_[0] = path("/user/hand/left");
        hand_[1] = path("/user/hand/right");

        XrActionSetCreateInfo as{XR_TYPE_ACTION_SET_CREATE_INFO};
        std::strcpy(as.actionSetName, "questdeck");
        std::strcpy(as.localizedActionSetName, "QuestDeck");
        if (!xrOk(xrCreateActionSet(instance_, &as, &actionSet_), "xrCreateActionSet")) {
            error = "Controller input could not be set up.";
            return false;
        }
        auto make = [&](XrAction* out, const char* name, const char* localized, XrActionType type, bool perHand) {
            XrActionCreateInfo ai{XR_TYPE_ACTION_CREATE_INFO};
            ai.actionType = type;
            std::strcpy(ai.actionName, name);
            std::strcpy(ai.localizedActionName, localized);
            ai.countSubactionPaths = perHand ? 2 : 0;
            ai.subactionPaths = perHand ? hand_ : nullptr;
            return xrOk(xrCreateAction(actionSet_, &ai, out), name);
        };
        if (!make(&aimPose_, "aim_pose", "Aim pose", XR_ACTION_TYPE_POSE_INPUT, true) ||
            !make(&gripPose_, "grip_pose", "Grip pose", XR_ACTION_TYPE_POSE_INPUT, true) ||
            !make(&trigger_, "trigger", "Trigger", XR_ACTION_TYPE_FLOAT_INPUT, true) ||
            !make(&squeeze_, "squeeze", "Squeeze", XR_ACTION_TYPE_FLOAT_INPUT, true) ||
            !make(&thumbstick_, "thumbstick", "Thumbstick", XR_ACTION_TYPE_VECTOR2F_INPUT, true) ||
            !make(&stickClick_, "thumbstick_click", "Thumbstick click", XR_ACTION_TYPE_BOOLEAN_INPUT, true) ||
            !make(&haptic_, "haptic", "Haptic", XR_ACTION_TYPE_VIBRATION_OUTPUT, true) ||
            !make(&btnA_, "button_a", "A", XR_ACTION_TYPE_BOOLEAN_INPUT, false) ||
            !make(&btnB_, "button_b", "B", XR_ACTION_TYPE_BOOLEAN_INPUT, false) ||
            !make(&btnX_, "button_x", "X", XR_ACTION_TYPE_BOOLEAN_INPUT, false) ||
            !make(&btnY_, "button_y", "Y", XR_ACTION_TYPE_BOOLEAN_INPUT, false) ||
            !make(&menu_, "menu", "Menu", XR_ACTION_TYPE_BOOLEAN_INPUT, false)) {
            error = "Controller actions could not be created.";
            return false;
        }

        struct Bind {
            XrAction a;
            const char* p;
        };
        const Bind touch[] = {
            {aimPose_, "/user/hand/left/input/aim/pose"},       {aimPose_, "/user/hand/right/input/aim/pose"},
            {gripPose_, "/user/hand/left/input/grip/pose"},     {gripPose_, "/user/hand/right/input/grip/pose"},
            {trigger_, "/user/hand/left/input/trigger/value"},  {trigger_, "/user/hand/right/input/trigger/value"},
            {squeeze_, "/user/hand/left/input/squeeze/value"},  {squeeze_, "/user/hand/right/input/squeeze/value"},
            {thumbstick_, "/user/hand/left/input/thumbstick"},  {thumbstick_, "/user/hand/right/input/thumbstick"},
            {stickClick_, "/user/hand/left/input/thumbstick/click"}, {stickClick_, "/user/hand/right/input/thumbstick/click"},
            {haptic_, "/user/hand/left/output/haptic"},         {haptic_, "/user/hand/right/output/haptic"},
            {btnA_, "/user/hand/right/input/a/click"},          {btnB_, "/user/hand/right/input/b/click"},
            {btnX_, "/user/hand/left/input/x/click"},           {btnY_, "/user/hand/left/input/y/click"},
            {menu_, "/user/hand/left/input/menu/click"},
        };
        suggest("/interaction_profiles/oculus/touch_controller", touch, sizeof(touch) / sizeof(touch[0]));
        const Bind simple[] = {
            {aimPose_, "/user/hand/left/input/aim/pose"},       {aimPose_, "/user/hand/right/input/aim/pose"},
            {gripPose_, "/user/hand/left/input/grip/pose"},     {gripPose_, "/user/hand/right/input/grip/pose"},
            {trigger_, "/user/hand/left/input/select/click"},   {trigger_, "/user/hand/right/input/select/click"},
            {menu_, "/user/hand/left/input/menu/click"},
            {haptic_, "/user/hand/left/output/haptic"},         {haptic_, "/user/hand/right/output/haptic"},
        };
        suggest("/interaction_profiles/khr/simple_controller", simple, sizeof(simple) / sizeof(simple[0]));

        for (int h = 0; h < 2; h++) {
            XrActionSpaceCreateInfo si{XR_TYPE_ACTION_SPACE_CREATE_INFO};
            si.poseInActionSpace.orientation.w = 1.0f;
            si.subactionPath = hand_[h];
            si.action = aimPose_;
            xrOk(xrCreateActionSpace(session_, &si, &aimSpace_[h]), "xrCreateActionSpace(aim)");
            si.action = gripPose_;
            xrOk(xrCreateActionSpace(session_, &si, &gripSpace_[h]), "xrCreateActionSpace(grip)");
        }

        XrSessionActionSetsAttachInfo attach{XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO};
        attach.countActionSets = 1;
        attach.actionSets = &actionSet_;
        if (!xrOk(xrAttachSessionActionSets(session_, &attach), "xrAttachSessionActionSets")) {
            error = "Controller input could not be attached to the session.";
            return false;
        }
        return true;
    }

    template <typename B>
    void suggest(const char* profile, const B* bindings, size_t count) {
        std::vector<XrActionSuggestedBinding> sb;
        for (size_t i = 0; i < count; i++) sb.push_back({bindings[i].a, path(bindings[i].p)});
        XrInteractionProfileSuggestedBinding s{XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
        s.interactionProfile = path(profile);
        s.suggestedBindings = sb.data();
        s.countSuggestedBindings = static_cast<uint32_t>(sb.size());
        if (xrOk(xrSuggestInteractionProfileBindings(instance_, &s), profile)) LOGI("input bindings suggested for %s", profile);
    }

    GLuint compile(GLenum type, const char* src) {
        GLuint s = glCreateShader(type);
        glShaderSource(s, 1, &src, nullptr);
        glCompileShader(s);
        GLint ok = 0;
        glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
        if (!ok) {
            char log[512];
            glGetShaderInfoLog(s, sizeof(log), nullptr, log);
            LOGE("shader compile: %s", log);
            glDeleteShader(s);
            return 0;
        }
        return s;
    }

    bool createScene(std::string& error) {
        const char* vs =
            "#version 300 es\n"
            "layout(location=0) in vec3 aPos;\n"
            "layout(location=1) in vec3 aColor;\n"
            "uniform mat4 uMvp;\n"
            "out vec3 vColor;\n"
            "void main(){ vColor = aColor; gl_Position = uMvp * vec4(aPos, 1.0); }\n";
        const char* fs =
            "#version 300 es\n"
            "precision mediump float;\n"
            "in vec3 vColor;\n"
            "out vec4 outColor;\n"
            "void main(){ outColor = vec4(vColor, 1.0); }\n";
        GLuint v = compile(GL_VERTEX_SHADER, vs), f = compile(GL_FRAGMENT_SHADER, fs);
        if (!v || !f) {
            error = "The room's shaders did not compile.";
            return false;
        }
        program_ = glCreateProgram();
        glAttachShader(program_, v);
        glAttachShader(program_, f);
        glLinkProgram(program_);
        glDeleteShader(v);
        glDeleteShader(f);
        GLint ok = 0;
        glGetProgramiv(program_, GL_LINK_STATUS, &ok);
        if (!ok) {
            error = "The room's shaders did not link.";
            return false;
        }
        uMvp_ = glGetUniformLocation(program_, "uMvp");

        const float dim = mode_ == MODE_THEATER ? 0.45f : 1.0f;
        std::vector<Vertex> verts;
        for (int i = -10; i <= 10; i++) {
            const float c = ((i == 0) ? 0.55f : 0.22f) * dim;
            verts.push_back({static_cast<float>(i), 0, -10, c, c, c + 0.1f * dim});
            verts.push_back({static_cast<float>(i), 0, 10, c, c, c + 0.1f * dim});
            verts.push_back({-10, 0, static_cast<float>(i), c, c, c + 0.1f * dim});
            verts.push_back({10, 0, static_cast<float>(i), c, c, c + 0.1f * dim});
        }
        gridVertexCount_ = static_cast<int>(verts.size());
        cubeFirst_ = gridVertexCount_;
        struct Face {
            float n[3], u[3], v[3], c[3];
        };
        const Face faces[6] = {
            {{0, 0, 1}, {1, 0, 0}, {0, 1, 0}, {0.9f, 0.3f, 0.3f}},   {{0, 0, -1}, {-1, 0, 0}, {0, 1, 0}, {0.3f, 0.9f, 0.3f}},
            {{1, 0, 0}, {0, 0, -1}, {0, 1, 0}, {0.3f, 0.4f, 0.95f}}, {{-1, 0, 0}, {0, 0, 1}, {0, 1, 0}, {0.9f, 0.8f, 0.2f}},
            {{0, 1, 0}, {1, 0, 0}, {0, 0, -1}, {0.8f, 0.4f, 0.9f}},  {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}, {0.3f, 0.8f, 0.8f}},
        };
        for (const Face& fc : faces) {
            auto corner = [&](float su, float sv) {
                return Vertex{0.5f * (fc.n[0] + su * fc.u[0] + sv * fc.v[0]), 0.5f * (fc.n[1] + su * fc.u[1] + sv * fc.v[1]),
                              0.5f * (fc.n[2] + su * fc.u[2] + sv * fc.v[2]), fc.c[0], fc.c[1], fc.c[2]};
            };
            const Vertex a = corner(-1, -1), b = corner(1, -1), c = corner(1, 1), d = corner(-1, 1);
            for (const Vertex& p : {a, b, c, a, c, d}) verts.push_back(p);
        }
        cubeVertexCount_ = static_cast<int>(verts.size()) - cubeFirst_;

        glGenBuffers(1, &staticVbo_);
        glBindBuffer(GL_ARRAY_BUFFER, staticVbo_);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(verts.size() * sizeof(Vertex)), verts.data(), GL_STATIC_DRAW);
        glGenBuffers(1, &dynamicVbo_);
        glBindBuffer(GL_ARRAY_BUFFER, dynamicVbo_);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(64 * sizeof(Vertex)), nullptr, GL_DYNAMIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        return true;
    }

    void bindVertices(GLuint vbo) {
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(0));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(3 * sizeof(float)));
    }

    void drawCube(const Mat4& viewProj, const Mat4& model) {
        const Mat4 mvp = mul(viewProj, model);
        glUniformMatrix4fv(uMvp_, 1, GL_FALSE, mvp.m);
        glDrawArrays(GL_TRIANGLES, cubeFirst_, cubeVertexCount_);
    }

    float floorY() const { return stageSpace_ ? 0.0f : -1.5f; }

    void renderEye(int eye, const XrView& view, uint32_t imageIndex) {
        EyeSwapchain& sc = eyes_[eye];
        glBindFramebuffer(GL_FRAMEBUFFER, sc.fbos[imageIndex]);
        glViewport(0, 0, sc.width, sc.height);
        glScissor(0, 0, sc.width, sc.height);
        if (mode_ == MODE_THEATER) glClearColor(0.008f, 0.01f, 0.02f, 1.0f);
        else glClearColor(0.02f, 0.03f, 0.07f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if (mode_ == MODE_STEAMVR) {
            glClearColor(0, 0, 0, 1);
            glClear(GL_COLOR_BUFFER_BIT);
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            return;
        }
        glEnable(GL_DEPTH_TEST);

        const Mat4 proj = projectionFov(view.fov, 0.05f, 100.0f);
        const Mat4 viewM = invertRigid(fromPose(view.pose));
        const Mat4 viewProj = mul(proj, viewM);
        glUseProgram(program_);

        bindVertices(staticVbo_);
        const Mat4 world = translation(0, floorY(), 0);
        glUniformMatrix4fv(uMvp_, 1, GL_FALSE, mul(viewProj, world).m);
        glDrawArrays(GL_LINES, 0, gridVertexCount_);

        if (mode_ == MODE_HOME) {
            drawCube(viewProj, mul(mul(world, translation(0, 1.3f, -2.0f)), mul(rotationY(spin_), scale(0.4f, 0.4f, 0.4f))));
            drawCube(viewProj, mul(world, mul(translation(-2.0f, 0.5f, -3.0f), scale(1.0f, 1.0f, 1.0f))));
            drawCube(viewProj, mul(world, mul(translation(2.0f, 0.75f, -3.0f), scale(0.8f, 1.5f, 0.8f))));
            drawCube(viewProj, mul(world, mul(translation(0.0f, 0.25f, -5.0f), scale(2.0f, 0.5f, 0.5f))));
        }

        for (int h = 0; h < 2; h++) {
            const Hand& hd = hands_[h];
            if (hd.gripValid) drawCube(viewProj, mul(fromPose(hd.grip), scale(0.04f, 0.04f, 0.1f)));
            if (hd.aimValid) {
                const Mat4 aim = fromPose(hd.aim);
                float r = 1.0f - hd.trigger, g = 1.0f, b = 1.0f - hd.trigger;
                if (hd.onPanel) { r = 0.35f; g = 0.65f; b = 1.0f; }
                Vertex ray[2] = {{0, 0, 0, r, g, b}, {0, 0, -hd.rayLength, r, g, b}};
                glBindBuffer(GL_ARRAY_BUFFER, dynamicVbo_);
                glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(ray), ray);
                bindVertices(dynamicVbo_);
                glUniformMatrix4fv(uMvp_, 1, GL_FALSE, mul(viewProj, aim).m);
                glDrawArrays(GL_LINES, 0, 2);
                bindVertices(staticVbo_);
            }
        }
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    void anchorFrom(const XrPosef& head) {
        anchorYaw_ = yawOf(head.orientation);
        anchorPos_ = head.position;
        anchored_ = true;
        LOGI("anchored at (%.2f, %.2f, %.2f) heading %.0f deg", anchorPos_.x, anchorPos_.y, anchorPos_.z, anchorYaw_ * 57.2958f);
    }

    void placeQuads() {
        if (!anchored_) return;
        const XrQuaternionf facing = yawQuat(anchorYaw_);
        const float dist = screenDistance.load(), width = screenWidth.load();
        if (theater_.handle) {
            const float h = width * static_cast<float>(theater_.height) / static_cast<float>(theater_.width);
            theater_.size = {width, h};
            theater_.pose.orientation = facing;
            theater_.pose.position = add(anchorPos_, rotate(facing, {0, 0.05f, -dist}));
            theater_.visible = true;
        }
        if (panel_.handle) {
            const float pw = 1.1f, ph = pw * static_cast<float>(PANEL_H) / static_cast<float>(PANEL_W);
            panel_.size = {pw, ph};
            const float below = theater_.handle ? -(theater_.size.height * 0.5f + 0.1f + ph * 0.5f) : -0.45f;
            const float pDist = theater_.handle ? std::min(dist * 0.55f, 1.25f) : 1.0f;
            const XrQuaternionf tilt{std::sin(-0.25f * 0.5f), 0, 0, std::cos(-0.25f * 0.5f)};
            panel_.pose.orientation = {facing.w * tilt.x + facing.x * tilt.w + facing.y * tilt.z - facing.z * tilt.y,
                                       facing.w * tilt.y - facing.x * tilt.z + facing.y * tilt.w + facing.z * tilt.x,
                                       facing.w * tilt.z + facing.x * tilt.y - facing.y * tilt.x + facing.z * tilt.w,
                                       facing.w * tilt.w - facing.x * tilt.x - facing.y * tilt.y - facing.z * tilt.z};
            panel_.pose.position = add(anchorPos_, rotate(facing, {0, below * (theater_.handle ? (pDist / dist) : 1.0f), -pDist}));
            panel_.visible = mode_ != MODE_STEAMVR;
        }
    }

    static bool hitQuad(const SurfaceQuad& q, const XrPosef& ray, float& u, float& v, float& t) {
        if (!q.visible) return false;
        const XrQuaternionf inv = conjugate(q.pose.orientation);
        const XrVector3f o = rotate(inv, sub(ray.position, q.pose.position));
        const XrVector3f d = rotate(inv, rotate(ray.orientation, {0, 0, -1}));
        if (std::fabs(d.z) < 1e-5f) return false;
        t = -o.z / d.z;
        if (t <= 0) return false;
        const XrVector3f p = add(o, scaled(d, t));
        u = p.x / q.size.width + 0.5f;
        v = 0.5f - p.y / q.size.height;
        return u >= 0 && u <= 1 && v >= 0 && v <= 1;
    }

    void pollEvents() {
        for (;;) {
            XrEventDataBuffer ev{XR_TYPE_EVENT_DATA_BUFFER};
            const XrResult r = xrPollEvent(instance_, &ev);
            if (r != XR_SUCCESS) return;
            switch (ev.type) {
                case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED: {
                    const auto* s = reinterpret_cast<const XrEventDataSessionStateChanged*>(&ev);
                    state_ = s->state;
                    LOGI("session state -> %d", static_cast<int>(state_));
                    if (state_ == XR_SESSION_STATE_READY && !sessionRunning_) {
                        XrSessionBeginInfo bi{XR_TYPE_SESSION_BEGIN_INFO};
                        bi.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
                        if (xrOk(xrBeginSession(session_, &bi), "xrBeginSession")) {
                            sessionRunning_ = true;
                            LOGI("session running");
                        }
                    } else if (state_ == XR_SESSION_STATE_STOPPING && sessionRunning_) {
                        xrOk(xrEndSession(session_), "xrEndSession");
                        sessionRunning_ = false;
                        LOGI("session ended by the runtime");
                    } else if (state_ == XR_SESSION_STATE_EXITING || state_ == XR_SESSION_STATE_LOSS_PENDING) {
                        LOGW("runtime is closing the session (state %d)", static_cast<int>(state_));
                        quit = true;
                    } else if (state_ != XR_SESSION_STATE_FOCUSED) {
                        releasePad();
                    }
                    break;
                }
                case XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING:
                    LOGW("instance loss pending");
                    quit = true;
                    break;
                case XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED:
                    logInteractionProfiles();
                    break;
                case XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING:
                    recenterAsked = true;
                    break;
                default:
                    break;
            }
        }
    }

    void logInteractionProfiles() {
        for (int h = 0; h < 2; h++) {
            XrInteractionProfileState ps{XR_TYPE_INTERACTION_PROFILE_STATE};
            if (XR_SUCCEEDED(xrGetCurrentInteractionProfile(session_, hand_[h], &ps)) && ps.interactionProfile != XR_NULL_PATH) {
                char buf[XR_MAX_PATH_LENGTH];
                uint32_t len = 0;
                xrPathToString(instance_, ps.interactionProfile, sizeof(buf), &len, buf);
                LOGI("%s hand profile: %s", h == 0 ? "left" : "right", buf);
            }
        }
    }

    bool boolState(XrAction a, XrPath sub) {
        XrActionStateGetInfo gi{XR_TYPE_ACTION_STATE_GET_INFO};
        gi.action = a;
        gi.subactionPath = sub;
        XrActionStateBoolean s{XR_TYPE_ACTION_STATE_BOOLEAN};
        return XR_SUCCEEDED(xrGetActionStateBoolean(session_, &gi, &s)) && s.isActive && s.currentState;
    }

    void buzz(int h, float amplitude, int64_t ns) {
        XrHapticVibration vib{XR_TYPE_HAPTIC_VIBRATION};
        vib.duration = ns;
        vib.frequency = XR_FREQUENCY_UNSPECIFIED;
        vib.amplitude = amplitude;
        XrHapticActionInfo hi{XR_TYPE_HAPTIC_ACTION_INFO};
        hi.action = haptic_;
        hi.subactionPath = hand_[h];
        xrApplyHapticFeedback(session_, &hi, reinterpret_cast<const XrHapticBaseHeader*>(&vib));
    }

    void syncInput(XrTime time) {
        XrActiveActionSet active{actionSet_, XR_NULL_PATH};
        XrActionsSyncInfo si{XR_TYPE_ACTIONS_SYNC_INFO};
        si.countActiveActionSets = 1;
        si.activeActionSets = &active;
        if (!XR_SUCCEEDED(xrSyncActions(session_, &si))) return;

        for (int h = 0; h < 2; h++) {
            Hand& hd = hands_[h];
            XrActionStateGetInfo gi{XR_TYPE_ACTION_STATE_GET_INFO};
            gi.subactionPath = hand_[h];

            gi.action = trigger_;
            XrActionStateFloat tf{XR_TYPE_ACTION_STATE_FLOAT};
            if (XR_SUCCEEDED(xrGetActionStateFloat(session_, &gi, &tf))) hd.trigger = tf.isActive ? tf.currentState : 0.0f;
            gi.action = squeeze_;
            XrActionStateFloat qf{XR_TYPE_ACTION_STATE_FLOAT};
            if (XR_SUCCEEDED(xrGetActionStateFloat(session_, &gi, &qf))) hd.squeeze = qf.isActive ? qf.currentState : 0.0f;
            gi.action = thumbstick_;
            XrActionStateVector2f sf{XR_TYPE_ACTION_STATE_VECTOR2F};
            if (XR_SUCCEEDED(xrGetActionStateVector2f(session_, &gi, &sf)) && sf.isActive) {
                hd.stickX = sf.currentState.x;
                hd.stickY = sf.currentState.y;
            } else {
                hd.stickX = hd.stickY = 0;
            }
            hd.stickClick = boolState(stickClick_, hand_[h]);
            gi.action = aimPose_;
            XrActionStatePose pf{XR_TYPE_ACTION_STATE_POSE};
            hd.active = XR_SUCCEEDED(xrGetActionStatePose(session_, &gi, &pf)) && pf.isActive;

            XrSpaceLocation loc{XR_TYPE_SPACE_LOCATION};
            const XrSpaceLocationFlags need = XR_SPACE_LOCATION_POSITION_VALID_BIT | XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;
            hd.aimValid = hd.active && XR_SUCCEEDED(xrLocateSpace(aimSpace_[h], appSpace_, time, &loc)) && (loc.locationFlags & need) == need;
            if (hd.aimValid) hd.aim = loc.pose;
            XrSpaceLocation gloc{XR_TYPE_SPACE_LOCATION};
            hd.gripValid = hd.active && XR_SUCCEEDED(xrLocateSpace(gripSpace_[h], appSpace_, time, &gloc)) && (gloc.locationFlags & need) == need;
            if (hd.gripValid) hd.grip = gloc.pose;

            const bool down = hd.squeeze > 0.8f;
            if (mode_ == MODE_HOME && down && !hd.squeezeWasDown) buzz(h, 0.5f, 40000000);
            hd.squeezeWasDown = down;

            float u = 0, v = 0, t = 0;
            hd.wasOnPanel = hd.onPanel;
            hd.onPanel = hd.aimValid && hitQuad(panel_, hd.aim, u, v, t);
            hd.rayLength = hd.onPanel ? t : 3.0f;
            if (hd.onPanel) {
                hd.panelU = u;
                hd.panelV = v;
                if (!hd.wasOnPanel) buzz(h, 0.15f, 8000000);
            }
        }
        a_ = boolState(btnA_, XR_NULL_PATH);
        b_ = boolState(btnB_, XR_NULL_PATH);
        x_ = boolState(btnX_, XR_NULL_PATH);
        y_ = boolState(btnY_, XR_NULL_PATH);
        menu_down_ = boolState(menu_, XR_NULL_PATH);
        spinSpeed_ = 0.6f + hands_[1].stickX * 3.0f;
        updatePanelPointer();
        if (mode_ == MODE_THEATER) sendPad();
    }

    void updatePanelPointer() {
        if (!panel_.handle || !cb.onPanelPointer) return;
        if (pointerHand_ >= 0 && !hands_[pointerHand_].onPanel) {
            if (hands_[pointerHand_].panelDown) env_->CallVoidMethod(cb.activity, cb.onPanelPointer, POINTER_UP, -1.0f, -1.0f);
            hands_[pointerHand_].panelDown = false;
            env_->CallVoidMethod(cb.activity, cb.onPanelPointer, POINTER_NONE, -1.0f, -1.0f);
            clearException();
            pointerHand_ = -1;
        }
        if (pointerHand_ < 0)
            for (int h = 1; h >= 0; h--)
                if (hands_[h].onPanel) {
                    pointerHand_ = h;
                    break;
                }
        if (pointerHand_ < 0) return;
        Hand& hd = hands_[pointerHand_];
        const bool down = hd.trigger > 0.6f;
        int state = POINTER_HOVER;
        if (down && !hd.panelDown) {
            state = POINTER_DOWN;
            buzz(pointerHand_, 0.35f, 15000000);
        } else if (!down && hd.panelDown) {
            state = POINTER_UP;
        }
        hd.panelDown = down;
        env_->CallVoidMethod(cb.activity, cb.onPanelPointer, state, hd.panelU, hd.panelV);
        clearException();
    }

    void sendPad() {
        if (!cb.onPad || !padArray_) return;
        const Hand& l = hands_[0];
        const Hand& r = hands_[1];
        int buttons = 0;
        if (a_) buttons |= 1 << BTN_A;
        if (b_) buttons |= 1 << BTN_B;
        if (x_) buttons |= 1 << BTN_X;
        if (y_) buttons |= 1 << BTN_Y;
        if (l.squeeze > 0.6f) buttons |= 1 << BTN_LB;
        if (r.squeeze > 0.6f) buttons |= 1 << BTN_RB;
        if (l.stickClick) buttons |= 1 << BTN_L3;
        if (r.stickClick) buttons |= 1 << BTN_R3;
        if (menu_down_) buttons |= 1 << BTN_START;
        const float pad[8] = {l.stickX, -l.stickY, r.stickX, -r.stickY,
                              l.onPanel ? 0.0f : l.trigger, r.onPanel ? 0.0f : r.trigger,
                              static_cast<float>(buttons), 1.0f};
        env_->SetFloatArrayRegion(padArray_, 0, 8, pad);
        env_->CallVoidMethod(cb.activity, cb.onPad, padArray_);
        clearException();
    }

    void releasePad() {
        if (mode_ != MODE_THEATER || !cb.onPad || !padArray_ || !env_) return;
        const float pad[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        env_->SetFloatArrayRegion(padArray_, 0, 8, pad);
        env_->CallVoidMethod(cb.activity, cb.onPad, padArray_);
        clearException();
    }

    void tickCompositor() {
        if (!theater_.handle || !cb.compositor || !cb.compositorVsync) return;
        const int64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        env_->CallStaticVoidMethod(cb.compositor, cb.compositorVsync, static_cast<jlong>(now));
        clearException();
    }

    void renderFrame() {
        XrFrameState fs{XR_TYPE_FRAME_STATE};
        if (!xrOk(xrWaitFrame(session_, nullptr, &fs), "xrWaitFrame")) return;
        tickCompositor();
        if (!xrOk(xrBeginFrame(session_, nullptr), "xrBeginFrame")) return;

        if (lastFrameTime_ != 0) spin_ += spinSpeed_ * static_cast<float>(fs.predictedDisplayTime - lastFrameTime_) * 1e-9f;
        lastFrameTime_ = fs.predictedDisplayTime;

        XrCompositionLayerProjection layer{XR_TYPE_COMPOSITION_LAYER_PROJECTION};
        XrCompositionLayerProjectionView layerViews[2]{};
        XrCompositionLayerQuad screenLayer{XR_TYPE_COMPOSITION_LAYER_QUAD}, panelLayer{XR_TYPE_COMPOSITION_LAYER_QUAD};
        const XrCompositionLayerBaseHeader* layers[3];
        uint32_t layerCount = 0;

        if (fs.shouldRender) {
            if (state_ == XR_SESSION_STATE_FOCUSED) syncInput(fs.predictedDisplayTime);
            if (mode_ == MODE_STEAMVR) steamVrTick(fs);
            SteamVrLink::Frame svr;
            const bool steamVrFrame = mode_ == MODE_STEAMVR && link_.connected() && haveEyes_ && link_.frame(svr);
            if (steamVrFrame != showingSteamVr_) {
                showingSteamVr_ = steamVrFrame;
                LOGI("%s", steamVrFrame ? "showing SteamVR" : "no SteamVR frames: showing the room");
            }

            XrView views[2] = {{XR_TYPE_VIEW}, {XR_TYPE_VIEW}};
            XrViewState vs{XR_TYPE_VIEW_STATE};
            XrViewLocateInfo li{XR_TYPE_VIEW_LOCATE_INFO};
            li.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
            li.displayTime = fs.predictedDisplayTime;
            li.space = appSpace_;
            uint32_t viewCount = 0;
            const XrViewStateFlags need = XR_VIEW_STATE_POSITION_VALID_BIT | XR_VIEW_STATE_ORIENTATION_VALID_BIT;
            if (XR_SUCCEEDED(xrLocateViews(session_, &li, &vs, 2, &viewCount, views)) && viewCount == 2 && (vs.viewStateFlags & need) == need) {
                if (!anchored_ || recenterAsked.exchange(false)) {
                    XrPosef head = views[0].pose;
                    head.position = {(views[0].pose.position.x + views[1].pose.position.x) * 0.5f,
                                     (views[0].pose.position.y + views[1].pose.position.y) * 0.5f,
                                     (views[0].pose.position.z + views[1].pose.position.z) * 0.5f};
                    anchorFrom(head);
                }
                placeQuads();
                bool ok = true;
                for (int e = 0; e < 2 && ok; e++) {
                    uint32_t idx = 0;
                    XrSwapchainImageAcquireInfo ai{XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
                    XrSwapchainImageWaitInfo wi{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
                    wi.timeout = XR_INFINITE_DURATION;
                    if (!xrOk(xrAcquireSwapchainImage(eyes_[e].handle, &ai, &idx), "xrAcquireSwapchainImage") ||
                        !xrOk(xrWaitSwapchainImage(eyes_[e].handle, &wi), "xrWaitSwapchainImage")) {
                        ok = false;
                        break;
                    }
                    if (steamVrFrame)
                        drawSteamVrEye(e, idx, svr);
                    else
                        renderEye(e, views[e], idx);
                    XrSwapchainImageReleaseInfo ri{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
                    xrOk(xrReleaseSwapchainImage(eyes_[e].handle, &ri), "xrReleaseSwapchainImage");
                    layerViews[e].type = XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW;
                    layerViews[e].pose = steamVrFrame ? compose(fromWire(svr.info.renderHead), eyeToHead_[e]) : views[e].pose;
                    layerViews[e].fov = steamVrFrame ? helloFov_[e] : views[e].fov;
                    layerViews[e].subImage.swapchain = eyes_[e].handle;
                    layerViews[e].subImage.imageRect.offset = {0, 0};
                    layerViews[e].subImage.imageRect.extent = {eyes_[e].width, eyes_[e].height};
                    layerViews[e].subImage.imageArrayIndex = 0;
                }
                if (ok) {
                    layer.space = appSpace_;
                    layer.layerFlags = 0;
                    layer.viewCount = 2;
                    layer.views = layerViews;
                    layers[layerCount++] = reinterpret_cast<const XrCompositionLayerBaseHeader*>(&layer);
                    if (theater_.visible) {
                        quadLayer(screenLayer, theater_, false);
                        layers[layerCount++] = reinterpret_cast<const XrCompositionLayerBaseHeader*>(&screenLayer);
                    }
                    if (panel_.visible) {
                        quadLayer(panelLayer, panel_, true);
                        layers[layerCount++] = reinterpret_cast<const XrCompositionLayerBaseHeader*>(&panelLayer);
                    }
                    framesRendered_++;
                    if (framesRendered_ == 1) LOGI("first frame submitted (%u layers)", layerCount);
                    logStatus(views);
                }
            }
        }

        XrFrameEndInfo ei{XR_TYPE_FRAME_END_INFO};
        ei.displayTime = fs.predictedDisplayTime;
        ei.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
        ei.layerCount = layerCount;
        ei.layers = layerCount ? layers : nullptr;
        xrOk(xrEndFrame(session_, &ei), "xrEndFrame");
    }

    void quadLayer(XrCompositionLayerQuad& l, const SurfaceQuad& q, bool blend) {
        l.next = flipLayers_ ? &flipLayout_ : nullptr;
        l.layerFlags = blend ? XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT : 0;
        l.space = appSpace_;
        l.eyeVisibility = XR_EYE_VISIBILITY_BOTH;
        l.subImage.swapchain = q.handle;
        l.subImage.imageRect.offset = {0, 0};
        l.subImage.imageRect.extent = {q.width, q.height};
        l.subImage.imageArrayIndex = 0;
        l.pose = q.pose;
        l.size = q.size;
    }

    void logStatus(const XrView* views) {
        const auto now = std::chrono::steady_clock::now();
        if (now - lastStatus_ < std::chrono::seconds(2)) return;
        lastStatus_ = now;
        const XrVector3f hp{(views[0].pose.position.x + views[1].pose.position.x) * 0.5f,
                            (views[0].pose.position.y + views[1].pose.position.y) * 0.5f,
                            (views[0].pose.position.z + views[1].pose.position.z) * 0.5f};
        LOGI("frames=%d state=%d head=(%.2f, %.2f, %.2f) q=(%.2f, %.2f, %.2f, %.2f)", framesRendered_, static_cast<int>(state_), hp.x, hp.y, hp.z,
             views[0].pose.orientation.x, views[0].pose.orientation.y, views[0].pose.orientation.z, views[0].pose.orientation.w);
        for (int h = 0; h < 2; h++) {
            const Hand& hd = hands_[h];
            LOGI("%s: active=%d aim=%d pos=(%.2f, %.2f, %.2f) trigger=%.2f squeeze=%.2f stick=(%.2f, %.2f) panel=%d", h == 0 ? "left " : "right", hd.active,
                 hd.aimValid, hd.aim.position.x, hd.aim.position.y, hd.aim.position.z, hd.trigger, hd.squeeze, hd.stickX, hd.stickY, hd.onPanel);
        }
    }

    void loop() {
        LOGI("frame loop started");
        bool exitAsked = false;
        auto exitDeadline = std::chrono::steady_clock::time_point::max();
        while (true) {
            pollEvents();
            if (quit && !exitAsked) {
                exitAsked = true;
                releasePad();
                exitDeadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1500);
                if (sessionRunning_) {
                    xrOk(xrRequestExitSession(session_), "xrRequestExitSession");
                } else {
                    break;
                }
            }
            if (exitAsked && (!sessionRunning_ || std::chrono::steady_clock::now() > exitDeadline)) break;
            if (sessionRunning_) {
                renderFrame();
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }
        LOGI("frame loop finished after %d frames", framesRendered_);
    }

    void destroyQuad(SurfaceQuad& q) {
        if (q.surface && q.ownsSurfaceRef && env_) env_->DeleteGlobalRef(q.surface);
        if (q.handle) xrDestroySwapchain(q.handle);
        q = SurfaceQuad();
    }

    void teardown() {
        if (sessionRunning_ && session_ != XR_NULL_HANDLE) {
            xrEndSession(session_);
            sessionRunning_ = false;
        }
        destroyQuad(theater_);
        destroyQuad(panel_);
        link_.stop();
        linkUp_ = false;
        if (blitProgram_) glDeleteProgram(blitProgram_);
        if (blitVao_) glDeleteVertexArrays(1, &blitVao_);
        blitProgram_ = blitVao_ = 0;
        if (viewSpace_) xrDestroySpace(viewSpace_);
        viewSpace_ = XR_NULL_HANDLE;
        for (int h = 0; h < 2; h++) {
            if (aimSpace_[h]) xrDestroySpace(aimSpace_[h]);
            if (gripSpace_[h]) xrDestroySpace(gripSpace_[h]);
            aimSpace_[h] = gripSpace_[h] = XR_NULL_HANDLE;
        }
        if (actionSet_) xrDestroyActionSet(actionSet_);
        actionSet_ = XR_NULL_HANDLE;
        for (auto& eye : eyes_) {
            if (!eye.fbos.empty()) glDeleteFramebuffers(static_cast<GLsizei>(eye.fbos.size()), eye.fbos.data());
            if (eye.depth) glDeleteRenderbuffers(1, &eye.depth);
            if (eye.handle) xrDestroySwapchain(eye.handle);
            eye = EyeSwapchain();
        }
        if (staticVbo_) glDeleteBuffers(1, &staticVbo_);
        if (dynamicVbo_) glDeleteBuffers(1, &dynamicVbo_);
        if (program_) glDeleteProgram(program_);
        staticVbo_ = dynamicVbo_ = program_ = 0;
        if (appSpace_) xrDestroySpace(appSpace_);
        appSpace_ = XR_NULL_HANDLE;
        if (session_) xrDestroySession(session_);
        session_ = XR_NULL_HANDLE;
        if (instance_) xrDestroyInstance(instance_);
        instance_ = XR_NULL_HANDLE;
        if (display_ != EGL_NO_DISPLAY) {
            eglMakeCurrent(display_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
            if (surface_ != EGL_NO_SURFACE) eglDestroySurface(display_, surface_);
            if (context_ != EGL_NO_CONTEXT) eglDestroyContext(display_, context_);
            eglTerminate(display_);
        }
        display_ = EGL_NO_DISPLAY;
        surface_ = EGL_NO_SURFACE;
        context_ = EGL_NO_CONTEXT;
        LOGI("session torn down");
    }
};

std::mutex gLock;
VrSession* gSession = nullptr;
std::thread gThread;

void reapFinished() {
    if (gSession && gSession->finished) {
        if (gThread.joinable()) gThread.join();
        delete gSession;
        gSession = nullptr;
    }
}

}  // namespace

extern "C" {

JNIEXPORT jboolean JNICALL Java_com_droiddeck_launcher_vr_QuestVrNative_nativeStart(JNIEnv* env, jclass, jobject activity, jint mode, jint theaterW,
                                                                                  jint theaterH) {
    std::lock_guard<std::mutex> lock(gLock);
    reapFinished();
    if (gSession) {
        LOGW("previous session still closing; waiting for it");
        gSession->quit = true;
        if (gThread.joinable()) gThread.join();
        delete gSession;
        gSession = nullptr;
    }
    if (env->GetJavaVM(&gVm) != JNI_OK) return JNI_FALSE;
    auto* s = new VrSession(mode, theaterW > 0 ? theaterW : 1920, theaterH > 0 ? theaterH : 1080);
    jclass cls = env->GetObjectClass(activity);
    bool missing = false;
    auto method = [&](const char* name, const char* sig) -> jmethodID {
        jmethodID m = env->GetMethodID(cls, name, sig);
        if (!m || env->ExceptionCheck()) {
            env->ExceptionClear();
            LOGE("activity callback %s%s not found", name, sig);
            missing = true;
            return nullptr;
        }
        return m;
    };
    s->cb.onError = method("onVrError", "(Ljava/lang/String;)V");
    s->cb.onExit = method("onVrExit", "()V");
    s->cb.onTheaterSurface = method("onTheaterSurface", "(Landroid/view/Surface;II)V");
    s->cb.onPanelSurface = method("onPanelSurface", "(Landroid/view/Surface;II)V");
    s->cb.onSurfacesGone = method("onSurfacesGone", "()V");
    s->cb.onPanelPointer = method("onPanelPointer", "(IFF)V");
    s->cb.onPad = method("onPad", "([F)V");
    env->DeleteLocalRef(cls);
    if (missing) {
        delete s;
        return JNI_FALSE;
    }
    if (mode == MODE_THEATER) {
        jclass comp = env->FindClass("com/droiddeck/launcher/wayland/WaylandCompositor");
        if (comp && !env->ExceptionCheck()) {
            s->cb.compositorVsync = env->GetStaticMethodID(comp, "nativeVsync", "(J)V");
            if (env->ExceptionCheck() || !s->cb.compositorVsync) {
                env->ExceptionClear();
                s->cb.compositorVsync = nullptr;
            } else {
                s->cb.compositor = static_cast<jclass>(env->NewGlobalRef(comp));
            }
        } else {
            env->ExceptionClear();
        }
        if (comp) env->DeleteLocalRef(comp);
        if (!s->cb.compositor) LOGW("compositor vsync not found: the theater screen follows the 2D display's vsync");
    }
    s->cb.activity = env->NewGlobalRef(activity);
    gSession = s;
    gThread = std::thread([s] { s->run(); });
    return JNI_TRUE;
}

JNIEXPORT void JNICALL Java_com_droiddeck_launcher_vr_QuestVrNative_nativeStop(JNIEnv*, jclass) {
    std::lock_guard<std::mutex> lock(gLock);
    if (gSession) gSession->quit = true;
    reapFinished();
}

JNIEXPORT void JNICALL Java_com_droiddeck_launcher_vr_QuestVrNative_nativeRecenter(JNIEnv*, jclass) {
    std::lock_guard<std::mutex> lock(gLock);
    if (gSession) gSession->recenterAsked = true;
}

JNIEXPORT void JNICALL Java_com_droiddeck_launcher_vr_QuestVrNative_nativeSetScreen(JNIEnv*, jclass, jfloat distance, jfloat width) {
    std::lock_guard<std::mutex> lock(gLock);
    if (!gSession) return;
    gSession->screenDistance = distance;
    gSession->screenWidth = width;
}

}  // extern "C"
