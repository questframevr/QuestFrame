#pragma once

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl3.h>
#include <GLES2/gl2ext.h>
#include <android/hardware_buffer.h>
#include <android/log.h>
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <cstddef>
#include <cstring>
#include <vector>

#include "steamvr_protocol.h"

#define SVR_LOG(prio, ...) __android_log_print(prio, "QuestVR", __VA_ARGS__)

class SteamVrLink {
public:
    struct Frame {
        GLuint texture = 0;
        qdvr_frame info{};
    };

    bool start(EGLDisplay display) {
        display_ = display;
        getNativeClientBuffer_ = reinterpret_cast<PFNEGLGETNATIVECLIENTBUFFERANDROIDPROC>(eglGetProcAddress("eglGetNativeClientBufferANDROID"));
        createImage_ = reinterpret_cast<PFNEGLCREATEIMAGEKHRPROC>(eglGetProcAddress("eglCreateImageKHR"));
        destroyImage_ = reinterpret_cast<PFNEGLDESTROYIMAGEKHRPROC>(eglGetProcAddress("eglDestroyImageKHR"));
        imageTarget_ = reinterpret_cast<PFNGLEGLIMAGETARGETTEXTURE2DOESPROC>(eglGetProcAddress("glEGLImageTargetTexture2DOES"));
        if (void* nw = dlopen("libnativewindow.so", RTLD_NOW | RTLD_LOCAL))
            getNativeHandle_ = reinterpret_cast<const NativeHandle* (*)(const AHardwareBuffer*)>(dlsym(nw, "AHardwareBuffer_getNativeHandle"));
        if (!getNativeClientBuffer_ || !createImage_ || !destroyImage_ || !imageTarget_ || !getNativeHandle_) {
            SVR_LOG(ANDROID_LOG_ERROR, "the GL/AHardwareBuffer functions SteamVR frames need are missing");
            return false;
        }
        listen_ = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        sockaddr_un a{};
        a.sun_family = AF_UNIX;
        const size_t n = strlen(QDVR_SOCKET_NAME);
        memcpy(a.sun_path + 1, QDVR_SOCKET_NAME, n);
        if (listen_ < 0 || bind(listen_, reinterpret_cast<sockaddr*>(&a), socklen_t(offsetof(sockaddr_un, sun_path) + 1 + n)) != 0 ||
            listen(listen_, 1) != 0) {
            SVR_LOG(ANDROID_LOG_ERROR, "the SteamVR socket @%s could not be opened: %s", QDVR_SOCKET_NAME, strerror(errno));
            if (listen_ >= 0) close(listen_);
            listen_ = -1;
            return false;
        }
        SVR_LOG(ANDROID_LOG_INFO, "waiting for SteamVR's driver on @%s", QDVR_SOCKET_NAME);
        return true;
    }

    void stop() {
        dropConnection();
        if (listen_ >= 0) close(listen_);
        listen_ = -1;
    }

    bool connected() const { return conn_ >= 0; }
    bool helloSent() const { return helloSent_; }

    void sendHello(const qdvr_hello& h) {
        if (conn_ < 0) return;
        helloSent_ = send(conn_, &h, sizeof(h), MSG_NOSIGNAL | MSG_DONTWAIT) == ssize_t(sizeof(h));
    }

    void sendTracking(const qdvr_tracking& t) {
        if (conn_ >= 0 && send(conn_, &t, sizeof(t), MSG_NOSIGNAL | MSG_DONTWAIT) < 0 && errno != EAGAIN) dropConnection();
    }

    void poll(std::vector<qdvr_haptic>& haptics) {
        if (listen_ < 0) return;
        if (conn_ < 0) {
            const int c = accept4(listen_, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
            if (c < 0) return;
            conn_ = c;
            helloSent_ = false;
            SVR_LOG(ANDROID_LOG_INFO, "SteamVR's driver connected");
        }
        alignas(8) uint8_t buf[512];
        for (;;) {
            const ssize_t n = recv(conn_, buf, sizeof(buf), MSG_DONTWAIT);
            if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
                SVR_LOG(ANDROID_LOG_INFO, "SteamVR's driver disconnected");
                dropConnection();
                return;
            }
            if (n < 0) return;
            if (n < 4) continue;
            uint32_t type;
            memcpy(&type, buf, 4);
            if (type == QDVR_MSG_NEED_BUFFERS && size_t(n) >= sizeof(qdvr_need_buffers)) {
                qdvr_need_buffers nb;
                memcpy(&nb, buf, sizeof(nb));
                allocate(nb.eyeWidth, nb.eyeHeight);
            } else if (type == QDVR_MSG_FRAME && size_t(n) >= sizeof(qdvr_frame)) {
                qdvr_frame f;
                memcpy(&f, buf, sizeof(f));
                if (f.generation == generation_ && f.index < buffers_.size()) {
                    frame_.info = f;
                    frame_.texture = buffers_[f.index].texture;
                    haveFrame_ = true;
                    if (++frames_ == 1) SVR_LOG(ANDROID_LOG_INFO, "first SteamVR frame");
                }
            } else if (type == QDVR_MSG_HAPTIC && size_t(n) >= sizeof(qdvr_haptic)) {
                qdvr_haptic h;
                memcpy(&h, buf, sizeof(h));
                haptics.push_back(h);
            }
        }
    }

    bool frame(Frame& out) const {
        if (!haveFrame_) return false;
        out = frame_;
        return true;
    }

    uint64_t framesReceived() const { return frames_; }

private:
    struct NativeHandle {
        int version, numFds, numInts;
        int data[0];
    };
    struct Buffer {
        AHardwareBuffer* ahb = nullptr;
        EGLImageKHR image = EGL_NO_IMAGE_KHR;
        GLuint texture = 0;
    };

    EGLDisplay display_ = EGL_NO_DISPLAY;
    PFNEGLGETNATIVECLIENTBUFFERANDROIDPROC getNativeClientBuffer_ = nullptr;
    PFNEGLCREATEIMAGEKHRPROC createImage_ = nullptr;
    PFNEGLDESTROYIMAGEKHRPROC destroyImage_ = nullptr;
    PFNGLEGLIMAGETARGETTEXTURE2DOESPROC imageTarget_ = nullptr;
    const NativeHandle* (*getNativeHandle_)(const AHardwareBuffer*) = nullptr;
    int listen_ = -1, conn_ = -1;
    bool helloSent_ = false;
    std::vector<Buffer> buffers_;
    uint32_t generation_ = 0;
    Frame frame_;
    bool haveFrame_ = false;
    uint64_t frames_ = 0;

    void dropConnection() {
        if (conn_ >= 0) close(conn_);
        conn_ = -1;
        helloSent_ = false;
        freeBuffers();
    }

    void freeBuffers() {
        for (Buffer& b : buffers_) {
            if (b.texture) glDeleteTextures(1, &b.texture);
            if (b.image != EGL_NO_IMAGE_KHR) destroyImage_(display_, b.image);
            if (b.ahb) AHardwareBuffer_release(b.ahb);
        }
        buffers_.clear();
        haveFrame_ = false;
    }

    void allocate(uint32_t eyeW, uint32_t eyeH) {
        freeBuffers();
        generation_++;
        AHardwareBuffer_Desc d{};
        d.width = eyeW * 2;
        d.height = eyeH;
        d.layers = 1;
        d.format = AHARDWAREBUFFER_FORMAT_R8G8B8A8_UNORM;
        d.usage = AHARDWAREBUFFER_USAGE_GPU_SAMPLED_IMAGE | AHARDWAREBUFFER_USAGE_GPU_COLOR_OUTPUT | AHARDWAREBUFFER_USAGE_CPU_READ_RARELY;
        qdvr_buffers msg{};
        msg.type = QDVR_MSG_BUFFERS;
        msg.generation = generation_;
        msg.width = d.width;
        msg.height = d.height;
        msg.drmFourcc = 0x34324241;
        int fds[QDVR_MAX_BUFFERS];
        for (int i = 0; i < QDVR_MAX_BUFFERS; i++) {
            Buffer b;
            if (AHardwareBuffer_allocate(&d, &b.ahb) != 0 || !b.ahb) {
                SVR_LOG(ANDROID_LOG_ERROR, "AHardwareBuffer_allocate %ux%u failed", d.width, d.height);
                freeBuffers();
                return;
            }
            AHardwareBuffer_Desc got;
            AHardwareBuffer_describe(b.ahb, &got);
            const NativeHandle* h = getNativeHandle_(b.ahb);
            if (!h || h->numFds < 1) {
                SVR_LOG(ANDROID_LOG_ERROR, "an AHardwareBuffer has no dma-buf");
                AHardwareBuffer_release(b.ahb);
                freeBuffers();
                return;
            }
            fds[i] = h->data[0];
            const bool ubwc = h->numInts >= 2 && h->data[h->numFds] == 0x676d736d && (h->data[h->numFds + 1] & 0x08000000);
            msg.strideBytes = got.stride * 4;
            msg.modifier = ubwc ? ((0x05ull << 56) | 1) : 0;
            const off_t size = lseek(fds[i], 0, SEEK_END);
            msg.sizeBytes = size > 0 ? uint64_t(size) : 0;

            const EGLint attrs[] = {EGL_IMAGE_PRESERVED_KHR, EGL_TRUE, EGL_NONE};
            b.image = createImage_(display_, EGL_NO_CONTEXT, EGL_NATIVE_BUFFER_ANDROID, getNativeClientBuffer_(b.ahb), attrs);
            glGenTextures(1, &b.texture);
            glBindTexture(GL_TEXTURE_2D, b.texture);
            if (b.image != EGL_NO_IMAGE_KHR) imageTarget_(GL_TEXTURE_2D, static_cast<GLeglImageOES>(b.image));
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glBindTexture(GL_TEXTURE_2D, 0);
            if (b.image == EGL_NO_IMAGE_KHR) SVR_LOG(ANDROID_LOG_ERROR, "an AHardwareBuffer could not become a GL texture (0x%x)", eglGetError());
            buffers_.push_back(b);
            if (i == 0)
                SVR_LOG(ANDROID_LOG_INFO, "SteamVR frame buffers: %ux%u, stride %u px, %s, %llu bytes", got.width, got.height, got.stride,
                    ubwc ? "UBWC" : "linear", static_cast<unsigned long long>(msg.sizeBytes));
        }
        msg.count = QDVR_MAX_BUFFERS;
        char cbuf[CMSG_SPACE(sizeof(fds))];
        memset(cbuf, 0, sizeof(cbuf));
        iovec iov{&msg, sizeof(msg)};
        msghdr mh{};
        mh.msg_iov = &iov;
        mh.msg_iovlen = 1;
        mh.msg_control = cbuf;
        mh.msg_controllen = sizeof(cbuf);
        cmsghdr* c = CMSG_FIRSTHDR(&mh);
        c->cmsg_level = SOL_SOCKET;
        c->cmsg_type = SCM_RIGHTS;
        c->cmsg_len = CMSG_LEN(sizeof(fds));
        memcpy(CMSG_DATA(c), fds, sizeof(fds));
        if (sendmsg(conn_, &mh, MSG_NOSIGNAL) != ssize_t(sizeof(msg))) SVR_LOG(ANDROID_LOG_ERROR, "sending the frame buffers failed: %s", strerror(errno));
    }
};
