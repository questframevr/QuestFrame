//
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdarg.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <cstdio>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#define VK_NO_PROTOTYPES
#include <vulkan/vulkan.h>

#include "openvr_driver.h"
#include "steamvr_protocol.h"

namespace {

// --- Logging ----------------------------------------------------------------------------------
void logf(const char* fmt, ...) {
    char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf) - 2, fmt, ap);
    va_end(ap);
    size_t n = strlen(buf);
    if (n == 0 || buf[n - 1] != '\n') {
        buf[n] = '\n';
        buf[n + 1] = 0;
    }
    if (vr::VRDriverLog()) vr::VRDriverLog()->Log(buf);
    fprintf(stderr, "[questdeck] %s", buf);
}

// --- Math ------------------------------------------------------------------------------------
vr::HmdQuaternion_t quat(const qdvr_pose& p) { return {p.qw, p.qx, p.qy, p.qz}; }

vr::HmdMatrix34_t matFromPose(const qdvr_pose& p) {
    const double w = p.qw, x = p.qx, y = p.qy, z = p.qz;
    vr::HmdMatrix34_t m{};
    m.m[0][0] = float(1 - 2 * (y * y + z * z));
    m.m[0][1] = float(2 * (x * y - z * w));
    m.m[0][2] = float(2 * (x * z + y * w));
    m.m[1][0] = float(2 * (x * y + z * w));
    m.m[1][1] = float(1 - 2 * (x * x + z * z));
    m.m[1][2] = float(2 * (y * z - x * w));
    m.m[2][0] = float(2 * (x * z - y * w));
    m.m[2][1] = float(2 * (y * z + x * w));
    m.m[2][2] = float(1 - 2 * (x * x + y * y));
    m.m[0][3] = p.px;
    m.m[1][3] = p.py;
    m.m[2][3] = p.pz;
    return m;
}

qdvr_pose poseFromMat(const vr::HmdMatrix34_t& m) {
    qdvr_pose p{};
    const float tr = m.m[0][0] + m.m[1][1] + m.m[2][2];
    float w, x, y, z;
    if (tr > 0) {
        const float s = std::sqrt(tr + 1.0f) * 2;
        w = 0.25f * s;
        x = (m.m[2][1] - m.m[1][2]) / s;
        y = (m.m[0][2] - m.m[2][0]) / s;
        z = (m.m[1][0] - m.m[0][1]) / s;
    } else if (m.m[0][0] > m.m[1][1] && m.m[0][0] > m.m[2][2]) {
        const float s = std::sqrt(1.0f + m.m[0][0] - m.m[1][1] - m.m[2][2]) * 2;
        w = (m.m[2][1] - m.m[1][2]) / s;
        x = 0.25f * s;
        y = (m.m[0][1] + m.m[1][0]) / s;
        z = (m.m[0][2] + m.m[2][0]) / s;
    } else if (m.m[1][1] > m.m[2][2]) {
        const float s = std::sqrt(1.0f + m.m[1][1] - m.m[0][0] - m.m[2][2]) * 2;
        w = (m.m[0][2] - m.m[2][0]) / s;
        x = (m.m[0][1] + m.m[1][0]) / s;
        y = 0.25f * s;
        z = (m.m[1][2] + m.m[2][1]) / s;
    } else {
        const float s = std::sqrt(1.0f + m.m[2][2] - m.m[0][0] - m.m[1][1]) * 2;
        w = (m.m[1][0] - m.m[0][1]) / s;
        x = (m.m[0][2] + m.m[2][0]) / s;
        y = (m.m[1][2] + m.m[2][1]) / s;
        z = 0.25f * s;
    }
    p.qw = w;
    p.qx = x;
    p.qy = y;
    p.qz = z;
    p.px = m.m[0][3];
    p.py = m.m[1][3];
    p.pz = m.m[2][3];
    return p;
}

vr::DriverPose_t driverPose(const qdvr_pose& p, bool valid) {
    vr::DriverPose_t d{};
    d.qWorldFromDriverRotation = {1, 0, 0, 0};
    d.qDriverFromHeadRotation = {1, 0, 0, 0};
    d.qRotation = valid ? quat(p) : vr::HmdQuaternion_t{1, 0, 0, 0};
    d.vecPosition[0] = p.px;
    d.vecPosition[1] = p.py;
    d.vecPosition[2] = p.pz;
    d.poseIsValid = valid;
    d.deviceIsConnected = true;
    d.result = valid ? vr::TrackingResult_Running_OK : vr::TrackingResult_Running_OutOfRange;
    return d;
}

class Bridge {
public:
    std::function<void(const qdvr_tracking&)> onTracking;

    void start() {
        reader_ = std::thread([this] { run(); });
    }
    void stop() {
        quit_ = true;
        int fd = fd_.exchange(-1);
        if (fd >= 0) {
            shutdown(fd, SHUT_RDWR);
            close(fd);
        }
        if (reader_.joinable()) reader_.join();
    }

    bool waitHello(int timeoutMs, qdvr_hello& out) {
        std::unique_lock<std::mutex> l(m_);
        cv_.wait_for(l, std::chrono::milliseconds(timeoutMs), [&] { return haveHello_; });
        if (haveHello_) out = hello_;
        return haveHello_;
    }
    bool hello(qdvr_hello& out) {
        std::lock_guard<std::mutex> l(m_);
        if (haveHello_) out = hello_;
        return haveHello_;
    }
    bool connected() const { return fd_ >= 0; }

    bool latestTracking(qdvr_tracking& out) {
        std::lock_guard<std::mutex> l(m_);
        if (!haveTracking_) return false;
        out = tracking_;
        return true;
    }
    uint32_t trackingSeq() {
        std::lock_guard<std::mutex> l(m_);
        return tracking_.seq;
    }
    void waitTrackingAfter(uint32_t seq, int timeoutMs) {
        std::unique_lock<std::mutex> l(m_);
        cv_.wait_for(l, std::chrono::milliseconds(timeoutMs), [&] { return tracking_.seq != seq || !haveTracking_; });
    }

    bool takeBuffers(qdvr_buffers& out, std::vector<int>& fds) {
        std::lock_guard<std::mutex> l(m_);
        if (!haveBuffers_) return false;
        out = buffers_;
        fds.swap(bufferFds_);
        bufferFds_.clear();
        haveBuffers_ = false;
        return true;
    }

    bool send(const void* msg, size_t len) {
        int fd = fd_;
        if (fd < 0) return false;
        std::lock_guard<std::mutex> l(sendLock_);
        return ::send(fd, msg, len, MSG_NOSIGNAL) == ssize_t(len);
    }

private:
    std::atomic<bool> quit_{false};
    std::atomic<int> fd_{-1};
    std::thread reader_;
    std::mutex m_, sendLock_;
    std::condition_variable cv_;
    bool haveHello_ = false, haveTracking_ = false, haveBuffers_ = false;
    qdvr_hello hello_{};
    qdvr_tracking tracking_{};
    qdvr_buffers buffers_{};
    std::vector<int> bufferFds_;

    int connectOnce() {
        int fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
        if (fd < 0) return -1;
        sockaddr_un a{};
        a.sun_family = AF_UNIX;
        const size_t n = strlen(QDVR_SOCKET_NAME);
        memcpy(a.sun_path + 1, QDVR_SOCKET_NAME, n);
        if (connect(fd, reinterpret_cast<sockaddr*>(&a), socklen_t(offsetof(sockaddr_un, sun_path) + 1 + n)) != 0) {
            close(fd);
            return -1;
        }
        return fd;
    }

    void run() {
        bool saidWaiting = false;
        while (!quit_) {
            int fd = connectOnce();
            if (fd < 0) {
                if (!saidWaiting) logf("waiting for the QuestDeck app (socket @%s)", QDVR_SOCKET_NAME);
                saidWaiting = true;
                std::this_thread::sleep_for(std::chrono::milliseconds(250));
                continue;
            }
            saidWaiting = false;
            logf("connected to the QuestDeck app");
            fd_ = fd;
            readLoop(fd);
            if (fd_.exchange(-1) == fd) close(fd);
            {
                std::lock_guard<std::mutex> l(m_);
                haveTracking_ = false;
                for (int b : bufferFds_) close(b);
                bufferFds_.clear();
                haveBuffers_ = false;
            }
            cv_.notify_all();
            if (!quit_) logf("the QuestDeck app went away");
        }
    }

    void readLoop(int fd) {
        alignas(8) uint8_t buf[4096];
        char cbuf[CMSG_SPACE(sizeof(int) * QDVR_MAX_BUFFERS)];
        while (!quit_) {
            iovec iov{buf, sizeof(buf)};
            msghdr mh{};
            mh.msg_iov = &iov;
            mh.msg_iovlen = 1;
            mh.msg_control = cbuf;
            mh.msg_controllen = sizeof(cbuf);
            const ssize_t n = recvmsg(fd, &mh, MSG_CMSG_CLOEXEC);
            if (n <= 0) return;
            std::vector<int> fds;
            for (cmsghdr* c = CMSG_FIRSTHDR(&mh); c; c = CMSG_NXTHDR(&mh, c)) {
                if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SCM_RIGHTS) {
                    const int count = int((c->cmsg_len - CMSG_LEN(0)) / sizeof(int));
                    for (int i = 0; i < count; i++) {
                        int f;
                        memcpy(&f, CMSG_DATA(c) + i * sizeof(int), sizeof(int));
                        fds.push_back(f);
                    }
                }
            }
            if (n < 4) continue;
            uint32_t type;
            memcpy(&type, buf, 4);
            if (type == QDVR_MSG_HELLO && size_t(n) >= sizeof(qdvr_hello)) {
                std::lock_guard<std::mutex> l(m_);
                memcpy(&hello_, buf, sizeof(hello_));
                haveHello_ = true;
                logf("headset: %ux%u per eye at %.0f Hz", hello_.eyeWidth, hello_.eyeHeight, hello_.refreshHz);
            } else if (type == QDVR_MSG_TRACKING && size_t(n) >= sizeof(qdvr_tracking)) {
                qdvr_tracking t;
                memcpy(&t, buf, sizeof(t));
                {
                    std::lock_guard<std::mutex> l(m_);
                    tracking_ = t;
                    haveTracking_ = true;
                }
                if (onTracking) onTracking(t);
            } else if (type == QDVR_MSG_BUFFERS && size_t(n) >= sizeof(qdvr_buffers)) {
                std::lock_guard<std::mutex> l(m_);
                for (int b : bufferFds_) close(b);
                memcpy(&buffers_, buf, sizeof(buffers_));
                bufferFds_.swap(fds);
                haveBuffers_ = true;
                logf("frame buffers from the app: %u of %ux%u (stride %u, modifier %#llx)", buffers_.count, buffers_.width,
                     buffers_.height, buffers_.strideBytes, (unsigned long long)buffers_.modifier);
            }
            for (int f : fds) close(f);
            cv_.notify_all();
        }
    }
};

uint32_t unormTwinOf(uint32_t f, bool& srgb) {
    srgb = true;
    switch (f) {
        case VK_FORMAT_R8G8B8A8_SRGB: return VK_FORMAT_R8G8B8A8_UNORM;
        case VK_FORMAT_B8G8R8A8_SRGB: return VK_FORMAT_B8G8R8A8_UNORM;
        case VK_FORMAT_A8B8G8R8_SRGB_PACK32: return VK_FORMAT_A8B8G8R8_UNORM_PACK32;
        default: srgb = false; return f;
    }
}

class Copier {
public:
    ~Copier() { destroy(); }

    bool init() {
        lib_ = dlopen("libvulkan.so.1", RTLD_NOW | RTLD_LOCAL);
        if (!lib_) {
            logf("Vulkan: %s", dlerror());
            return false;
        }
        gipa_ = reinterpret_cast<PFN_vkGetInstanceProcAddr>(dlsym(lib_, "vkGetInstanceProcAddr"));
        if (!gipa_) return false;
#define GI(n) n = reinterpret_cast<PFN_##n>(gipa_(VK_NULL_HANDLE, #n))
        GI(vkCreateInstance);
#undef GI
        VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO};
        app.pApplicationName = "driver_questdeck";
        app.apiVersion = VK_API_VERSION_1_1;
        VkInstanceCreateInfo ici{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
        ici.pApplicationInfo = &app;
        if (vkCreateInstance(&ici, nullptr, &instance_) != VK_SUCCESS) {
            logf("Vulkan: vkCreateInstance failed");
            return false;
        }
#define IN(n) n = reinterpret_cast<PFN_##n>(gipa_(instance_, #n))
        IN(vkDestroyInstance);
        IN(vkEnumeratePhysicalDevices);
        IN(vkGetPhysicalDeviceProperties);
        IN(vkGetPhysicalDeviceQueueFamilyProperties);
        IN(vkGetPhysicalDeviceMemoryProperties);
        IN(vkEnumerateDeviceExtensionProperties);
        IN(vkCreateDevice);
        IN(vkGetDeviceProcAddr);
#undef IN
        uint32_t n = 0;
        vkEnumeratePhysicalDevices(instance_, &n, nullptr);
        if (!n) {
            logf("Vulkan: no GPU");
            return false;
        }
        std::vector<VkPhysicalDevice> pds(n);
        vkEnumeratePhysicalDevices(instance_, &n, pds.data());
        phys_ = pds[0];
        VkPhysicalDeviceProperties props;
        vkGetPhysicalDeviceProperties(phys_, &props);
        vkGetPhysicalDeviceMemoryProperties(phys_, &memProps_);
        uint32_t qn = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(phys_, &qn, nullptr);
        std::vector<VkQueueFamilyProperties> qf(qn);
        vkGetPhysicalDeviceQueueFamilyProperties(phys_, &qn, qf.data());
        family_ = UINT32_MAX;
        for (uint32_t i = 0; i < qn; i++)
            if (qf[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) {
                family_ = i;
                break;
            }
        if (family_ == UINT32_MAX) return false;

        uint32_t en = 0;
        vkEnumerateDeviceExtensionProperties(phys_, nullptr, &en, nullptr);
        std::vector<VkExtensionProperties> exts(en);
        vkEnumerateDeviceExtensionProperties(phys_, nullptr, &en, exts.data());
        auto has = [&](const char* name) {
            for (auto& e : exts)
                if (!strcmp(e.extensionName, name)) return true;
            return false;
        };
        std::vector<const char*> want;
        for (const char* e : {VK_KHR_EXTERNAL_MEMORY_FD_EXTENSION_NAME, VK_EXT_EXTERNAL_MEMORY_DMA_BUF_EXTENSION_NAME,
                              VK_EXT_IMAGE_DRM_FORMAT_MODIFIER_EXTENSION_NAME, VK_KHR_IMAGE_FORMAT_LIST_EXTENSION_NAME,
                              VK_EXT_QUEUE_FAMILY_FOREIGN_EXTENSION_NAME}) {
            if (has(e))
                want.push_back(e);
            else
                logf("Vulkan: %s missing", e);
        }
        modifiers_ = has(VK_EXT_IMAGE_DRM_FORMAT_MODIFIER_EXTENSION_NAME);
        foreign_ = has(VK_EXT_QUEUE_FAMILY_FOREIGN_EXTENSION_NAME);
        float prio = 1.0f;
        VkDeviceQueueCreateInfo qci{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
        qci.queueFamilyIndex = family_;
        qci.queueCount = 1;
        qci.pQueuePriorities = &prio;
        VkDeviceCreateInfo dci{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
        dci.queueCreateInfoCount = 1;
        dci.pQueueCreateInfos = &qci;
        dci.enabledExtensionCount = uint32_t(want.size());
        dci.ppEnabledExtensionNames = want.data();
        if (vkCreateDevice(phys_, &dci, nullptr, &device_) != VK_SUCCESS) {
            logf("Vulkan: vkCreateDevice failed");
            return false;
        }
#define DV(n) n = reinterpret_cast<PFN_##n>(vkGetDeviceProcAddr(device_, #n))
        DV(vkDestroyDevice);
        DV(vkGetDeviceQueue);
        DV(vkCreateImage);
        DV(vkDestroyImage);
        DV(vkGetImageMemoryRequirements);
        DV(vkAllocateMemory);
        DV(vkFreeMemory);
        DV(vkBindImageMemory);
        DV(vkGetMemoryFdPropertiesKHR);
        DV(vkCreateCommandPool);
        DV(vkDestroyCommandPool);
        DV(vkAllocateCommandBuffers);
        DV(vkBeginCommandBuffer);
        DV(vkEndCommandBuffer);
        DV(vkCmdPipelineBarrier);
        DV(vkCmdBlitImage);
        DV(vkCmdCopyImage);
        DV(vkCmdClearColorImage);
        DV(vkQueueSubmit);
        DV(vkCreateFence);
        DV(vkDestroyFence);
        DV(vkWaitForFences);
        DV(vkResetFences);
        DV(vkDeviceWaitIdle);
        DV(vkResetCommandBuffer);
#undef DV
        vkGetDeviceQueue(device_, family_, 0, &queue_);
        VkCommandPoolCreateInfo pci{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
        pci.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        pci.queueFamilyIndex = family_;
        vkCreateCommandPool(device_, &pci, nullptr, &pool_);
        VkCommandBufferAllocateInfo cai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
        cai.commandPool = pool_;
        cai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        cai.commandBufferCount = 1;
        vkAllocateCommandBuffers(device_, &cai, &cmd_);
        VkFenceCreateInfo fci{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
        vkCreateFence(device_, &fci, nullptr, &fence_);
        logf("Vulkan: %s, copies on queue family %u%s", props.deviceName, family_, modifiers_ ? "" : " (no DRM format modifiers)");
        return true;
    }

    struct Image {
        VkImage image = VK_NULL_HANDLE;
        VkDeviceMemory memory = VK_NULL_HANDLE;
        VkImageLayout layout = VK_IMAGE_LAYOUT_UNDEFINED;
        uint32_t width = 0, height = 0;
    };

    bool importEye(int fd, uint32_t vkFormat, uint32_t w, uint32_t h, Image& out) {
        const char* env = getenv("QUESTDECK_EYE_IMPORT");
        const std::string mode = env ? env : "ubwc";
        const bool modifier = mode == "ubwc" || mode == "linear";
        bool srgb = false;
        const VkFormat formats[2] = {VkFormat(vkFormat), VkFormat(unormTwinOf(vkFormat, srgb))};
        VkImageFormatListCreateInfo list{VK_STRUCTURE_TYPE_IMAGE_FORMAT_LIST_CREATE_INFO};
        list.viewFormatCount = formats[0] == formats[1] ? 1 : 2;
        list.pViewFormats = formats;
        const uint64_t mods[1] = {mode == "ubwc" ? ((0x05ull << 56) | 1) : 0ull};
        VkImageDrmFormatModifierListCreateInfoEXT modList{VK_STRUCTURE_TYPE_IMAGE_DRM_FORMAT_MODIFIER_LIST_CREATE_INFO_EXT};
        modList.drmFormatModifierCount = 1;
        modList.pDrmFormatModifiers = mods;
        VkExternalMemoryImageCreateInfo ext{VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_IMAGE_CREATE_INFO};
        ext.handleTypes = modifier ? VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT : VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT;
        if (modifier) {
            modList.pNext = &list;
            ext.pNext = &modList;
        } else if (mode == "list") {
            ext.pNext = &list;
        }
        VkImageCreateInfo ci{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
        ci.pNext = &ext;
        ci.flags = VK_IMAGE_CREATE_MUTABLE_FORMAT_BIT;
        ci.imageType = VK_IMAGE_TYPE_2D;
        ci.format = VkFormat(vkFormat);
        ci.extent = {w, h, 1};
        ci.mipLevels = 1;
        ci.arrayLayers = 1;
        ci.samples = VK_SAMPLE_COUNT_1_BIT;
        ci.tiling = modifier ? VK_IMAGE_TILING_DRM_FORMAT_MODIFIER_EXT : VK_IMAGE_TILING_OPTIMAL;
        ci.usage = VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT |
                   VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT;
        ci.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        VkResult cr = vkCreateImage(device_, &ci, nullptr, &out.image);
        if (cr != VK_SUCCESS && (ci.usage & VK_IMAGE_USAGE_STORAGE_BIT)) {
            ci.usage &= ~VK_IMAGE_USAGE_STORAGE_BIT;
            cr = vkCreateImage(device_, &ci, nullptr, &out.image);
        }
        if (cr != VK_SUCCESS) {
            logf("Vulkan: eye image (format %u, %ux%u, %s) could not be created: %d", vkFormat, w, h, mode.c_str(), cr);
            close(fd);
            return false;
        }
        VkMemoryRequirements req;
        vkGetImageMemoryRequirements(device_, out.image, &req);
        uint32_t bits = req.memoryTypeBits;
        VkMemoryFdPropertiesKHR fp{VK_STRUCTURE_TYPE_MEMORY_FD_PROPERTIES_KHR};
        if (modifier && vkGetMemoryFdPropertiesKHR &&
            vkGetMemoryFdPropertiesKHR(device_, VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT, fd, &fp) == VK_SUCCESS)
            bits &= fp.memoryTypeBits;
        VkImportMemoryFdInfoKHR imp{VK_STRUCTURE_TYPE_IMPORT_MEMORY_FD_INFO_KHR};
        imp.handleType = VkExternalMemoryHandleTypeFlagBits(ext.handleTypes);
        imp.fd = fd;
        VkMemoryDedicatedAllocateInfo ded{VK_STRUCTURE_TYPE_MEMORY_DEDICATED_ALLOCATE_INFO};
        ded.pNext = &imp;
        ded.image = out.image;
        VkMemoryAllocateInfo ai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        ai.pNext = &ded;
        const off_t fdSize = lseek(fd, 0, SEEK_END);
        ai.allocationSize = fdSize > off_t(req.size) ? uint64_t(fdSize) : req.size;
        ai.memoryTypeIndex = memoryType(bits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        VkResult ar = vkAllocateMemory(device_, &ai, nullptr, &out.memory);
        if (ar != VK_SUCCESS) {
            logf("Vulkan: importing an eye texture (%s) failed: %d", mode.c_str(), ar);
            close(fd);
            vkDestroyImage(device_, out.image, nullptr);
            out = Image();
            return false;
        }
        vkBindImageMemory(device_, out.image, out.memory, 0);
        out.width = w;
        out.height = h;
        static bool said = false;
        if (!said) logf("eye textures imported as %s (needs %llu bytes, fd has %lld)", mode.c_str(), (unsigned long long)req.size, (long long)fdSize);
        said = true;
        return true;
    }

    bool importTarget(int fd, const qdvr_buffers& b, Image& out) {
        VkExternalMemoryImageCreateInfo ext{VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_IMAGE_CREATE_INFO};
        ext.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT;
        VkSubresourceLayout plane{};
        plane.offset = 0;
        plane.rowPitch = b.strideBytes;
        VkImageDrmFormatModifierExplicitCreateInfoEXT mod{VK_STRUCTURE_TYPE_IMAGE_DRM_FORMAT_MODIFIER_EXPLICIT_CREATE_INFO_EXT};
        mod.drmFormatModifier = b.modifier;
        mod.drmFormatModifierPlaneCount = 1;
        mod.pPlaneLayouts = &plane;
        VkImageCreateInfo ci{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
        ci.imageType = VK_IMAGE_TYPE_2D;
        ci.format = VK_FORMAT_R8G8B8A8_UNORM;
        ci.extent = {b.width, b.height, 1};
        ci.mipLevels = 1;
        ci.arrayLayers = 1;
        ci.samples = VK_SAMPLE_COUNT_1_BIT;
        ci.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT;
        ci.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        if (modifiers_) {
            mod.pNext = &ext;
            ci.pNext = &mod;
            ci.tiling = VK_IMAGE_TILING_DRM_FORMAT_MODIFIER_EXT;
        } else {
            ci.pNext = &ext;
            ci.tiling = VK_IMAGE_TILING_LINEAR;
        }
        if (vkCreateImage(device_, &ci, nullptr, &out.image) != VK_SUCCESS) {
            logf("Vulkan: frame buffer image (%ux%u, stride %u) could not be created", b.width, b.height, b.strideBytes);
            close(fd);
            return false;
        }
        VkMemoryRequirements req;
        vkGetImageMemoryRequirements(device_, out.image, &req);
        VkMemoryFdPropertiesKHR fp{VK_STRUCTURE_TYPE_MEMORY_FD_PROPERTIES_KHR};
        uint32_t bits = req.memoryTypeBits;
        if (vkGetMemoryFdPropertiesKHR && vkGetMemoryFdPropertiesKHR(device_, VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT, fd, &fp) == VK_SUCCESS)
            bits &= fp.memoryTypeBits;
        VkImportMemoryFdInfoKHR imp{VK_STRUCTURE_TYPE_IMPORT_MEMORY_FD_INFO_KHR};
        imp.handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT;
        imp.fd = fd;
        VkMemoryDedicatedAllocateInfo ded{VK_STRUCTURE_TYPE_MEMORY_DEDICATED_ALLOCATE_INFO};
        ded.pNext = &imp;
        ded.image = out.image;
        VkMemoryAllocateInfo ai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        ai.pNext = &ded;
        ai.allocationSize = b.sizeBytes > req.size ? b.sizeBytes : req.size;
        ai.memoryTypeIndex = memoryType(bits, 0);
        if (vkAllocateMemory(device_, &ai, nullptr, &out.memory) != VK_SUCCESS) {
            logf("Vulkan: importing a frame buffer failed (size %llu, needs %llu)", (unsigned long long)b.sizeBytes,
                 (unsigned long long)req.size);
            close(fd);
            vkDestroyImage(device_, out.image, nullptr);
            out = Image();
            return false;
        }
        vkBindImageMemory(device_, out.image, out.memory, 0);
        out.width = b.width;
        out.height = b.height;
        return true;
    }

    void release(Image& img) {
        if (img.image) vkDestroyImage(device_, img.image, nullptr);
        if (img.memory) vkFreeMemory(device_, img.memory, nullptr);
        img = Image();
    }

    bool copy(Image& left, const vr::VRTextureBounds_t& lb, Image& right, const vr::VRTextureBounds_t& rb, Image& dst) {
        vkResetCommandBuffer(cmd_, 0);
        VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
        bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        vkBeginCommandBuffer(cmd_, &bi);
        VkImageMemoryBarrier pre[3];
        uint32_t np = 0;
        auto barrier = [&](Image& img, VkImageLayout to, VkAccessFlags access, bool fromForeign) {
            VkImageMemoryBarrier b{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
            b.srcAccessMask = 0;
            b.dstAccessMask = access;
            b.oldLayout = img.layout;
            b.newLayout = to;
            b.srcQueueFamilyIndex = (fromForeign && foreign_) ? VK_QUEUE_FAMILY_FOREIGN_EXT : VK_QUEUE_FAMILY_IGNORED;
            b.dstQueueFamilyIndex = (fromForeign && foreign_) ? family_ : VK_QUEUE_FAMILY_IGNORED;
            b.image = img.image;
            b.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
            img.layout = to;
            return b;
        };
        pre[np++] = barrier(left, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_ACCESS_TRANSFER_READ_BIT, false);
        if (right.image != left.image) pre[np++] = barrier(right, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_ACCESS_TRANSFER_READ_BIT, false);
        dst.layout = VK_IMAGE_LAYOUT_UNDEFINED;
        pre[np++] = barrier(dst, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_ACCESS_TRANSFER_WRITE_BIT, false);
        vkCmdPipelineBarrier(cmd_, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, np, pre);

        const int32_t half = int32_t(dst.width / 2);
        auto copyRaw = [&](Image& src, const vr::VRTextureBounds_t& b, int32_t x0) {
            const int32_t x1 = int32_t(std::lround(std::min(b.uMin, b.uMax) * src.width));
            const int32_t y1 = int32_t(std::lround(std::min(b.vMin, b.vMax) * src.height));
            const int32_t x2 = int32_t(std::lround(std::max(b.uMin, b.uMax) * src.width));
            const int32_t y2 = int32_t(std::lround(std::max(b.vMin, b.vMax) * src.height));
            VkImageCopy r{};
            r.srcSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
            r.dstSubresource = r.srcSubresource;
            r.srcOffset = {x1, y1, 0};
            r.dstOffset = {x0 + x1, y1, 0};
            r.extent = {uint32_t(std::max(0, x2 - x1)), uint32_t(std::max(0, y2 - y1)), 1};
            vkCmdCopyImage(cmd_, src.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, dst.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &r);
        };
        auto blit = [&](Image& src, const vr::VRTextureBounds_t& b, int32_t x0) {
            VkImageBlit r{};
            r.srcSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
            r.dstSubresource = r.srcSubresource;
            r.srcOffsets[0] = {int32_t(std::lround(b.uMin * src.width)), int32_t(std::lround(b.vMin * src.height)), 0};
            r.srcOffsets[1] = {int32_t(std::lround(b.uMax * src.width)), int32_t(std::lround(b.vMax * src.height)), 1};
            r.dstOffsets[0] = {x0, 0, 0};
            r.dstOffsets[1] = {x0 + half, int32_t(dst.height), 1};
            vkCmdBlitImage(cmd_, src.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, dst.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &r,
                           VK_FILTER_LINEAR);
        };
        if (int32_t(left.width) == half && int32_t(right.width) == half && left.height == dst.height && right.height == dst.height) {
            copyRaw(left, lb, 0);
            copyRaw(right, rb, half);
        } else {
            blit(left, lb, 0);
            blit(right, rb, half);
        }

        VkImageMemoryBarrier post{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
        post.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        post.dstAccessMask = 0;
        post.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        post.newLayout = VK_IMAGE_LAYOUT_GENERAL;
        post.srcQueueFamilyIndex = foreign_ ? family_ : VK_QUEUE_FAMILY_IGNORED;
        post.dstQueueFamilyIndex = foreign_ ? VK_QUEUE_FAMILY_FOREIGN_EXT : VK_QUEUE_FAMILY_IGNORED;
        post.image = dst.image;
        post.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        vkCmdPipelineBarrier(cmd_, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, 0, 0, nullptr, 0, nullptr, 1, &post);
        dst.layout = VK_IMAGE_LAYOUT_GENERAL;
        vkEndCommandBuffer(cmd_);

        VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
        si.commandBufferCount = 1;
        si.pCommandBuffers = &cmd_;
        vkResetFences(device_, 1, &fence_);
        if (vkQueueSubmit(queue_, 1, &si, fence_) != VK_SUCCESS) return false;
        return vkWaitForFences(device_, 1, &fence_, VK_TRUE, 100000000ull) == VK_SUCCESS;
    }

    void idle() {
        if (device_) vkDeviceWaitIdle(device_);
    }

private:
    void* lib_ = nullptr;
    PFN_vkGetInstanceProcAddr gipa_ = nullptr;
    VkInstance instance_ = VK_NULL_HANDLE;
    VkPhysicalDevice phys_ = VK_NULL_HANDLE;
    VkDevice device_ = VK_NULL_HANDLE;
    VkQueue queue_ = VK_NULL_HANDLE;
    uint32_t family_ = 0;
    VkPhysicalDeviceMemoryProperties memProps_{};
    VkCommandPool pool_ = VK_NULL_HANDLE;
    VkCommandBuffer cmd_ = VK_NULL_HANDLE;
    VkFence fence_ = VK_NULL_HANDLE;
    bool modifiers_ = false, foreign_ = false;

    PFN_vkCreateInstance vkCreateInstance = nullptr;
    PFN_vkDestroyInstance vkDestroyInstance = nullptr;
    PFN_vkEnumeratePhysicalDevices vkEnumeratePhysicalDevices = nullptr;
    PFN_vkGetPhysicalDeviceProperties vkGetPhysicalDeviceProperties = nullptr;
    PFN_vkGetPhysicalDeviceQueueFamilyProperties vkGetPhysicalDeviceQueueFamilyProperties = nullptr;
    PFN_vkGetPhysicalDeviceMemoryProperties vkGetPhysicalDeviceMemoryProperties = nullptr;
    PFN_vkEnumerateDeviceExtensionProperties vkEnumerateDeviceExtensionProperties = nullptr;
    PFN_vkCreateDevice vkCreateDevice = nullptr;
    PFN_vkGetDeviceProcAddr vkGetDeviceProcAddr = nullptr;
    PFN_vkDestroyDevice vkDestroyDevice = nullptr;
    PFN_vkGetDeviceQueue vkGetDeviceQueue = nullptr;
    PFN_vkCreateImage vkCreateImage = nullptr;
    PFN_vkDestroyImage vkDestroyImage = nullptr;
    PFN_vkGetImageMemoryRequirements vkGetImageMemoryRequirements = nullptr;
    PFN_vkAllocateMemory vkAllocateMemory = nullptr;
    PFN_vkFreeMemory vkFreeMemory = nullptr;
    PFN_vkBindImageMemory vkBindImageMemory = nullptr;
    PFN_vkGetMemoryFdPropertiesKHR vkGetMemoryFdPropertiesKHR = nullptr;
    PFN_vkCreateCommandPool vkCreateCommandPool = nullptr;
    PFN_vkDestroyCommandPool vkDestroyCommandPool = nullptr;
    PFN_vkAllocateCommandBuffers vkAllocateCommandBuffers = nullptr;
    PFN_vkBeginCommandBuffer vkBeginCommandBuffer = nullptr;
    PFN_vkEndCommandBuffer vkEndCommandBuffer = nullptr;
    PFN_vkCmdPipelineBarrier vkCmdPipelineBarrier = nullptr;
    PFN_vkCmdBlitImage vkCmdBlitImage = nullptr;
    PFN_vkCmdCopyImage vkCmdCopyImage = nullptr;
    PFN_vkCmdClearColorImage vkCmdClearColorImage = nullptr;
    PFN_vkQueueSubmit vkQueueSubmit = nullptr;
    PFN_vkCreateFence vkCreateFence = nullptr;
    PFN_vkDestroyFence vkDestroyFence = nullptr;
    PFN_vkWaitForFences vkWaitForFences = nullptr;
    PFN_vkResetFences vkResetFences = nullptr;
    PFN_vkDeviceWaitIdle vkDeviceWaitIdle = nullptr;
    PFN_vkResetCommandBuffer vkResetCommandBuffer = nullptr;

    uint32_t memoryType(uint32_t bits, VkMemoryPropertyFlags want) {
        for (uint32_t i = 0; i < memProps_.memoryTypeCount; i++)
            if ((bits & (1u << i)) && (memProps_.memoryTypes[i].propertyFlags & want) == want) return i;
        for (uint32_t i = 0; i < memProps_.memoryTypeCount; i++)
            if (bits & (1u << i)) return i;
        return 0;
    }

    void destroy() {
        if (device_) {
            vkDeviceWaitIdle(device_);
            if (fence_) vkDestroyFence(device_, fence_, nullptr);
            if (pool_) vkDestroyCommandPool(device_, pool_, nullptr);
            vkDestroyDevice(device_, nullptr);
        }
        if (instance_) vkDestroyInstance(instance_, nullptr);
        device_ = VK_NULL_HANDLE;
        instance_ = VK_NULL_HANDLE;
    }
};

class DirectMode : public vr::IVRDriverDirectModeComponent {
public:
    DirectMode(Bridge& bridge) : bridge_(bridge) {}

    bool init() { return copier_.init(); }

    void CreateSwapTextureSet(uint32_t pid, const SwapTextureSetDesc_t* desc, SwapTextureSet_t* out) override {
        logf("CreateSwapTextureSet pid %u: %ux%u format %u, %u samples", pid, desc->nWidth, desc->nHeight, desc->nFormat, desc->nSampleCount);
        std::lock_guard<std::mutex> l(lock_);
        auto* set = new TexSet();
        set->pid = pid;
        set->desc = *desc;
        for (int i = 0; i < 3; i++) {
            vr::SharedTextureHandle_t h = 0;
            if (!vr::VRIPCResourceManager()->NewSharedVulkanImage(desc->nFormat, desc->nWidth, desc->nHeight, true, false, true, 1, 1,
                                                                  VK_IMAGE_CREATE_MUTABLE_FORMAT_BIT,
                                                                  VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_SAMPLED_BIT |
                                                                      VK_IMAGE_USAGE_INPUT_ATTACHMENT_BIT,
                                                                  &h)) {
                logf("NewSharedVulkanImage failed");
                destroySet(set);
                return;
            }
            uint64_t ipc = 0;
            vr::VRIPCResourceManager()->RefResource(h, &ipc);
            int fd = -1;
            if (!vr::VRIPCResourceManager()->ReceiveSharedFd(ipc, &fd)) {
                logf("ReceiveSharedFd failed");
                vr::VRIPCResourceManager()->UnrefResource(h);
                destroySet(set);
                return;
            }
            set->handles[i] = h;
            bool srgb = false;
            unormTwinOf(desc->nFormat, srgb);
            if (!copier_.importEye(fd, desc->nFormat, desc->nWidth, desc->nHeight, set->images[i])) {
                destroySet(set);
                return;
            }
            set->srgb = srgb;
            byHandle_[h] = {set, i};
            out->rSharedTextureHandles[i] = h;
        }
        out->unTextureFlags = 0;
        sets_.push_back(set);
        if (desc->nWidth != eyeW_ || desc->nHeight != eyeH_) {
            eyeW_ = desc->nWidth;
            eyeH_ = desc->nHeight;
            qdvr_need_buffers nb{QDVR_MSG_NEED_BUFFERS, eyeW_, eyeH_, desc->nFormat};
            bridge_.send(&nb, sizeof(nb));
            needSent_ = true;
        }
    }

    void DestroySwapTextureSet(vr::SharedTextureHandle_t h) override {
        std::lock_guard<std::mutex> l(lock_);
        auto it = byHandle_.find(h);
        if (it != byHandle_.end()) destroySet(it->second.first);
    }

    void DestroyAllSwapTextureSets(uint32_t pid) override {
        std::lock_guard<std::mutex> l(lock_);
        std::vector<TexSet*> doomed;
        for (TexSet* s : sets_)
            if (s->pid == pid) doomed.push_back(s);
        for (TexSet* s : doomed) destroySet(s);
    }

    void GetNextSwapTextureSetIndex(vr::SharedTextureHandle_t handles[2], uint32_t (*indices)[2]) override {
        std::lock_guard<std::mutex> l(lock_);
        for (int e = 0; e < 2; e++) {
            auto it = byHandle_.find(handles[e]);
            if (it == byHandle_.end()) continue;
            TexSet* s = it->second.first;
            s->next = (s->next + 1) % 3;
            if (indices) (*indices)[e] = s->next;
        }
    }

    void SubmitLayer(const SubmitLayerPerEye_t (&perEye)[2]) override {
        std::lock_guard<std::mutex> l(lock_);
        if (layers_++ == 0) {
            layer_[0] = perEye[0];
            layer_[1] = perEye[1];
        }
    }

    void Present(vr::SharedTextureHandle_t) override {
        std::unique_lock<std::mutex> l(lock_);
        const int layers = layers_;
        layers_ = 0;
        if (!layers) return;
        refreshTargets();
        if (targets_.empty()) {
            if (!needSent_ && eyeW_) {
                qdvr_need_buffers nb{QDVR_MSG_NEED_BUFFERS, eyeW_, eyeH_, 0};
                needSent_ = bridge_.send(&nb, sizeof(nb));
            }
            return;
        }
        auto li = byHandle_.find(layer_[0].hTexture), ri = byHandle_.find(layer_[1].hTexture);
        if (li == byHandle_.end() || ri == byHandle_.end()) return;
        Copier::Image& left = li->second.first->images[li->second.second];
        Copier::Image& right = ri->second.first->images[ri->second.second];
        const bool srgb = li->second.first->srgb;
        const uint32_t index = next_;
        next_ = (next_ + 1) % uint32_t(targets_.size());
        std::this_thread::sleep_for(std::chrono::microseconds(1500));
        if (!copier_.copy(left, layer_[0].bounds, right, layer_[1].bounds, targets_[index])) {
            if (!copyFailedSaid_) logf("copying a frame failed");
            copyFailedSaid_ = true;
            return;
        }
        qdvr_frame f{};
        f.type = QDVR_MSG_FRAME;
        f.generation = generation_;
        f.index = index;
        f.renderHead = poseFromMat(layer_[0].mHmdPose);
        f.srgb = srgb;
        bridge_.send(&f, sizeof(f));
        if (++frames_ == 1 || frames_ % 900 == 0) logf("frames sent to the headset: %llu", (unsigned long long)frames_);
    }

    void PostPresent(const Throttling_t*) override {
        bridge_.waitTrackingAfter(lastSeq_, 30);
        lastSeq_ = bridge_.trackingSeq();
    }

    void GetFrameTiming(vr::DriverDirectMode_FrameTiming* t) override {
        if (!t || t->m_nSize < sizeof(vr::DriverDirectMode_FrameTiming)) return;
        t->m_nNumFramePresents = 1;
        t->m_nNumMisPresented = 0;
        t->m_nNumDroppedFrames = 0;
        t->m_nReprojectionFlags = 0;
    }

private:
    struct TexSet {
        uint32_t pid = 0;
        SwapTextureSetDesc_t desc{};
        vr::SharedTextureHandle_t handles[3]{};
        Copier::Image images[3];
        uint32_t next = 0;
        bool srgb = false;
    };
    Bridge& bridge_;
    Copier copier_;
    std::mutex lock_;
    std::vector<TexSet*> sets_;
    std::map<vr::SharedTextureHandle_t, std::pair<TexSet*, int>> byHandle_;
    SubmitLayerPerEye_t layer_[2]{};
    int layers_ = 0;
    uint32_t eyeW_ = 0, eyeH_ = 0;
    bool needSent_ = false;
    std::vector<Copier::Image> targets_;
    uint32_t generation_ = 0, next_ = 0;
    uint64_t frames_ = 0;
    uint32_t lastSeq_ = 0;
    bool copyFailedSaid_ = false;

    void destroySet(TexSet* s) {
        copier_.idle();
        for (int i = 0; i < 3; i++) {
            if (s->handles[i]) {
                byHandle_.erase(s->handles[i]);
                vr::VRIPCResourceManager()->UnrefResource(s->handles[i]);
            }
            copier_.release(s->images[i]);
        }
        for (auto it = sets_.begin(); it != sets_.end(); ++it)
            if (*it == s) {
                sets_.erase(it);
                break;
            }
        delete s;
    }

    void refreshTargets() {
        qdvr_buffers b{};
        std::vector<int> fds;
        if (!bridge_.takeBuffers(b, fds)) {
            if (!bridge_.connected() && !targets_.empty()) {
                copier_.idle();
                for (auto& t : targets_) copier_.release(t);
                targets_.clear();
                needSent_ = false;
            }
            return;
        }
        copier_.idle();
        for (auto& t : targets_) copier_.release(t);
        targets_.clear();
        for (int fd : fds) {
            Copier::Image img;
            if (!copier_.importTarget(fd, b, img)) {
                for (auto& t : targets_) copier_.release(t);
                targets_.clear();
                return;
            }
            targets_.push_back(img);
        }
        generation_ = b.generation;
        next_ = 0;
        logf("frame buffers ready (generation %u)", generation_);
    }
};

class Hmd : public vr::ITrackedDeviceServerDriver, public vr::IVRDisplayComponent {
public:
    Hmd(Bridge& bridge) : bridge_(bridge), direct_(bridge) {
        hello_.eyeWidth = 1832;
        hello_.eyeHeight = 1920;
        hello_.refreshHz = 72;
        for (int e = 0; e < 2; e++) {
            hello_.fov[e] = {e == 0 ? -1.06f : -0.82f, e == 0 ? 0.82f : 1.06f, 0.95f, -1.1f};
            hello_.eyeToHead[e] = {e == 0 ? -0.0315f : 0.0315f, 0, 0, 0, 0, 0, 1};
        }
    }

    bool initDirect() { return direct_.init(); }
    void setHello(const qdvr_hello& h) {
        hello_ = h;
        if (id_ != vr::k_unTrackedDeviceIndexInvalid) applyDisplay();
    }
    uint32_t id() const { return id_; }

    vr::EVRInitError Activate(uint32_t id) override {
        id_ = id;
        auto* p = vr::VRProperties();
        const vr::PropertyContainerHandle_t c = p->TrackedDeviceToPropertyContainer(id);
        p->SetStringProperty(c, vr::Prop_ModelNumber_String, "Meta Quest (QuestDeck)");
        p->SetStringProperty(c, vr::Prop_ManufacturerName_String, "Meta");
        p->SetStringProperty(c, vr::Prop_TrackingSystemName_String, "questdeck");
        p->SetStringProperty(c, vr::Prop_RenderModelName_String, "generic_hmd");
        p->SetStringProperty(c, vr::Prop_SerialNumber_String, "QuestDeck-HMD");
        p->SetFloatProperty(c, vr::Prop_UserIpdMeters_Float, ipd());
        p->SetFloatProperty(c, vr::Prop_UserHeadToEyeDepthMeters_Float, 0.0f);
        p->SetFloatProperty(c, vr::Prop_DisplayFrequency_Float, hello_.refreshHz);
        p->SetFloatProperty(c, vr::Prop_SecondsFromVsyncToPhotons_Float, 0.0f);
        p->SetUint64Property(c, vr::Prop_CurrentUniverseId_Uint64, 2);
        p->SetBoolProperty(c, vr::Prop_IsOnDesktop_Bool, false);
        p->SetBoolProperty(c, vr::Prop_DisplayDebugMode_Bool, false);
        p->SetBoolProperty(c, vr::Prop_DriverDirectModeSendsVsyncEvents_Bool, true);
        p->SetBoolProperty(c, vr::Prop_Hmd_AllowsClientToControlTextureIndex, true);
        p->SetBoolProperty(c, vr::Prop_DeviceProvidesBatteryStatus_Bool, false);
        p->SetBoolProperty(c, vr::Prop_ContainsProximitySensor_Bool, false);
        applyDisplay();
        logf("headset device %u active", id);
        return vr::VRInitError_None;
    }
    void Deactivate() override { id_ = vr::k_unTrackedDeviceIndexInvalid; }
    void EnterStandby() override {}
    void* GetComponent(const char* name) override {
        if (!strcmp(name, vr::IVRDisplayComponent_Version)) return static_cast<vr::IVRDisplayComponent*>(this);
        if (!strcmp(name, vr::IVRDriverDirectModeComponent_Version)) return static_cast<vr::IVRDriverDirectModeComponent*>(&direct_);
        return nullptr;
    }
    void DebugRequest(const char*, char* resp, uint32_t size) override {
        if (size) resp[0] = 0;
    }
    vr::DriverPose_t GetPose() override { return pose_; }

    void track(const qdvr_tracking& t) {
        pose_ = driverPose(t.head, t.headValid);
        if (id_ != vr::k_unTrackedDeviceIndexInvalid) vr::VRServerDriverHost()->TrackedDevicePoseUpdated(id_, pose_, sizeof(pose_));
    }

    // IVRDisplayComponent
    void GetWindowBounds(int32_t* x, int32_t* y, uint32_t* w, uint32_t* h) override {
        *x = 0;
        *y = 0;
        *w = hello_.eyeWidth * 2;
        *h = hello_.eyeHeight;
    }
    bool IsDisplayOnDesktop() override { return false; }
    bool IsDisplayRealDisplay() override { return false; }
    void GetRecommendedRenderTargetSize(uint32_t* w, uint32_t* h) override {
        *w = hello_.eyeWidth;
        *h = hello_.eyeHeight;
    }
    void GetEyeOutputViewport(vr::EVREye eye, uint32_t* x, uint32_t* y, uint32_t* w, uint32_t* h) override {
        *x = eye == vr::Eye_Left ? 0 : hello_.eyeWidth;
        *y = 0;
        *w = hello_.eyeWidth;
        *h = hello_.eyeHeight;
    }
    void GetProjectionRaw(vr::EVREye eye, float* l, float* r, float* t, float* b) override {
        const qdvr_fov& f = hello_.fov[eye == vr::Eye_Left ? 0 : 1];
        *l = f.left;
        *r = f.right;
        *t = f.down;
        *b = f.up;
    }
    vr::DistortionCoordinates_t ComputeDistortion(vr::EVREye, float u, float v) override { return {{u, v}, {u, v}, {u, v}}; }
    bool ComputeInverseDistortion(vr::HmdVector2_t* out, vr::EVREye, uint32_t, float u, float v) override {
        out->v[0] = u;
        out->v[1] = v;
        return true;
    }

private:
    Bridge& bridge_;
    DirectMode direct_;
    uint32_t id_ = vr::k_unTrackedDeviceIndexInvalid;
    qdvr_hello hello_{};
    vr::DriverPose_t pose_ = driverPose(qdvr_pose{0, 1.6f, 0, 0, 0, 0, 1}, false);

    float ipd() const { return std::fabs(hello_.eyeToHead[1].px - hello_.eyeToHead[0].px); }

    void applyDisplay() {
        auto* host = vr::VRServerDriverHost();
        vr::HmdMatrix34_t l = matFromPose(hello_.eyeToHead[0]), r = matFromPose(hello_.eyeToHead[1]);
        host->SetDisplayEyeToHead(id_, l, r);
        auto rect = [](const qdvr_fov& f) {
            vr::HmdRect2_t x;
            x.vTopLeft.v[0] = f.left;
            x.vTopLeft.v[1] = f.down;
            x.vBottomRight.v[0] = f.right;
            x.vBottomRight.v[1] = f.up;
            return x;
        };
        host->SetDisplayProjectionRaw(id_, rect(hello_.fov[0]), rect(hello_.fov[1]));
        host->SetRecommendedRenderTargetSize(id_, hello_.eyeWidth, hello_.eyeHeight);
        auto* p = vr::VRProperties();
        const vr::PropertyContainerHandle_t c = p->TrackedDeviceToPropertyContainer(id_);
        p->SetFloatProperty(c, vr::Prop_DisplayFrequency_Float, hello_.refreshHz);
        p->SetFloatProperty(c, vr::Prop_UserIpdMeters_Float, ipd());
    }
};

class Controller : public vr::ITrackedDeviceServerDriver {
public:
    explicit Controller(int hand) : hand_(hand) {}
    uint32_t id() const { return id_; }
    vr::VRInputComponentHandle_t haptic() const { return haptic_; }

    vr::EVRInitError Activate(uint32_t id) override {
        id_ = id;
        auto* p = vr::VRProperties();
        const vr::PropertyContainerHandle_t c = p->TrackedDeviceToPropertyContainer(id);
        const bool left = hand_ == 0;
        p->SetStringProperty(c, vr::Prop_TrackingSystemName_String, "oculus");
        p->SetStringProperty(c, vr::Prop_ManufacturerName_String, "Oculus");
        p->SetStringProperty(c, vr::Prop_ModelNumber_String, left ? "Meta Quest Touch Plus (Left Controller)" : "Meta Quest Touch Plus (Right Controller)");
        p->SetStringProperty(c, vr::Prop_SerialNumber_String, left ? "QuestDeck-Left" : "QuestDeck-Right");
        p->SetStringProperty(c, vr::Prop_RenderModelName_String, left ? "oculus_quest_plus_controller_left" : "oculus_quest_plus_controller_right");
        p->SetStringProperty(c, vr::Prop_ControllerType_String, "oculus_touch");
        p->SetStringProperty(c, vr::Prop_InputProfilePath_String, "{oculus}/input/touch_profile.json");
        p->SetStringProperty(c, vr::Prop_RegisteredDeviceType_String, left ? "oculus/QuestDeck_Controller_Left" : "oculus/QuestDeck_Controller_Right");
        p->SetInt32Property(c, vr::Prop_ControllerRoleHint_Int32, left ? vr::TrackedControllerRole_LeftHand : vr::TrackedControllerRole_RightHand);
        p->SetInt32Property(c, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_Controller);
        auto* in = vr::VRDriverInput();
        const char* a = left ? "/input/x" : "/input/a";
        const char* b = left ? "/input/y" : "/input/b";
        in->CreateBooleanComponent(c, (std::string(a) + "/click").c_str(), &aClick_);
        in->CreateBooleanComponent(c, (std::string(a) + "/touch").c_str(), &aTouch_);
        in->CreateBooleanComponent(c, (std::string(b) + "/click").c_str(), &bClick_);
        in->CreateBooleanComponent(c, (std::string(b) + "/touch").c_str(), &bTouch_);
        if (left) in->CreateBooleanComponent(c, "/input/system/click", &system_);
        in->CreateScalarComponent(c, "/input/trigger/value", &trigger_, vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
        in->CreateBooleanComponent(c, "/input/trigger/click", &triggerClick_);
        in->CreateBooleanComponent(c, "/input/trigger/touch", &triggerTouch_);
        in->CreateScalarComponent(c, "/input/grip/value", &grip_, vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
        in->CreateBooleanComponent(c, "/input/grip/touch", &gripTouch_);
        in->CreateScalarComponent(c, "/input/joystick/x", &stickX_, vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
        in->CreateScalarComponent(c, "/input/joystick/y", &stickY_, vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
        in->CreateBooleanComponent(c, "/input/joystick/click", &stickClick_);
        in->CreateBooleanComponent(c, "/input/joystick/touch", &stickTouch_);
        in->CreateBooleanComponent(c, "/input/thumbrest/touch", &thumbrest_);
        in->CreateHapticComponent(c, "/output/haptic", &haptic_);
        logf("%s controller device %u active", left ? "left" : "right", id);
        return vr::VRInitError_None;
    }
    void Deactivate() override { id_ = vr::k_unTrackedDeviceIndexInvalid; }
    void EnterStandby() override {}
    void* GetComponent(const char*) override { return nullptr; }
    void DebugRequest(const char*, char* resp, uint32_t size) override {
        if (size) resp[0] = 0;
    }
    vr::DriverPose_t GetPose() override { return pose_; }

    void track(const qdvr_hand& h) {
        if (id_ == vr::k_unTrackedDeviceIndexInvalid) return;
        const bool valid = (h.flags & QDVR_HAND_ACTIVE) && (h.flags & QDVR_HAND_POSE_VALID);
        pose_ = driverPose(h.gripPose, valid);
        pose_.deviceIsConnected = (h.flags & QDVR_HAND_ACTIVE) != 0;
        const double pitch = -15.0 * 3.14159265358979 / 180.0;
        pose_.qDriverFromHeadRotation = {std::cos(pitch / 2), std::sin(pitch / 2), 0, 0};
        pose_.vecDriverFromHeadTranslation[0] = hand_ == 0 ? -0.005 : 0.005;
        pose_.vecDriverFromHeadTranslation[1] = -0.005 + 0.11 * std::sin(pitch);
        pose_.vecDriverFromHeadTranslation[2] = -0.11 * std::cos(pitch);
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(id_, pose_, sizeof(pose_));
        auto* in = vr::VRDriverInput();
        in->UpdateBooleanComponent(aClick_, h.buttons & QDVR_BTN_A, 0);
        in->UpdateBooleanComponent(aTouch_, h.touches & QDVR_BTN_A, 0);
        in->UpdateBooleanComponent(bClick_, h.buttons & QDVR_BTN_B, 0);
        in->UpdateBooleanComponent(bTouch_, h.touches & QDVR_BTN_B, 0);
        if (system_) in->UpdateBooleanComponent(system_, h.buttons & QDVR_BTN_SYSTEM, 0);
        in->UpdateScalarComponent(trigger_, h.trigger, 0);
        in->UpdateBooleanComponent(triggerClick_, h.trigger > 0.75f, 0);
        in->UpdateBooleanComponent(triggerTouch_, (h.touches & QDVR_BTN_TRIGGER) || h.trigger > 0.05f, 0);
        in->UpdateScalarComponent(grip_, h.grip, 0);
        in->UpdateBooleanComponent(gripTouch_, h.grip > 0.05f, 0);
        in->UpdateScalarComponent(stickX_, h.stickX, 0);
        in->UpdateScalarComponent(stickY_, h.stickY, 0);
        in->UpdateBooleanComponent(stickClick_, h.buttons & QDVR_BTN_STICK, 0);
        in->UpdateBooleanComponent(stickTouch_, (h.touches & QDVR_BTN_STICK) || std::fabs(h.stickX) + std::fabs(h.stickY) > 0.05f, 0);
        in->UpdateBooleanComponent(thumbrest_, h.touches & QDVR_BTN_SYSTEM, 0);
    }

private:
    const int hand_;
    uint32_t id_ = vr::k_unTrackedDeviceIndexInvalid;
    vr::DriverPose_t pose_ = driverPose(qdvr_pose{0, 0, 0, 0, 0, 0, 1}, false);
    vr::VRInputComponentHandle_t aClick_ = 0, aTouch_ = 0, bClick_ = 0, bTouch_ = 0, system_ = 0, trigger_ = 0, triggerClick_ = 0,
                                 triggerTouch_ = 0, grip_ = 0, gripTouch_ = 0, stickX_ = 0, stickY_ = 0, stickClick_ = 0, stickTouch_ = 0,
                                 thumbrest_ = 0, haptic_ = 0;
};

class Provider : public vr::IServerTrackedDeviceProvider {
public:
    vr::EVRInitError Init(vr::IVRDriverContext* ctx) override {
        VR_INIT_SERVER_DRIVER_CONTEXT(ctx);
        logf("driver_questdeck starting");
        hmd_ = new Hmd(bridge_);
        if (!hmd_->initDirect()) {
            logf("no Vulkan for copying frames: the headset cannot show SteamVR");
            return vr::VRInitError_Driver_Failed;
        }
        left_ = new Controller(0);
        right_ = new Controller(1);
        bridge_.onTracking = [this](const qdvr_tracking& t) {
            hmd_->track(t);
            left_->track(t.hands[0]);
            right_->track(t.hands[1]);
            vr::VRServerDriverHost()->VsyncEvent(0);
        };
        bridge_.start();
        qdvr_hello h;
        if (bridge_.waitHello(4000, h))
            hmd_->setHello(h);
        else
            logf("no headset description yet; starting with defaults");
        vr::VRServerDriverHost()->TrackedDeviceAdded("QuestDeck-HMD", vr::TrackedDeviceClass_HMD, hmd_);
        vr::VRServerDriverHost()->TrackedDeviceAdded("QuestDeck-Left", vr::TrackedDeviceClass_Controller, left_);
        vr::VRServerDriverHost()->TrackedDeviceAdded("QuestDeck-Right", vr::TrackedDeviceClass_Controller, right_);
        return vr::VRInitError_None;
    }

    void Cleanup() override {
        bridge_.stop();
        VR_CLEANUP_SERVER_DRIVER_CONTEXT();
    }
    const char* const* GetInterfaceVersions() override { return vr::k_InterfaceVersions; }

    void RunFrame() override {
        qdvr_hello h;
        if (bridge_.hello(h) && memcmp(&h, &applied_, sizeof(h)) != 0) {
            applied_ = h;
            hmd_->setHello(h);
        }
        vr::VREvent_t ev;
        while (vr::VRServerDriverHost()->PollNextEvent(&ev, sizeof(ev))) {
            if (ev.eventType != vr::VREvent_Input_HapticVibration) continue;
            const auto& v = ev.data.hapticVibration;
            int hand = -1;
            if (left_ && v.componentHandle == left_->haptic()) hand = 0;
            if (right_ && v.componentHandle == right_->haptic()) hand = 1;
            if (hand < 0) continue;
            qdvr_haptic m{QDVR_MSG_HAPTIC, uint32_t(hand), v.fDurationSeconds, v.fFrequency, v.fAmplitude};
            bridge_.send(&m, sizeof(m));
        }
    }
    bool ShouldBlockStandbyMode() override { return false; }
    void EnterStandby() override {}
    void LeaveStandby() override {}

private:
    Bridge bridge_;
    Hmd* hmd_ = nullptr;
    Controller* left_ = nullptr;
    Controller* right_ = nullptr;
    qdvr_hello applied_{};
};

Provider gProvider;

}  // namespace

extern "C" __attribute__((visibility("default"))) void* HmdDriverFactory(const char* name, int* code) {
    if (!strcmp(name, vr::IServerTrackedDeviceProvider_Version)) return &gProvider;
    if (code) *code = vr::VRInitError_Init_InterfaceNotFound;
    return nullptr;
}
