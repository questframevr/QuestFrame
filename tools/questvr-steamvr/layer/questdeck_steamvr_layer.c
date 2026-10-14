//
//
#define _GNU_SOURCE
#include <fcntl.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>
#include <vulkan/vk_layer.h>
#include <vulkan/vulkan.h>

#define EXPORT __attribute__((visibility("default")))

static void logf_(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
static void logf_(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    fprintf(stderr, "[questdeck-layer] ");
    vfprintf(stderr, fmt, ap);
    fputc('\n', stderr);
    va_end(ap);
}

static int gDebug = -1;
static int debugOn(void) {
    if (gDebug < 0) gDebug = getenv("QUESTDECK_LAYER_DEBUG") != NULL;
    return gDebug;
}
#define DBG(...)     do {                             if (debugOn()) logf_(__VA_ARGS__);     } while (0)

static const char* const DROPPED[] = {
    "VK_KHR_display", "VK_KHR_get_display_properties2", "VK_EXT_direct_mode_display", "VK_EXT_display_surface_counter",
    "VK_EXT_acquire_drm_display", "VK_EXT_acquire_xlib_display",
};

static int dropped(const char* name) {
    for (size_t i = 0; i < sizeof(DROPPED) / sizeof(DROPPED[0]); i++)
        if (!strcmp(name, DROPPED[i])) return 1;
    return 0;
}

static pthread_mutex_t gLock = PTHREAD_MUTEX_INITIALIZER;

// --- Instances ----------------------------------------------------------------------------------
typedef struct Instance {
    void* key;
    VkInstance handle;
    PFN_vkGetInstanceProcAddr gipa;
    PFN_vkDestroyInstance destroy;
    PFN_vkGetPhysicalDeviceExternalSemaphoreProperties extSem;
    PFN_vkGetPhysicalDeviceExternalSemaphoreProperties extSemKhr;
    struct Instance* next;
} Instance;
static Instance* gInstances;

static void* key(const void* dispatchable) { return *(void* const*)dispatchable; }

static Instance* instanceFor(const void* dispatchable) {
    void* k = key(dispatchable);
    pthread_mutex_lock(&gLock);
    Instance* i = gInstances;
    while (i && i->key != k) i = i->next;
    pthread_mutex_unlock(&gLock);
    return i;
}

// --- Devices ------------------------------------------------------------------------------------
typedef struct Device {
    void* key;
    VkDevice handle;
    PFN_vkGetDeviceProcAddr gdpa;
    PFN_vkDestroyDevice destroy;
    PFN_vkCreateSemaphore createSemaphore;
    PFN_vkDestroySemaphore destroySemaphore;
    PFN_vkGetSemaphoreCounterValue counterValue;
    PFN_vkSignalSemaphore signalSemaphore;
    PFN_vkWaitSemaphores waitSemaphores;
    PFN_vkQueueSubmit queueSubmit;
    PFN_vkQueueSubmit2 queueSubmit2;
    PFN_vkGetSemaphoreFdKHR getSemaphoreFd;
    PFN_vkImportSemaphoreFdKHR importSemaphoreFd;
    PFN_vkGetFenceStatus getFenceStatus;
    PFN_vkWaitForFences waitForFences;
    PFN_vkAcquireNextImageKHR acquireNextImage;
    PFN_vkQueuePresentKHR queuePresent;
    struct Device* next;
} Device;
static Device* gDevices;

static Device* deviceFor(const void* dispatchable) {
    void* k = key(dispatchable);
    pthread_mutex_lock(&gLock);
    Device* d = gDevices;
    while (d && d->key != k) d = d->next;
    pthread_mutex_unlock(&gLock);
    return d;
}

typedef struct Shared {
    VkSemaphore semaphore;
    Device* device;
    int fd;
    _Atomic uint64_t* counter;  // mapped from fd
    _Atomic uint64_t pendingSignal;
    uint64_t lastLocal;
    struct Shared* next;
} Shared;
static Shared* gShared;
static pthread_t gWatcher;
static int gWatcherRunning;

static Shared* sharedFor(VkSemaphore s) {
    pthread_mutex_lock(&gLock);
    Shared* x = gShared;
    while (x && x->semaphore != s) x = x->next;
    pthread_mutex_unlock(&gLock);
    return x;
}

static void atomicMax(_Atomic uint64_t* p, uint64_t v) {
    uint64_t cur = atomic_load(p);
    while (cur < v && !atomic_compare_exchange_weak(p, &cur, v)) {
    }
}

static void* watch(void* arg) {
    (void)arg;
    for (;;) {
        pthread_mutex_lock(&gLock);
        for (Shared* x = gShared; x; x = x->next) {
            uint64_t local = 0;
            if (x->device->counterValue(x->device->handle, x->semaphore, &local) != VK_SUCCESS) continue;
            x->lastLocal = local;
            const uint64_t shared = atomic_load(x->counter);
            if (local > shared) {
                atomicMax(x->counter, local);
            } else if (shared > local && atomic_load(&x->pendingSignal) <= local) {
                VkSemaphoreSignalInfo si = {VK_STRUCTURE_TYPE_SEMAPHORE_SIGNAL_INFO};
                si.semaphore = x->semaphore;
                si.value = shared;
                x->device->signalSemaphore(x->device->handle, &si);
            }
        }
        pthread_mutex_unlock(&gLock);
        struct timespec ts = {0, 200000};  // 0.2 ms
        nanosleep(&ts, NULL);
    }
    return NULL;
}

static void addShared(Device* d, VkSemaphore s, int fd, uint64_t initial) {
    Shared* x = calloc(1, sizeof(*x));
    x->semaphore = s;
    x->device = d;
    x->fd = fd;
    x->counter = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (x->counter == MAP_FAILED) {
        logf_("mapping a shared timeline failed");
        close(fd);
        free(x);
        return;
    }
    atomicMax(x->counter, initial);
    pthread_mutex_lock(&gLock);
    x->next = gShared;
    gShared = x;
    if (!gWatcherRunning) {
        gWatcherRunning = 1;
        pthread_create(&gWatcher, NULL, watch, NULL);
        pthread_detach(gWatcher);
    }
    pthread_mutex_unlock(&gLock);
}

static void removeShared(VkSemaphore s) {
    pthread_mutex_lock(&gLock);
    for (Shared** p = &gShared; *p; p = &(*p)->next)
        if ((*p)->semaphore == s) {
            Shared* x = *p;
            *p = x->next;
            munmap((void*)x->counter, 4096);
            close(x->fd);
            free(x);
            break;
        }
    pthread_mutex_unlock(&gLock);
}

static int newCounterFd(void) {
    int fd = (int)syscall(SYS_memfd_create, "questdeck-timeline", 1u /* MFD_CLOEXEC */);
    if (fd >= 0 && ftruncate(fd, 4096) != 0) {
        close(fd);
        fd = -1;
    }
    return fd;
}

static const void* findNext(const void* chain, VkStructureType t) {
    for (const VkBaseInStructure* b = chain; b; b = b->pNext)
        if (b->sType == t) return b;
    return NULL;
}

static VkResult VKAPI_CALL CreateInstance(const VkInstanceCreateInfo* ci, const VkAllocationCallbacks* alloc, VkInstance* out) {
    VkLayerInstanceCreateInfo* link = (VkLayerInstanceCreateInfo*)ci->pNext;
    while (link && !(link->sType == VK_STRUCTURE_TYPE_LOADER_INSTANCE_CREATE_INFO && link->function == VK_LAYER_LINK_INFO))
        link = (VkLayerInstanceCreateInfo*)link->pNext;
    if (!link) return VK_ERROR_INITIALIZATION_FAILED;
    PFN_vkGetInstanceProcAddr gipa = link->u.pLayerInfo->pfnNextGetInstanceProcAddr;
    link->u.pLayerInfo = link->u.pLayerInfo->pNext;
    PFN_vkCreateInstance next = (PFN_vkCreateInstance)gipa(VK_NULL_HANDLE, "vkCreateInstance");
    if (!next) return VK_ERROR_INITIALIZATION_FAILED;

    VkInstanceCreateInfo copy = *ci;
    const char** names = calloc(ci->enabledExtensionCount + 1, sizeof(char*));
    uint32_t n = 0;
    for (uint32_t i = 0; i < ci->enabledExtensionCount; i++)
        if (!dropped(ci->ppEnabledExtensionNames[i])) names[n++] = ci->ppEnabledExtensionNames[i];
    copy.enabledExtensionCount = n;
    copy.ppEnabledExtensionNames = names;
    VkResult r = next(&copy, alloc, out);
    free(names);
    if (r != VK_SUCCESS) return r;
    Instance* i = calloc(1, sizeof(*i));
    i->key = key(*out);
    i->handle = *out;
    i->gipa = gipa;
    i->destroy = (PFN_vkDestroyInstance)gipa(*out, "vkDestroyInstance");
    i->extSem = (PFN_vkGetPhysicalDeviceExternalSemaphoreProperties)gipa(*out, "vkGetPhysicalDeviceExternalSemaphoreProperties");
    i->extSemKhr = (PFN_vkGetPhysicalDeviceExternalSemaphoreProperties)gipa(*out, "vkGetPhysicalDeviceExternalSemaphorePropertiesKHR");
    pthread_mutex_lock(&gLock);
    i->next = gInstances;
    gInstances = i;
    pthread_mutex_unlock(&gLock);
    return r;
}

static void VKAPI_CALL DestroyInstance(VkInstance instance, const VkAllocationCallbacks* alloc) {
    if (!instance) return;
    void* k = key(instance);
    pthread_mutex_lock(&gLock);
    Instance* found = NULL;
    for (Instance** p = &gInstances; *p; p = &(*p)->next)
        if ((*p)->key == k) {
            found = *p;
            *p = found->next;
            break;
        }
    pthread_mutex_unlock(&gLock);
    if (found) {
        found->destroy(instance, alloc);
        free(found);
    }
}

static void sharedTimelineProperties(const VkPhysicalDeviceExternalSemaphoreInfo* in, VkExternalSemaphoreProperties* out) {
    const VkSemaphoreTypeCreateInfo* t = findNext(in->pNext, VK_STRUCTURE_TYPE_SEMAPHORE_TYPE_CREATE_INFO);
    if (t && t->semaphoreType == VK_SEMAPHORE_TYPE_TIMELINE && in->handleType == VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_FD_BIT) {
        out->exportFromImportedHandleTypes = VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_FD_BIT;
        out->compatibleHandleTypes = VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_FD_BIT;
        out->externalSemaphoreFeatures = VK_EXTERNAL_SEMAPHORE_FEATURE_EXPORTABLE_BIT | VK_EXTERNAL_SEMAPHORE_FEATURE_IMPORTABLE_BIT;
    }
}

static void VKAPI_CALL GetExternalSemaphoreProperties(VkPhysicalDevice pd, const VkPhysicalDeviceExternalSemaphoreInfo* in,
                                                      VkExternalSemaphoreProperties* out) {
    Instance* i = instanceFor(pd);
    if (i && i->extSem) i->extSem(pd, in, out);
    sharedTimelineProperties(in, out);
}

static void VKAPI_CALL GetExternalSemaphorePropertiesKhr(VkPhysicalDevice pd, const VkPhysicalDeviceExternalSemaphoreInfo* in,
                                                         VkExternalSemaphoreProperties* out) {
    Instance* i = instanceFor(pd);
    if (i && (i->extSemKhr || i->extSem)) (i->extSemKhr ? i->extSemKhr : i->extSem)(pd, in, out);
    sharedTimelineProperties(in, out);
}

// --- Devices ------------------------------------------------------------------------------------
static VkResult VKAPI_CALL CreateDevice(VkPhysicalDevice pd, const VkDeviceCreateInfo* ci, const VkAllocationCallbacks* alloc, VkDevice* out) {
    VkLayerDeviceCreateInfo* link = (VkLayerDeviceCreateInfo*)ci->pNext;
    while (link && !(link->sType == VK_STRUCTURE_TYPE_LOADER_DEVICE_CREATE_INFO && link->function == VK_LAYER_LINK_INFO))
        link = (VkLayerDeviceCreateInfo*)link->pNext;
    if (!link) return VK_ERROR_INITIALIZATION_FAILED;
    PFN_vkGetInstanceProcAddr gipa = link->u.pLayerInfo->pfnNextGetInstanceProcAddr;
    PFN_vkGetDeviceProcAddr gdpa = link->u.pLayerInfo->pfnNextGetDeviceProcAddr;
    link->u.pLayerInfo = link->u.pLayerInfo->pNext;
    Instance* inst = instanceFor(pd);
    PFN_vkCreateDevice next = (PFN_vkCreateDevice)gipa(inst ? inst->handle : VK_NULL_HANDLE, "vkCreateDevice");
    if (!next) return VK_ERROR_INITIALIZATION_FAILED;
    VkDeviceCreateInfo copy = *ci;
    const char** names = calloc(ci->enabledExtensionCount + 1, sizeof(char*));
    uint32_t n = 0;
    for (uint32_t i = 0; i < ci->enabledExtensionCount; i++)
        if (strcmp(ci->ppEnabledExtensionNames[i], "VK_EXT_display_control")) names[n++] = ci->ppEnabledExtensionNames[i];
    copy.enabledExtensionCount = n;
    copy.ppEnabledExtensionNames = names;
    VkDeviceQueueCreateInfo* queues = calloc(ci->queueCreateInfoCount + 1, sizeof(*queues));
    for (uint32_t i = 0; i < ci->queueCreateInfoCount; i++) {
        queues[i] = ci->pQueueCreateInfos[i];
        const VkBaseInStructure** link = (const VkBaseInStructure**)&queues[i].pNext;
        while (*link) {
            if ((*link)->sType == VK_STRUCTURE_TYPE_DEVICE_QUEUE_GLOBAL_PRIORITY_CREATE_INFO_KHR) {
                DBG("pid %d: queue family %u: dropping global priority %d", getpid(), queues[i].queueFamilyIndex,
                    ((const VkDeviceQueueGlobalPriorityCreateInfoKHR*)*link)->globalPriority);
                *link = (*link)->pNext;
            } else {
                link = (const VkBaseInStructure**)&(*link)->pNext;
            }
        }
    }
    copy.pQueueCreateInfos = queues;
    VkResult r = next(pd, &copy, alloc, out);
    free(queues);
    free(names);
    if (r != VK_SUCCESS) return r;
    Device* d = calloc(1, sizeof(*d));
    d->key = key(*out);
    d->handle = *out;
    d->gdpa = gdpa;
#define DF(field, name) d->field = (void*)gdpa(*out, name)
    DF(destroy, "vkDestroyDevice");
    DF(createSemaphore, "vkCreateSemaphore");
    DF(destroySemaphore, "vkDestroySemaphore");
    DF(counterValue, "vkGetSemaphoreCounterValue");
    if (!d->counterValue) DF(counterValue, "vkGetSemaphoreCounterValueKHR");
    DF(signalSemaphore, "vkSignalSemaphore");
    if (!d->signalSemaphore) DF(signalSemaphore, "vkSignalSemaphoreKHR");
    DF(waitSemaphores, "vkWaitSemaphores");
    if (!d->waitSemaphores) DF(waitSemaphores, "vkWaitSemaphoresKHR");
    DF(queueSubmit, "vkQueueSubmit");
    DF(queueSubmit2, "vkQueueSubmit2");
    if (!d->queueSubmit2) DF(queueSubmit2, "vkQueueSubmit2KHR");
    DF(getSemaphoreFd, "vkGetSemaphoreFdKHR");
    DF(importSemaphoreFd, "vkImportSemaphoreFdKHR");
    DF(getFenceStatus, "vkGetFenceStatus");
    DF(waitForFences, "vkWaitForFences");
    DF(acquireNextImage, "vkAcquireNextImageKHR");
    DF(queuePresent, "vkQueuePresentKHR");
#undef DF
    pthread_mutex_lock(&gLock);
    d->next = gDevices;
    gDevices = d;
    pthread_mutex_unlock(&gLock);
    return r;
}

static void VKAPI_CALL DestroyDevice(VkDevice device, const VkAllocationCallbacks* alloc) {
    if (!device) return;
    void* k = key(device);
    pthread_mutex_lock(&gLock);
    Device* found = NULL;
    for (Device** p = &gDevices; *p; p = &(*p)->next)
        if ((*p)->key == k) {
            found = *p;
            *p = found->next;
            break;
        }
    for (Shared** p = &gShared; *p;) {
        if ((*p)->device == found) {
            Shared* x = *p;
            *p = x->next;
            munmap((void*)x->counter, 4096);
            close(x->fd);
            free(x);
        } else {
            p = &(*p)->next;
        }
    }
    pthread_mutex_unlock(&gLock);
    if (found) {
        found->destroy(device, alloc);
        free(found);
    }
}

static VkResult VKAPI_CALL CreateSemaphore(VkDevice device, const VkSemaphoreCreateInfo* ci, const VkAllocationCallbacks* alloc, VkSemaphore* out) {
    Device* d = deviceFor(device);
    const VkExportSemaphoreCreateInfo* exp = findNext(ci->pNext, VK_STRUCTURE_TYPE_EXPORT_SEMAPHORE_CREATE_INFO);
    const VkSemaphoreTypeCreateInfo* type = findNext(ci->pNext, VK_STRUCTURE_TYPE_SEMAPHORE_TYPE_CREATE_INFO);
    const int sharedTimeline = exp && (exp->handleTypes & VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_FD_BIT) && type &&
                               type->semaphoreType == VK_SEMAPHORE_TYPE_TIMELINE;
    if (!sharedTimeline) return d->createSemaphore(device, ci, alloc, out);
    VkSemaphoreTypeCreateInfo t = *type;
    t.pNext = NULL;
    VkSemaphoreCreateInfo copy = *ci;
    copy.pNext = &t;
    VkResult r = d->createSemaphore(device, &copy, alloc, out);
    if (r != VK_SUCCESS) return r;
    int fd = newCounterFd();
    if (fd < 0) {
        d->destroySemaphore(device, *out, alloc);
        return VK_ERROR_OUT_OF_HOST_MEMORY;
    }
    addShared(d, *out, fd, t.initialValue);
    DBG("pid %d: shared timeline %p created (initial %llu)", getpid(), (void*)*out, (unsigned long long)t.initialValue);
    return VK_SUCCESS;
}

static void VKAPI_CALL DestroySemaphore(VkDevice device, VkSemaphore s, const VkAllocationCallbacks* alloc) {
    if (s) removeShared(s);
    deviceFor(device)->destroySemaphore(device, s, alloc);
}

static VkResult VKAPI_CALL GetSemaphoreFd(VkDevice device, const VkSemaphoreGetFdInfoKHR* info, int* fd) {
    DBG("pid %d: export semaphore %p handle type %#x", getpid(), (void*)info->semaphore, info->handleType);
    if (info->handleType == VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_FD_BIT) {
        Shared* x = sharedFor(info->semaphore);
        if (!x) return VK_ERROR_INVALID_EXTERNAL_HANDLE;
        *fd = fcntl(x->fd, F_DUPFD_CLOEXEC, 0);
        return *fd >= 0 ? VK_SUCCESS : VK_ERROR_TOO_MANY_OBJECTS;
    }
    return deviceFor(device)->getSemaphoreFd(device, info, fd);
}

static VkResult VKAPI_CALL ImportSemaphoreFd(VkDevice device, const VkImportSemaphoreFdInfoKHR* info) {
    if (info->handleType == VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_FD_BIT) {
        Device* d = deviceFor(device);
        removeShared(info->semaphore);
        uint64_t local = 0;
        d->counterValue(device, info->semaphore, &local);
        addShared(d, info->semaphore, info->fd, local);
        DBG("pid %d: shared timeline %p imported (local %llu)", getpid(), (void*)info->semaphore, (unsigned long long)local);
        return VK_SUCCESS;
    }
    DBG("pid %d: import semaphore %p handle type %#x flags %#x", getpid(), (void*)info->semaphore, info->handleType, info->flags);
    return deviceFor(device)->importSemaphoreFd(device, info);
}

static VkResult VKAPI_CALL WaitSemaphores(VkDevice device, const VkSemaphoreWaitInfo* info, uint64_t timeout) {
    VkResult r = deviceFor(device)->waitSemaphores(device, info, timeout);
    if (debugOn() && r != VK_SUCCESS) {
        for (uint32_t i = 0; i < info->semaphoreCount; i++) {
            Shared* x = sharedFor(info->pSemaphores[i]);
            uint64_t local = 0;
            deviceFor(device)->counterValue(device, info->pSemaphores[i], &local);
            logf_("pid %d: wait %p for %llu: %d (local %llu, shared %s %llu, pending %llu, timeout %llu)", getpid(), (void*)info->pSemaphores[i],
                  (unsigned long long)info->pValues[i], r, (unsigned long long)local, x ? "yes" : "no",
                  x ? (unsigned long long)atomic_load(x->counter) : 0ull, x ? (unsigned long long)atomic_load(&x->pendingSignal) : 0ull,
                  (unsigned long long)timeout);
        }
    }
    return r;
}

static void noteSignal(VkSemaphore s, uint64_t value) {
    Shared* x = sharedFor(s);
    if (x) atomicMax(&x->pendingSignal, value);
}

static _Atomic unsigned gSubmits, gFencePolls;

static VkResult VKAPI_CALL QueueSubmit(VkQueue queue, uint32_t count, const VkSubmitInfo* submits, VkFence fence) {
    if (debugOn() && atomic_fetch_add(&gSubmits, 1) < 40)
        for (uint32_t i = 0; i < count; i++) {
            const VkTimelineSemaphoreSubmitInfo* tl = findNext(submits[i].pNext, VK_STRUCTURE_TYPE_TIMELINE_SEMAPHORE_SUBMIT_INFO);
            logf_("pid %d: submit %u: %u cmdbufs, fence %p, waits %u, signals %u", getpid(), i, submits[i].commandBufferCount, (void*)fence,
                  submits[i].waitSemaphoreCount, submits[i].signalSemaphoreCount);
            for (uint32_t j = 0; j < submits[i].waitSemaphoreCount; j++)
                logf_("   wait %p value %llu shared %d", (void*)submits[i].pWaitSemaphores[j],
                      (unsigned long long)(tl && tl->pWaitSemaphoreValues && j < tl->waitSemaphoreValueCount ? tl->pWaitSemaphoreValues[j] : 0),
                      sharedFor(submits[i].pWaitSemaphores[j]) != NULL);
            for (uint32_t j = 0; j < submits[i].signalSemaphoreCount; j++)
                logf_("   signal %p value %llu shared %d", (void*)submits[i].pSignalSemaphores[j],
                      (unsigned long long)(tl && tl->pSignalSemaphoreValues && j < tl->signalSemaphoreValueCount ? tl->pSignalSemaphoreValues[j] : 0),
                      sharedFor(submits[i].pSignalSemaphores[j]) != NULL);
        }
    for (uint32_t i = 0; i < count; i++) {
        const VkTimelineSemaphoreSubmitInfo* tl = findNext(submits[i].pNext, VK_STRUCTURE_TYPE_TIMELINE_SEMAPHORE_SUBMIT_INFO);
        if (!tl || !tl->pSignalSemaphoreValues) continue;
        for (uint32_t j = 0; j < submits[i].signalSemaphoreCount && j < tl->signalSemaphoreValueCount; j++)
            noteSignal(submits[i].pSignalSemaphores[j], tl->pSignalSemaphoreValues[j]);
    }
    return deviceFor(queue)->queueSubmit(queue, count, submits, fence);
}

static VkResult VKAPI_CALL QueueSubmit2(VkQueue queue, uint32_t count, const VkSubmitInfo2* submits, VkFence fence) {
    for (uint32_t i = 0; i < count; i++)
        for (uint32_t j = 0; j < submits[i].signalSemaphoreInfoCount; j++)
            noteSignal(submits[i].pSignalSemaphoreInfos[j].semaphore, submits[i].pSignalSemaphoreInfos[j].value);
    return deviceFor(queue)->queueSubmit2(queue, count, submits, fence);
}

static VkResult VKAPI_CALL GetFenceStatus(VkDevice device, VkFence fence) {
    VkResult r = deviceFor(device)->getFenceStatus(device, fence);
    if (debugOn() && r != VK_SUCCESS) {
        unsigned n = atomic_fetch_add(&gFencePolls, 1);
        if (n < 3 || n % 20000 == 0) logf_("pid %d: fence %p not ready (%d), poll %u", getpid(), (void*)fence, r, n);
    }
    return r;
}

static VkResult VKAPI_CALL WaitForFences(VkDevice device, uint32_t n, const VkFence* fences, VkBool32 all, uint64_t timeout) {
    VkResult r = deviceFor(device)->waitForFences(device, n, fences, all, timeout);
    if (debugOn() && r != VK_SUCCESS) logf_("pid %d: wait for %u fences (first %p) timeout %llu: %d", getpid(), n, (void*)fences[0], (unsigned long long)timeout, r);
    return r;
}

static VkResult VKAPI_CALL AcquireNextImage(VkDevice device, VkSwapchainKHR sc, uint64_t timeout, VkSemaphore sem, VkFence fence, uint32_t* index) {
    VkResult r = deviceFor(device)->acquireNextImage(device, sc, timeout, sem, fence, index);
    DBG("pid %d: acquire swapchain %p timeout %llu sem %p fence %p: %d index %u", getpid(), (void*)sc, (unsigned long long)timeout, (void*)sem, (void*)fence, r, *index);
    return r;
}

static VkResult VKAPI_CALL QueuePresent(VkQueue queue, const VkPresentInfoKHR* info) {
    VkResult r = deviceFor(queue)->queuePresent(queue, info);
    DBG("pid %d: present %u swapchains, %u waits: %d", getpid(), info->swapchainCount, info->waitSemaphoreCount, r);
    return r;
}

static VkResult VKAPI_CALL SignalSemaphore(VkDevice device, const VkSemaphoreSignalInfo* info) {
    Shared* x = sharedFor(info->semaphore);
    if (x) atomicMax(&x->pendingSignal, info->value);
    VkResult r = deviceFor(device)->signalSemaphore(device, info);
    if (x && r == VK_SUCCESS) atomicMax(x->counter, info->value);
    return r;
}

static VkResult VKAPI_CALL GetSemaphoreCounterValue(VkDevice device, VkSemaphore s, uint64_t* value) {
    VkResult r = deviceFor(device)->counterValue(device, s, value);
    Shared* x = sharedFor(s);
    if (x && r == VK_SUCCESS) {
        const uint64_t shared = atomic_load(x->counter);
        if (shared > *value) *value = shared;
    }
    return r;
}

static VkResult VKAPI_CALL NoneCount(VkPhysicalDevice pd, uint32_t* count, void* props) {
    (void)pd;
    (void)props;
    *count = 0;
    return VK_SUCCESS;
}
static VkResult VKAPI_CALL NoneSupportedDisplays(VkPhysicalDevice pd, uint32_t plane, uint32_t* count, VkDisplayKHR* displays) {
    (void)pd;
    (void)plane;
    (void)displays;
    *count = 0;
    return VK_SUCCESS;
}
static VkResult VKAPI_CALL NoneModes(VkPhysicalDevice pd, VkDisplayKHR d, uint32_t* count, void* props) {
    (void)pd;
    (void)d;
    (void)props;
    *count = 0;
    return VK_SUCCESS;
}
static VkResult VKAPI_CALL Fail4(void* a, void* b, void* c, void* d) {
    (void)a;
    (void)b;
    (void)c;
    (void)d;
    return VK_ERROR_INITIALIZATION_FAILED;
}
static VkResult VKAPI_CALL ReleaseDisplay(VkPhysicalDevice pd, VkDisplayKHR d) {
    (void)pd;
    (void)d;
    return VK_SUCCESS;
}
static VkResult VKAPI_CALL GetDrmDisplay(VkPhysicalDevice pd, int32_t fd, uint32_t connector, VkDisplayKHR* display) {
    (void)pd;
    (void)fd;
    (void)connector;
    *display = VK_NULL_HANDLE;
    return VK_ERROR_INITIALIZATION_FAILED;
}
static VkResult VKAPI_CALL GetRandROutputDisplay(VkPhysicalDevice pd, void* dpy, unsigned long output, VkDisplayKHR* display) {
    (void)pd;
    (void)dpy;
    (void)output;
    *display = VK_NULL_HANDLE;
    return VK_ERROR_INITIALIZATION_FAILED;
}

static PFN_vkVoidFunction displayFunction(const char* name) {
    if (!strcmp(name, "vkGetPhysicalDeviceDisplayPropertiesKHR") || !strcmp(name, "vkGetPhysicalDeviceDisplayProperties2KHR") ||
        !strcmp(name, "vkGetPhysicalDeviceDisplayPlanePropertiesKHR") || !strcmp(name, "vkGetPhysicalDeviceDisplayPlaneProperties2KHR"))
        return (PFN_vkVoidFunction)NoneCount;
    if (!strcmp(name, "vkGetDisplayPlaneSupportedDisplaysKHR")) return (PFN_vkVoidFunction)NoneSupportedDisplays;
    if (!strcmp(name, "vkGetDisplayModePropertiesKHR") || !strcmp(name, "vkGetDisplayModeProperties2KHR")) return (PFN_vkVoidFunction)NoneModes;
    if (!strcmp(name, "vkCreateDisplayModeKHR") || !strcmp(name, "vkGetDisplayPlaneCapabilitiesKHR") ||
        !strcmp(name, "vkGetDisplayPlaneCapabilities2KHR") || !strcmp(name, "vkCreateDisplayPlaneSurfaceKHR") ||
        !strcmp(name, "vkAcquireXlibDisplayEXT") || !strcmp(name, "vkAcquireDrmDisplayEXT") ||
        !strcmp(name, "vkGetPhysicalDeviceSurfaceCapabilities2EXT"))
        return (PFN_vkVoidFunction)Fail4;
    if (!strcmp(name, "vkReleaseDisplayEXT")) return (PFN_vkVoidFunction)ReleaseDisplay;
    if (!strcmp(name, "vkGetDrmDisplayEXT")) return (PFN_vkVoidFunction)GetDrmDisplay;
    if (!strcmp(name, "vkGetRandROutputDisplayEXT")) return (PFN_vkVoidFunction)GetRandROutputDisplay;
    return NULL;
}

// --- Dispatch -------------------------------------------------------------------------------------
static PFN_vkVoidFunction deviceFunction(const char* name) {
#define F(n, f) \
    if (!strcmp(name, n)) return (PFN_vkVoidFunction)f
    F("vkDestroyDevice", DestroyDevice);
    F("vkCreateSemaphore", CreateSemaphore);
    F("vkDestroySemaphore", DestroySemaphore);
    F("vkGetSemaphoreFdKHR", GetSemaphoreFd);
    F("vkImportSemaphoreFdKHR", ImportSemaphoreFd);
    F("vkQueueSubmit", QueueSubmit);
    F("vkQueueSubmit2", QueueSubmit2);
    F("vkQueueSubmit2KHR", QueueSubmit2);
    F("vkSignalSemaphore", SignalSemaphore);
    F("vkSignalSemaphoreKHR", SignalSemaphore);
    F("vkGetSemaphoreCounterValue", GetSemaphoreCounterValue);
    F("vkWaitSemaphores", WaitSemaphores);
    F("vkGetFenceStatus", GetFenceStatus);
    F("vkWaitForFences", WaitForFences);
    F("vkAcquireNextImageKHR", AcquireNextImage);
    F("vkQueuePresentKHR", QueuePresent);
    F("vkWaitSemaphoresKHR", WaitSemaphores);
    F("vkGetSemaphoreCounterValueKHR", GetSemaphoreCounterValue);
#undef F
    return NULL;
}

static PFN_vkVoidFunction VKAPI_CALL GetDeviceProcAddr(VkDevice device, const char* name) {
    if (!strcmp(name, "vkGetDeviceProcAddr")) return (PFN_vkVoidFunction)GetDeviceProcAddr;
    Device* d = deviceFor(device);
    if (!d) return NULL;
    PFN_vkVoidFunction next = d->gdpa(device, name);
    PFN_vkVoidFunction mine = deviceFunction(name);
    return (mine && next) ? mine : next;
}

EXPORT PFN_vkVoidFunction VKAPI_CALL QuestDeckVr_GetInstanceProcAddr(VkInstance instance, const char* name) {
    if (!strcmp(name, "vkGetInstanceProcAddr")) return (PFN_vkVoidFunction)QuestDeckVr_GetInstanceProcAddr;
    if (!strcmp(name, "vkCreateInstance")) return (PFN_vkVoidFunction)CreateInstance;
    if (!strcmp(name, "vkDestroyInstance")) return (PFN_vkVoidFunction)DestroyInstance;
    if (!strcmp(name, "vkCreateDevice")) return (PFN_vkVoidFunction)CreateDevice;
    if (!strcmp(name, "vkGetDeviceProcAddr")) return (PFN_vkVoidFunction)GetDeviceProcAddr;
    if (!strcmp(name, "vkGetPhysicalDeviceExternalSemaphoreProperties")) return (PFN_vkVoidFunction)GetExternalSemaphoreProperties;
    if (!strcmp(name, "vkGetPhysicalDeviceExternalSemaphorePropertiesKHR")) return (PFN_vkVoidFunction)GetExternalSemaphorePropertiesKhr;
    PFN_vkVoidFunction f = displayFunction(name);
    if (f) return f;
    if (instance) {
        PFN_vkVoidFunction d = deviceFunction(name);
        if (d) return d;
        Instance* i = instanceFor(instance);
        return i ? i->gipa(instance, name) : NULL;
    }
    return NULL;
}

EXPORT VkResult VKAPI_CALL QuestDeckVr_NegotiateLoaderLayerInterfaceVersion(VkNegotiateLayerInterface* v) {
    if (v->loaderLayerInterfaceVersion > 2) v->loaderLayerInterfaceVersion = 2;
    v->pfnGetInstanceProcAddr = QuestDeckVr_GetInstanceProcAddr;
    v->pfnGetDeviceProcAddr = GetDeviceProcAddr;
    v->pfnGetPhysicalDeviceProcAddr = NULL;
    return VK_SUCCESS;
}
