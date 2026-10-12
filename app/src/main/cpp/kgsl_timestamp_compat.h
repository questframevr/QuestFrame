/*
 * KGSL's timestamp read answered from the memstore, for kernels whose SELinux policy refuses it
 * to apps.
 *
 * Meta's Horizon OS (Quest 3 / 3S) denies IOCTL_KGSL_CMDSTREAM_READTIMESTAMP_CTXTID (0x916) to an
 * app domain with EACCES. Turnip's KGSL backend reads a context's retired timestamp with it
 * whenever it asks "has this finished?" without blocking (kgsl_poll_timestamp: a zero-timeout
 * wait, vkGetFenceStatus, a timeline semaphore's value), and turns the refusal into
 * VK_ERROR_UNKNOWN. Recent Turnips (Mesa 26.x, e.g. Mesa_Turnip_v26.3.0-20261010) take that path
 * from gamescope's very first frame: vk.WaitSemaphores fails with VkResult -13 and the session
 * ends. Older drivers waited instead of reading and never met the denial.
 *
 * KGSL publishes the same timestamps in the memstore, one page per device that userspace may map
 * read-only (KGSL_PROP_DEVICE_SHADOW gives its size, and its GPU address is the mmap offset), with
 * a struct kgsl_devmemstore per context id: soptimestamp is CONSUMED, eoptimestamp is RETIRED.
 * After the kernel refuses a read with EACCES the answer comes from there; QUEUED, which the
 * memstore does not keep, is answered with RETIRED, the most a reader can rely on anyway.
 *
 * The kernel is always asked first, so a device that allows the read never takes this path.
 *
 * Header-only, with its own copies of the uapi structs (layout: msm_kgsl.h), so it builds in the
 * glibc preload (tools/linuxfs/preload/kgslpoll.c) and the bionic hook (kgsl_gpuobj_hook.c) alike.
 * The caller supplies the ioctl to use; nothing here calls the C library's.
 *
 * BL_KGSL_TIMESTAMP=0 turns it off.
 */
#ifndef DROIDDECK_KGSL_TIMESTAMP_COMPAT_H
#define DROIDDECK_KGSL_TIMESTAMP_COMPAT_H

#include <errno.h>
#include <pthread.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/types.h>

#ifndef DD_KGSL_NOTE
#define DD_KGSL_NOTE(msg) ((void)0) /* the includer may log the one-time switch-over */
#endif

#define DD_KGSL_TS_IOC_TYPE 0x09
#define DD_KGSL_PROP_DEVICE_SHADOW 0x2
#define DD_KGSL_TIMESTAMP_CONSUMED 0x1

struct dd_kgsl_device_getproperty {
    unsigned int type;
    void *value;
    size_t sizebytes;
};

struct dd_kgsl_shadowprop {
    unsigned long gpuaddr;
    size_t size;
    unsigned int flags;
};

struct dd_kgsl_cmdstream_readtimestamp_ctxtid {
    unsigned int context_id;
    unsigned int type;
    unsigned int timestamp;
};

struct dd_kgsl_devmemstore {
    volatile unsigned int soptimestamp;
    unsigned int sbz;
    volatile unsigned int eoptimestamp;
    unsigned int sbz2;
    volatile unsigned int preempted;
    unsigned int sbz3;
    volatile unsigned int ref_wait_ts;
    unsigned int sbz4;
    unsigned int current_context;
    unsigned int sbz5;
};

#define DD_IOCTL_KGSL_DEVICE_GETPROPERTY \
    _IOWR(DD_KGSL_TS_IOC_TYPE, 0x2, struct dd_kgsl_device_getproperty)
#define DD_IOCTL_KGSL_CMDSTREAM_READTIMESTAMP_CTXTID \
    _IOWR(DD_KGSL_TS_IOC_TYPE, 0x16, struct dd_kgsl_cmdstream_readtimestamp_ctxtid)

typedef int (*dd_kgsl_ts_ioctl_fn)(int, unsigned long, void *);

/* The device's memstore, mapped on the first refused read. It is one page set per device, not
 * per fd, so one mapping serves every fd of the process. */
static const struct dd_kgsl_devmemstore *dd_kgsl_memstore;
static size_t dd_kgsl_memstore_size;
static int dd_kgsl_memstore_tried;
static pthread_mutex_t dd_kgsl_memstore_lock = PTHREAD_MUTEX_INITIALIZER;

static const struct dd_kgsl_devmemstore *dd_kgsl_memstore_map(dd_kgsl_ts_ioctl_fn real, int fd,
                                                              size_t *size) {
    pthread_mutex_lock(&dd_kgsl_memstore_lock);
    if (!dd_kgsl_memstore_tried) {
        struct dd_kgsl_shadowprop shadow = {0};
        struct dd_kgsl_device_getproperty prop = {DD_KGSL_PROP_DEVICE_SHADOW, &shadow,
                                                  sizeof(shadow)};
        dd_kgsl_memstore_tried = 1;
        if (real(fd, DD_IOCTL_KGSL_DEVICE_GETPROPERTY, &prop) == 0 && shadow.size) {
            void *p = mmap(NULL, shadow.size, PROT_READ, MAP_SHARED, fd, (off_t)shadow.gpuaddr);
            if (p != MAP_FAILED) {
                dd_kgsl_memstore = p;
                dd_kgsl_memstore_size = shadow.size;
                DD_KGSL_NOTE("KGSL timestamp read denied; answering it from the memstore");
            } else {
                DD_KGSL_NOTE("KGSL timestamp read denied and the memstore could not be mapped");
            }
        }
    }
    *size = dd_kgsl_memstore_size;
    pthread_mutex_unlock(&dd_kgsl_memstore_lock);
    return dd_kgsl_memstore;
}

static int dd_kgsl_timestamp_enabled(void) {
    static int cached = -1;
    if (cached < 0) {
        const char *v = getenv("BL_KGSL_TIMESTAMP");
        cached = !(v != NULL && v[0] == '0');
    }
    return cached;
}

/* A READTIMESTAMP_CTXTID, with the memstore's answer where the kernel refuses it. Returns the
 * ioctl's result, with errno set on failure. */
static int dd_kgsl_read_timestamp(dd_kgsl_ts_ioctl_fn real, int fd,
                                  struct dd_kgsl_cmdstream_readtimestamp_ctxtid *rt) {
    int rc = real(fd, DD_IOCTL_KGSL_CMDSTREAM_READTIMESTAMP_CTXTID, rt);
    if (rc == 0 || errno != EACCES || !dd_kgsl_timestamp_enabled()) return rc;

    size_t size = 0;
    const struct dd_kgsl_devmemstore *m = dd_kgsl_memstore_map(real, fd, &size);
    if (m == NULL || ((size_t)rt->context_id + 1) * sizeof(*m) > size) {
        errno = EACCES;
        return rc;
    }
    rt->timestamp = rt->type == DD_KGSL_TIMESTAMP_CONSUMED ? m[rt->context_id].soptimestamp
                                                           : m[rt->context_id].eoptimestamp;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    return 0;
}

/* True when the call was a timestamp read, answered here with its result in *rc. */
static int dd_kgsl_timestamp_compat(dd_kgsl_ts_ioctl_fn real, int fd, unsigned long request,
                                    void *arg, int *rc) {
    if (request != DD_IOCTL_KGSL_CMDSTREAM_READTIMESTAMP_CTXTID || arg == NULL) return 0;
    *rc = dd_kgsl_read_timestamp(real, fd, arg);
    return 1;
}

#endif
