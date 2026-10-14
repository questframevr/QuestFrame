//
//
#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <pthread.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#define KGSL_IOC_TYPE 0x09
#define KGSL_PROP_DEVICE_SHADOW 0x2
#define KGSL_TIMESTAMP_CONSUMED 0x1

struct kgsl_device_getproperty {
    unsigned int type;
    void* value;
    size_t sizebytes;
};
struct kgsl_shadowprop {
    unsigned long gpuaddr;
    size_t size;
    unsigned int flags;
};
struct kgsl_cmdstream_readtimestamp_ctxtid {
    unsigned int context_id;
    unsigned int type;
    unsigned int timestamp;
};
struct kgsl_devmemstore {
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

#define IOCTL_KGSL_DEVICE_GETPROPERTY _IOWR(KGSL_IOC_TYPE, 0x2, struct kgsl_device_getproperty)
#define IOCTL_KGSL_CMDSTREAM_READTIMESTAMP_CTXTID _IOWR(KGSL_IOC_TYPE, 0x16, struct kgsl_cmdstream_readtimestamp_ctxtid)

typedef int (*ioctl_fn)(int, unsigned long, ...);
static ioctl_fn real;

#define MAX_FDS 1024
static struct {
    const struct kgsl_devmemstore* base;
    size_t size;
    int tried;
} maps[MAX_FDS];
static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;

static const struct kgsl_devmemstore* memstore(int fd, size_t* size) {
    if (fd < 0 || fd >= MAX_FDS) return NULL;
    pthread_mutex_lock(&lock);
    if (!maps[fd].tried) {
        maps[fd].tried = 1;
        struct kgsl_shadowprop shadow = {0};
        struct kgsl_device_getproperty prop = {KGSL_PROP_DEVICE_SHADOW, &shadow, sizeof(shadow)};
        if (real(fd, IOCTL_KGSL_DEVICE_GETPROPERTY, &prop) == 0 && shadow.size) {
            void* p = mmap(NULL, shadow.size, PROT_READ, MAP_SHARED, fd, (off_t)shadow.gpuaddr);
            if (p != MAP_FAILED) {
                maps[fd].base = p;
                maps[fd].size = shadow.size;
            } else {
                fprintf(stderr, "[questdeck-kgslts] memstore mmap failed (%d)\n", errno);
            }
        }
    }
    const struct kgsl_devmemstore* m = maps[fd].base;
    *size = maps[fd].size;
    pthread_mutex_unlock(&lock);
    return m;
}

__attribute__((visibility("default"))) int ioctl(int fd, unsigned long request, ...) {
    va_list ap;
    va_start(ap, request);
    void* arg = va_arg(ap, void*);
    va_end(ap);
    if (!real) real = (ioctl_fn)dlsym(RTLD_NEXT, "ioctl");
    const int r = real(fd, request, arg);
    if (r == 0 || errno != EACCES || request != IOCTL_KGSL_CMDSTREAM_READTIMESTAMP_CTXTID || !arg) return r;
    struct kgsl_cmdstream_readtimestamp_ctxtid* rt = arg;
    size_t size = 0;
    const struct kgsl_devmemstore* m = memstore(fd, &size);
    if (!m || (rt->context_id + 1) * sizeof(*m) > size) {
        errno = EACCES;
        return r;
    }
    rt->timestamp = rt->type == KGSL_TIMESTAMP_CONSUMED ? m[rt->context_id].soptimestamp : m[rt->context_id].eoptimestamp;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    return 0;
}
