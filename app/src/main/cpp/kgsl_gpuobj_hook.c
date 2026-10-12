/*
 * ioctl() for the app's own Turnip, with KGSL's legacy GPU-memory ioctls translated where the
 * kernel denies them (issue #245; the translation is kgsl_gpuobj_compat.h).
 *
 * adrenotools loads this into the driver's linker namespace as a global library before the driver
 * (ADRENOTOOLS_DRIVER_KGSL_GPUOBJ, see hook_impl.cpp), so the driver's own ioctl import resolves
 * here rather than to libc. This is the second Turnip in a session: the glibc one in the guest is
 * covered by libblsession.so, but the compositor in the app process loads a bionic build, and
 * when its first allocation fails every dmabuf import after it fails with nothing in the log
 * ("0 windows open").
 *
 * The timestamp read Horizon OS also denies is answered from the memstore
 * (kgsl_timestamp_compat.h).
 *
 * The real call is the syscall itself. Anything that is not one of the two legacy allocation
 * ioctls or the timestamp read goes straight through.
 */
#define _GNU_SOURCE
#include <android/log.h>
#include <stdarg.h>
#include <sys/syscall.h>
#include <unistd.h>

#define DD_KGSL_NOTE(msg) \
    __android_log_print(ANDROID_LOG_WARN, "kgsl_gpuobj_hook", "%s", msg)

#include "kgsl_gpuobj_compat.h"
#include "kgsl_timestamp_compat.h"

static int raw_ioctl(int fd, unsigned long request, void *arg) {
    return (int)syscall(SYS_ioctl, fd, (unsigned int)request, arg);
}

__attribute__((visibility("default"))) int ioctl(int fd, int request, ...) {
    va_list ap;
    void *arg;
    int rc;

    va_start(ap, request);
    arg = va_arg(ap, void *);
    va_end(ap);
    if (dd_kgsl_gpuobj_compat(raw_ioctl, fd, (unsigned long)(unsigned int)request, arg, &rc))
        return rc;
    if (dd_kgsl_timestamp_compat(raw_ioctl, fd, (unsigned long)(unsigned int)request, arg, &rc))
        return rc;
    return raw_ioctl(fd, (unsigned long)(unsigned int)request, arg);
}
