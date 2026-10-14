//
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <linux/futex.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/sem.h>
#include <sys/stat.h>
#include <signal.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

#define MAX_SEMS 64

static void trace(const char* fmt, ...) {
    static int fd = -2;
    if (fd == -2) {
        const char* p = getenv("QUESTDECK_SEM_LOG");
        fd = p ? open(p, O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0644) : -1;
    }
    if (fd < 0) return;
    char buf[256];
    int n = snprintf(buf, sizeof(buf), "%d ", getpid());
    va_list ap;
    va_start(ap, fmt);
    n += vsnprintf(buf + n, sizeof(buf) - (size_t)n, fmt, ap);
    va_end(ap);
    if (n > (int)sizeof(buf) - 2) n = (int)sizeof(buf) - 2;
    buf[n++] = '\n';
    (void)!write(fd, buf, (size_t)n);
}
#define MAX_SETS 256

struct set {
    uint32_t magic;
    uint32_t nsems;
    int64_t otime, ctime;
    int32_t pid[MAX_SEMS];
    int32_t value[MAX_SEMS]; // futex words
    uint32_t mode;
    int32_t holder[MAX_SEMS];
    int32_t held[MAX_SEMS];
};
#define MAGIC 0x51445356u  // "QDSV"

static struct {
    struct set* s;
    key_t key;
    char path[64];
} sets[MAX_SETS];

static void pathFor(key_t key, char* out, size_t n) { snprintf(out, n, "/dev/shm/qd-sysvsem-%08x", (unsigned)key); }

static struct set* get(int id) {
    if (id < 0 || id >= MAX_SETS || !sets[id].s) {
        errno = EINVAL;
        return NULL;
    }
    return sets[id].s;
}

static int semget_(key_t key, int nsems, int flags);
__attribute__((visibility("default"))) int semget(key_t key, int nsems, int flags) {
    int r = semget_(key, nsems, flags);
    trace("semget key %#x n %d flags %#o -> %d (%s)", (unsigned)key, nsems, flags, r, r < 0 ? strerror(errno) : "ok");
    return r;
}
static int semget_(key_t key, int nsems, int flags) {
    if (nsems < 0 || nsems > MAX_SEMS) {
        errno = EINVAL;
        return -1;
    }
    char path[64];
    static unsigned privateCount;
    if (key == IPC_PRIVATE)
        snprintf(path, sizeof(path), "/dev/shm/qd-sysvsem-p%d-%u", getpid(), privateCount++);
    else
        pathFor(key, path, sizeof(path));
    int oflags = O_RDWR | O_CLOEXEC;
    if (flags & IPC_CREAT) oflags |= O_CREAT;
    if ((flags & IPC_CREAT) && (flags & IPC_EXCL)) oflags |= O_EXCL;
    int fd = open(path, oflags, 0666);
    if (fd < 0) return -1;
    struct stat st;
    int created = fstat(fd, &st) == 0 && st.st_size == 0;
    if (created && ftruncate(fd, sizeof(struct set)) != 0) {
        close(fd);
        return -1;
    }
    struct set* s = mmap(NULL, sizeof(struct set), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (s == MAP_FAILED) return -1;
    if (created || __atomic_load_n(&s->magic, __ATOMIC_ACQUIRE) != MAGIC) {
        s->nsems = (uint32_t)(nsems ? nsems : 1);
        s->mode = (uint32_t)(flags & 0777);
        s->ctime = time(NULL);
        __atomic_store_n(&s->magic, MAGIC, __ATOMIC_RELEASE);
    } else if ((uint32_t)nsems > s->nsems) {
        munmap(s, sizeof(struct set));
        errno = EINVAL;
        return -1;
    }
    for (int i = 0; i < MAX_SETS; i++)
        if (sets[i].s && !strcmp(sets[i].path, path)) {  // already mapped here
            munmap(s, sizeof(struct set));
            return i;
        }
    for (int i = 0; i < MAX_SETS; i++)
        if (!sets[i].s) {
            sets[i].s = s;
            sets[i].key = key;
            snprintf(sets[i].path, sizeof(sets[i].path), "%s", path);
            return i;
        }
    munmap(s, sizeof(struct set));
    errno = ENOSPC;
    return -1;
}

static void wake(int32_t* w) { syscall(SYS_futex, w, FUTEX_WAKE, INT_MAX, NULL, NULL, 0); }

static int recoverDeadHolder(struct set* s, unsigned short k) {
    int32_t holder = __atomic_load_n(&s->holder[k], __ATOMIC_ACQUIRE);
    if (holder <= 0 || holder == getpid() || kill(holder, 0) == 0 || errno != ESRCH) return 0;
    if (!__atomic_compare_exchange_n(&s->holder[k], &holder, 0, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) return 0;
    __atomic_fetch_add(&s->value[k], s->held[k], __ATOMIC_ACQ_REL);
    s->held[k] = 0;
    wake(&s->value[k]);
    return 1;
}

__attribute__((visibility("default"))) int semctl(int id, int num, int cmd, ...) {
    va_list ap;
    va_start(ap, cmd);
    union {
        int val;
        struct semid_ds* buf;
        unsigned short* array;
    } arg;
    arg.buf = va_arg(ap, struct semid_ds*);
    va_end(ap);
    trace("semctl id %d num %d cmd %d val %d", id, num, cmd, arg.val);
    struct set* s = get(id);
    if (!s) return -1;
    if ((cmd == GETVAL || cmd == SETVAL || cmd == GETPID || cmd == GETNCNT || cmd == GETZCNT) && (num < 0 || (uint32_t)num >= s->nsems)) {
        errno = EINVAL;
        return -1;
    }
    switch (cmd) {
        case IPC_RMID:
            unlink(sets[id].path);
            __atomic_store_n(&s->magic, 0u, __ATOMIC_RELEASE);
            for (uint32_t i = 0; i < s->nsems; i++) wake(&s->value[i]);
            return 0;
        case IPC_STAT:
        case SEM_STAT:
            if (!arg.buf) {
                errno = EFAULT;
                return -1;
            }
            memset(arg.buf, 0, sizeof(*arg.buf));
            arg.buf->sem_perm.uid = arg.buf->sem_perm.cuid = getuid();
            arg.buf->sem_perm.gid = arg.buf->sem_perm.cgid = getgid();
            arg.buf->sem_perm.mode = s->mode;
            arg.buf->sem_perm.__key = sets[id].key;
            arg.buf->sem_otime = s->otime;
            arg.buf->sem_ctime = s->ctime;
            arg.buf->sem_nsems = s->nsems;
            return 0;
        case IPC_SET:
            return 0;
        case GETVAL:
            return __atomic_load_n(&s->value[num], __ATOMIC_ACQUIRE);
        case GETPID:
            return s->pid[num];
        case GETNCNT:
        case GETZCNT:
            return 0;
        case SETVAL:
            __atomic_store_n(&s->value[num], arg.val, __ATOMIC_RELEASE);
            s->ctime = time(NULL);
            wake(&s->value[num]);
            return 0;
        case GETALL:
            for (uint32_t i = 0; i < s->nsems; i++) arg.array[i] = (unsigned short)__atomic_load_n(&s->value[i], __ATOMIC_ACQUIRE);
            return 0;
        case SETALL:
            for (uint32_t i = 0; i < s->nsems; i++) {
                __atomic_store_n(&s->value[i], arg.array[i], __ATOMIC_RELEASE);
                wake(&s->value[i]);
            }
            s->ctime = time(NULL);
            return 0;
        default:
            errno = EINVAL;
            return -1;
    }
}

static int apply(struct set* s, struct sembuf* ops, size_t n, int* waitIndex) {
    for (size_t i = 0; i < n; i++) {
        const int32_t v = __atomic_load_n(&s->value[ops[i].sem_num], __ATOMIC_ACQUIRE);
        if ((ops[i].sem_op < 0 && v < -ops[i].sem_op) || (ops[i].sem_op == 0 && v != 0)) {
            *waitIndex = (int)i;
            return 0;
        }
    }
    for (size_t i = 0; i < n; i++) {
        int32_t* w = &s->value[ops[i].sem_num];
        if (ops[i].sem_op == 0) continue;
        int32_t cur = __atomic_load_n(w, __ATOMIC_ACQUIRE);
        for (;;) {
            const int32_t next = cur + ops[i].sem_op;
            if (next < 0) {
                for (size_t j = 0; j < i; j++)
                    if (ops[j].sem_op) __atomic_fetch_sub(&s->value[ops[j].sem_num], ops[j].sem_op, __ATOMIC_ACQ_REL);
                *waitIndex = (int)i;
                return 0;
            }
            if (__atomic_compare_exchange_n(w, &cur, next, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) break;
        }
    }
    for (size_t i = 0; i < n; i++) {
        const unsigned short k = ops[i].sem_num;
        s->pid[k] = getpid();
        if (ops[i].sem_flg & SEM_UNDO) {
            if (ops[i].sem_op < 0) {
                s->holder[k] = getpid();
                s->held[k] = -ops[i].sem_op;
            } else if (ops[i].sem_op > 0 && s->holder[k] == getpid()) {
                s->holder[k] = 0;
                s->held[k] = 0;
            }
        }
        if (ops[i].sem_op > 0) wake(&s->value[k]);
    }
    s->otime = time(NULL);
    return 1;
}

static int semtimedop_(int id, struct sembuf* ops, size_t n, const struct timespec* timeout);
__attribute__((visibility("default"))) int semtimedop(int id, struct sembuf* ops, size_t n, const struct timespec* timeout) {
    trace("semop id %d n %zu op0 %d/%d flg %#x timeout %d", id, n, n ? ops[0].sem_num : -1, n ? ops[0].sem_op : 0, n ? ops[0].sem_flg : 0, timeout != NULL);
    int r = semtimedop_(id, ops, n, timeout);
    trace("semop id %d -> %d (%s)", id, r, r < 0 ? strerror(errno) : "ok");
    return r;
}
static int semtimedop_(int id, struct sembuf* ops, size_t n, const struct timespec* timeout) {
    struct set* s = get(id);
    if (!s) return -1;
    for (size_t i = 0; i < n; i++)
        if (ops[i].sem_num >= s->nsems) {
            errno = EFBIG;
            return -1;
        }
    struct timespec deadline;
    if (timeout) {
        clock_gettime(CLOCK_MONOTONIC, &deadline);
        deadline.tv_sec += timeout->tv_sec;
        deadline.tv_nsec += timeout->tv_nsec;
        if (deadline.tv_nsec >= 1000000000) {
            deadline.tv_sec++;
            deadline.tv_nsec -= 1000000000;
        }
    }
    for (;;) {
        int wi = -1;
        if (apply(s, ops, n, &wi)) return 0;
        if (recoverDeadHolder(s, ops[wi].sem_num)) continue;
        if (ops[wi].sem_flg & IPC_NOWAIT) {
            errno = EAGAIN;
            return -1;
        }
        int32_t* w = &s->value[ops[wi].sem_num];
        const int32_t seen = __atomic_load_n(w, __ATOMIC_ACQUIRE);
        struct timespec rel = {0, 50000000}, *relp = &rel;
        if (timeout) {
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            int64_t left = (deadline.tv_sec - now.tv_sec) * 1000000000LL + (deadline.tv_nsec - now.tv_nsec);
            if (left <= 0) {
                errno = EAGAIN;
                return -1;
            }
            if (left < 50000000) rel.tv_nsec = left;
        }
        syscall(SYS_futex, w, FUTEX_WAIT, seen, relp, NULL, 0);
        recoverDeadHolder(s, ops[wi].sem_num);
        if (__atomic_load_n(&s->magic, __ATOMIC_ACQUIRE) != MAGIC) {
            errno = EIDRM;
            return -1;
        }
    }
}

__attribute__((visibility("default"))) int semop(int id, struct sembuf* ops, size_t n) { return semtimedop(id, ops, n, NULL); }
