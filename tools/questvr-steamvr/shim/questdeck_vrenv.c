#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <stdarg.h>
#include <sys/syscall.h>
#include <signal.h>
#include <execinfo.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <linux/futex.h>
#include <pthread.h>
#include <stdint.h>
#include <sys/uio.h>

#ifndef QUESTDECK_GAME
__attribute__((visibility("default"))) int memfd_create(const char* name, unsigned int flags) {
    (void)name;
    (void)flags;
    errno = ENOSYS;
    return -1;
}
#endif

static struct robust_list_head* thread_robust_head(void) {
    pthread_mutexattr_t attr;
    pthread_mutex_t mutex;
    struct robust_list_head* head = NULL;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setrobust(&attr, PTHREAD_MUTEX_ROBUST);
    pthread_mutex_init(&mutex, &attr);
    pthread_mutexattr_destroy(&attr);
    if (pthread_mutex_lock(&mutex) != 0) return NULL;
    uintptr_t lo = (uintptr_t)&mutex, hi = lo + sizeof(mutex);
    for (uintptr_t p = lo; p + sizeof(void*) <= hi && !head; p += sizeof(void*)) {
        struct robust_list_head* candidate = *(struct robust_list_head**)p;
        struct robust_list_head copy;
        struct iovec local = {&copy, sizeof(copy)}, remote = {candidate, sizeof(copy)};
        if ((uintptr_t)candidate & (sizeof(void*) - 1)) continue;
        if (process_vm_readv(getpid(), &local, 1, &remote, 1, 0) != (ssize_t)sizeof(copy)) continue;
        uintptr_t entry = (uintptr_t)copy.list.next;
        if (entry >= lo && entry < hi && copy.futex_offset < 0 && copy.futex_offset > -256) head = candidate;
    }
    pthread_mutex_unlock(&mutex);
    pthread_mutex_destroy(&mutex);
    return head;
}

__attribute__((visibility("default"))) long syscall(long n, ...) {
    static long (*real)(long, ...);
    if (!real) real = (long (*)(long, ...))dlsym(RTLD_NEXT, "syscall");
#ifndef QUESTDECK_GAME
    if (n == SYS_memfd_create) {
        errno = ENOSYS;
        return -1;
    }
#endif
    va_list ap;
    va_start(ap, n);
    long a1 = va_arg(ap, long), a2 = va_arg(ap, long), a3 = va_arg(ap, long), a4 = va_arg(ap, long), a5 = va_arg(ap, long),
         a6 = va_arg(ap, long);
    va_end(ap);
    if (n == SYS_get_robust_list && (a1 == 0 || a1 == (long)gettid()) && a2 && a3) {
        long r = real(n, a1, a2, a3, a4, a5, a6);
        if (r == 0 && *(void**)a2) return r;
        struct robust_list_head* head = thread_robust_head();
        if (!head) return r;
        *(struct robust_list_head**)a2 = head;
        *(size_t*)a3 = sizeof(*head);
        return 0;
    }
    return real(n, a1, a2, a3, a4, a5, a6);
}

#ifndef QUESTDECK_GAME
static const char NAME[] = "STEAM_COMPAT_DATA_PATH";

static int blocked(const char* s) {
    const size_t n = sizeof(NAME) - 1;
    return s && !strncmp(s, NAME, n) && (s[n] == '=' || s[n] == 0);
}

__attribute__((visibility("default"))) int setenv(const char* name, const char* value, int overwrite) {
    static int (*real)(const char*, const char*, int);
    if (blocked(name)) return 0;
    if (!real) real = (int (*)(const char*, const char*, int))dlsym(RTLD_NEXT, "setenv");
    return real(name, value, overwrite);
}

__attribute__((visibility("default"))) int putenv(char* string) {
    static int (*real)(char*);
    if (blocked(string)) return 0;
    if (!real) real = (int (*)(char*))dlsym(RTLD_NEXT, "putenv");
    return real(string);
}

__attribute__((visibility("default"))) int execve(const char* path, char* const argv[], char* const envp[]) {
    static int (*real)(const char*, char* const*, char* const*);
    if (!real) real = (int (*)(const char*, char* const*, char* const*))dlsym(RTLD_NEXT, "execve");
    size_t n = 0;
    while (envp && envp[n]) n++;
    char** clean = (char**)malloc((n + 1) * sizeof(char*));
    if (!clean) return real(path, argv, envp);
    size_t k = 0;
    for (size_t i = 0; i < n; i++)
        if (!blocked(envp[i])) clean[k++] = envp[i];
    clean[k] = NULL;
    int r = real(path, argv, clean);
    free(clean);
    return r;
}

static struct sigaction appHandler[65];
static int crashLogging;

static void crashed(int sig, siginfo_t* si, void* uc) {
    const char* p = getenv("QUESTDECK_EXIT_LOG");
    int fd = p ? open(p, O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0644) : -1;
    if (fd >= 0) {
        char head[96];
        int n = snprintf(head, sizeof(head), "== signal %d in pid %d (addr %p)\n", sig, getpid(), si ? si->si_addr : NULL);
        (void)!write(fd, head, (size_t)n);
        char c[400];
        int cf = open("/proc/self/cmdline", O_RDONLY | O_CLOEXEC);
        int r = cf >= 0 ? (int)read(cf, c, sizeof(c) - 1) : 0;
        if (cf >= 0) close(cf);
        for (int i = 0; i < r; i++)
            if (!c[i]) c[i] = ' ';
        if (r > 0) {
            c[r] = '\n';
            (void)!write(fd, c, (size_t)r + 1);
        }
        void* pc[48];
        backtrace_symbols_fd(pc, backtrace(pc, 48), fd);
        close(fd);
    }
    struct sigaction* a = &appHandler[sig];
    if ((a->sa_flags & SA_SIGINFO) && a->sa_sigaction) a->sa_sigaction(sig, si, uc);
    else if (a->sa_handler != SIG_DFL && a->sa_handler != SIG_IGN && a->sa_handler) a->sa_handler(sig);
    else { signal(sig, SIG_DFL); raise(sig); }
}

static int fatal(int sig) { return sig == SIGSEGV || sig == SIGBUS || sig == SIGILL || sig == SIGABRT || sig == SIGTRAP || sig == SIGFPE; }

__attribute__((visibility("default"))) int sigaction(int sig, const struct sigaction* act, struct sigaction* old) {
    static int (*real)(int, const struct sigaction*, struct sigaction*);
    if (!real) real = (int (*)(int, const struct sigaction*, struct sigaction*))dlsym(RTLD_NEXT, "sigaction");
    if (!crashLogging || !fatal(sig) || sig > 64) return real(sig, act, old);
    if (old) *old = appHandler[sig];
    if (!act) return 0;
    appHandler[sig] = *act;
    struct sigaction mine = *act;
    mine.sa_flags |= SA_SIGINFO;
    mine.sa_sigaction = crashed;
    return real(sig, &mine, NULL);
}

// QUESTDECK_EXIT_MATCH in it.
static void atExit(void) {
    const char* p = getenv("QUESTDECK_EXIT_LOG");
    int fd = p ? open(p, O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0644) : -1;
    if (fd < 0) return;
    char head[64];
    int n = snprintf(head, sizeof(head), "== exit of pid %d\n", getpid());
    (void)!write(fd, head, (size_t)n);
    void* pc[48];
    backtrace_symbols_fd(pc, backtrace(pc, 48), fd);
    close(fd);
}

__attribute__((constructor)) static void init(void) {
    unsetenv(NAME);
    const char* match = getenv("QUESTDECK_EXIT_MATCH");
    if (!match) return;
    char c[512] = "";
    int fd = open("/proc/self/cmdline", O_RDONLY | O_CLOEXEC);
    if (fd < 0) return;
    int r = (int)read(fd, c, sizeof(c) - 1);
    close(fd);
    for (int i = 0; i < r; i++)
        if (!c[i]) c[i] = ' ';
    if (r > 0 && strstr(c, match)) {
        atexit(atExit);
        crashLogging = 1;
        for (int sig = 1; sig < 32; sig++)
            if (fatal(sig)) {
                struct sigaction d;
                memset(&d, 0, sizeof(d));
                d.sa_handler = SIG_DFL;
                sigaction(sig, &d, NULL);
            }
    }
}
#endif
