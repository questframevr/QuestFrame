#include <stdio.h>
#include <stdlib.h>
#include <dlfcn.h>
typedef unsigned (*init_t)(int *, int, const char *);
typedef const char *(*desc_t)(int);
typedef int (*b_t)(void);
typedef void (*v_t)(void);
int main(int argc, char **argv) {
  void *h = dlopen("libopenvr_api.so", RTLD_NOW);
  if (!h) { printf("dlopen: %s\n", dlerror()); return 1; }
  init_t init = (init_t)dlsym(h, "VR_InitInternal2");
  desc_t desc = (desc_t)dlsym(h, "VR_GetVRInitErrorAsEnglishDescription");
  desc_t sym = (desc_t)dlsym(h, "VR_GetVRInitErrorAsSymbol");
  b_t present = (b_t)dlsym(h, "VR_IsHmdPresent"), rt = (b_t)dlsym(h, "VR_IsRuntimeInstalled");
  v_t shut = (v_t)dlsym(h, "VR_ShutdownInternal");
  printf("runtime installed %d, hmd present %d\n", rt ? rt() : -1, present ? present() : -1);
  int type = argc > 1 ? atoi(argv[1]) : 3, err = 0;
  unsigned tok = init(&err, type, "");
  printf("VR_Init(type %d): token %u, error %d %s: %s\n", type, tok, err, sym(err), desc(err));
  fflush(stdout);
  if (!err && argc > 2) { extern unsigned sleep(unsigned); sleep(atoi(argv[2])); }
  if (!err) shut();
  return err;
}
