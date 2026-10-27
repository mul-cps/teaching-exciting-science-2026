#define _GNU_SOURCE
#include <dlfcn.h>
#include <assert.h>
#include <stdint.h>
typedef void(*swap_t)(void*,unsigned long);
int main(void){swap_t s=(swap_t)dlsym(RTLD_DEFAULT,"glXSwapBuffers");assert(s);s((void*)(uintptr_t)1,11);s((void*)(uintptr_t)1,11);return 0;}
