#include <stddef.h>
#include <stdint.h>
extern void *__libc_malloc(size_t),*__libc_calloc(size_t,size_t),*__libc_realloc(void*,size_t);
extern void __libc_free(void*);
static uint64_t allocations;
static int enabled;
void audit_enable(int value){enabled=value;}
uint64_t audit_count(void){return allocations;}
void *malloc(size_t n){if(enabled)allocations++;return __libc_malloc(n);}
void *calloc(size_t a,size_t b){if(enabled)allocations++;return __libc_calloc(a,b);}
void *realloc(void*p,size_t n){if(enabled)allocations++;return __libc_realloc(p,n);}
void free(void*p){__libc_free(p);}
