#define _GNU_SOURCE
#include <GL/glx.h>
#include <stdint.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <time.h>
#include <stdlib.h>
#include <dlfcn.h>
static uint64_t now_ns=1000000000ULL, swaps, queries;
static int resized, real_clock;
static Display *last_display;static GLXDrawable last_drawable;
int fake_arguments(Display*d,GLXDrawable x){return last_display==d && last_drawable==x;}
static int (*next_clock)(clockid_t,struct timespec*);
void fake_real_clock(void){real_clock=1;next_clock=(int(*)(clockid_t,struct timespec*))dlsym(RTLD_NEXT,"clock_gettime");}
void fake_time(uint64_t n){now_ns=n;}
void fake_resize(void){resized=1;}
uint64_t fake_swaps(void){return swaps;}
uint64_t fake_queries(void){return queries;}
void glXSwapBuffers(Display *d,GLXDrawable x){last_display=d;last_drawable=x;swaps++;}
const GLubyte *glGetString(GLenum key){
 if(getenv("FAKE_MISSING_METADATA"))return NULL;
 if(key==GL_VENDOR)return (const GLubyte*)"Fake \"vendor\"";
 if(key==GL_RENDERER)return (const GLubyte*)"Fake\\renderer\nGPU";
 return (const GLubyte*)"4.6 fake";
}
GLXContext glXGetCurrentContext(void){return (GLXContext)(uintptr_t)3;}
Bool glXIsDirect(Display*d,GLXContext c){(void)d;(void)c;return True;}
void glXQueryDrawable(Display*d,GLXDrawable x,int attribute,unsigned int *v){
 (void)d;queries++;*v=(attribute==GLX_WIDTH?1000:500)+(unsigned)x+(resized?200:0);
}
int clock_gettime(clockid_t which,struct timespec *t){
 if(real_clock && next_clock)return next_clock(which,t);
 if(which!=CLOCK_MONOTONIC)return (int)syscall(SYS_clock_gettime,which,t);
 t->tv_sec=(time_t)(now_ns/1000000000ULL);t->tv_nsec=(long)(now_ns%1000000000ULL);return 0;
}
