#define _GNU_SOURCE
#include <GL/glx.h>
#include <dlfcn.h>
#include <pthread.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>

/* This wrapper forwards the real operation before bounded CPU bookkeeping. */
#define SLOT_COUNT 16
#define TEXT_LIMIT 128
struct slot {
    Display *display;
    GLXDrawable drawable;
    uint64_t total, interval_start, interval_base;
};
static struct slot slots[SLOT_COUNT];
static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_once_t once = PTHREAD_ONCE_INIT;
static int limit_reported, time_reported;
static void (*real_swap)(Display *, GLXDrawable);
static const GLubyte *(*get_string)(GLenum);
static GLXContext (*current_context)(void);
static Bool (*is_direct)(Display *, GLXContext);
static void (*query_drawable)(Display *, GLXDrawable, int, unsigned int *);

static void output(const char *text, size_t n) {
    /* At most one bounded JSON line. A failed/short write loses telemetry only. */
    ssize_t result;
    do { result = write(STDERR_FILENO, text, n); } while (result < 0 && errno == EINTR);
}
static void resolve(void) {
    real_swap = (void (*)(Display *, GLXDrawable))dlsym(RTLD_NEXT, "glXSwapBuffers");
    get_string = (const GLubyte *(*)(GLenum))dlsym(RTLD_NEXT, "glGetString");
    current_context = (GLXContext (*)(void))dlsym(RTLD_NEXT, "glXGetCurrentContext");
    is_direct = (Bool (*)(Display *, GLXContext))dlsym(RTLD_NEXT, "glXIsDirect");
    query_drawable = (void (*)(Display *, GLXDrawable, int, unsigned int *))dlsym(RTLD_NEXT, "glXQueryDrawable");
    if (!real_swap) {
        static const char error[] = "{\"event\":\"symbol_error\",\"symbol\":\"glXSwapBuffers\",\"telemetry_valid\":false}\n";
        output(error, sizeof(error)-1);
    }
}
static uint64_t monotonic_ns(void) {
    struct timespec t;
    if (clock_gettime(CLOCK_MONOTONIC, &t)) return 0;
    return (uint64_t)t.tv_sec*1000000000ULL+(uint64_t)t.tv_nsec;
}
static int escaped(char *out, const GLubyte *input) {
    static const char hex[] = "0123456789abcdef";
    size_t n = 0, i = 0;
    if (!input) { out[0]=0; return 0; }
    for (; i<TEXT_LIMIT && input[i]; i++) {
        unsigned char c=input[i];
        if (c=='"' || c=='\\') { out[n++]='\\'; out[n++]=(char)c; }
        else if (c<32 || c>=127) {
            out[n++]='\\';out[n++]='u';out[n++]='0';out[n++]='0';out[n++]=hex[c>>4];out[n++]=hex[c&15];
        } else out[n++]=(char)c;
    }
    out[n]=0;
    return input[i]==0;
}
static void dimensions(Display *d, GLXDrawable x, unsigned int *width, unsigned int *height) {
    *width=0; *height=0;
    if (query_drawable) {
        query_drawable(d,x,GLX_WIDTH,width);
        query_drawable(d,x,GLX_HEIGHT,height);
    }
}
static void renderer(const struct slot *s, uint64_t now) {
    char vendor[6*TEXT_LIMIT+1], device[6*TEXT_LIMIT+1], version[6*TEXT_LIMIT+1], line[3072];
    GLXContext context=current_context ? current_context() : NULL;
    const GLubyte *v=NULL, *r=NULL, *g=NULL;
    unsigned int width,height;
    if (context && get_string) { v=get_string(GL_VENDOR); r=get_string(GL_RENDERER); g=get_string(GL_VERSION); }
    int a=escaped(vendor,v),b=escaped(device,r),c=escaped(version,g);
    int direct_known=context && is_direct;
    int direct=direct_known ? is_direct(s->display,context) : 0;
    dimensions(s->display,s->drawable,&width,&height);
    int complete=a && b && c && direct_known && width && height;
    int n=snprintf(line,sizeof(line),
        "{\"event\":\"renderer\",\"display\":%" PRIuPTR ",\"drawable\":%lu,\"monotonic_ns\":%" PRIu64 ",\"total_swaps\":1,\"vendor\":\"%s\",\"renderer\":\"%s\",\"version\":\"%s\",\"width\":%u,\"height\":%u,\"direct\":%s,\"direct_known\":%s,\"metadata_complete\":%s}\n",
        (uintptr_t)s->display,s->drawable,now,vendor,device,version,width,height,
        direct?"true":"false",direct_known?"true":"false",complete?"true":"false");
    if(n>0 && (size_t)n<sizeof(line)) output(line,(size_t)n);
}
static void interval(const struct slot *s, uint64_t now) {
    char line[768];
    unsigned int width,height;
    uint64_t elapsed=now-s->interval_start, count=s->total-s->interval_base;
    /* Integer formatting keeps JSON valid under non-C numeric locales. */
    uint64_t milli_hz=(uint64_t)(((__uint128_t)count*1000000000000ULL)/elapsed);
    dimensions(s->display,s->drawable,&width,&height);
    int n=snprintf(line,sizeof(line),
        "{\"event\":\"interval\",\"display\":%" PRIuPTR ",\"drawable\":%lu,\"start_ns\":%" PRIu64 ",\"end_ns\":%" PRIu64 ",\"elapsed_ns\":%" PRIu64 ",\"interval_swaps\":%" PRIu64 ",\"total_swaps\":%" PRIu64 ",\"swap_hz\":%" PRIu64 ".%03" PRIu64 ",\"width\":%u,\"height\":%u}\n",
        (uintptr_t)s->display,s->drawable,s->interval_start,now,elapsed,count,s->total,
        milli_hz/1000,milli_hz%1000,width,height);
    if(n>0 && (size_t)n<sizeof(line)) output(line,(size_t)n);
}
void glXSwapBuffers(Display *display, GLXDrawable drawable) {
    pthread_once(&once,resolve);
    if (!real_swap) return; /* Cannot invent a real swap when resolution fails. */
    /* missing forwarded call */
    pthread_mutex_lock(&lock);
    uint64_t now=monotonic_ns();
    if (!now) {
        if(!time_reported) {
            static const char error[]="{\"event\":\"clock_error\",\"telemetry_valid\":false}\n";
            output(error,sizeof(error)-1);time_reported=1;
        }
        pthread_mutex_unlock(&lock);return;
    }
    struct slot *found=NULL,*empty=NULL;
    for (unsigned int i=0;i<SLOT_COUNT;i++) {
        if (slots[i].total && slots[i].display==display && slots[i].drawable==drawable) { found=&slots[i];break; }
        if (!slots[i].total && !empty) empty=&slots[i];
    }
    if (!found && empty) {
        *empty=(struct slot){display,drawable,1,now,1};renderer(empty,now);
    } else if (found) {
        found->total++;
        if (now>=found->interval_start && now-found->interval_start>=1000000000ULL) {
            interval(found,now);found->interval_start=now;found->interval_base=found->total;
        }
    } else if(!limit_reported) {
        static const char error[]="{\"event\":\"slot_limit\",\"capacity\":16,\"untracked_drawables_valid\":false}\n";
        output(error,sizeof(error)-1);limit_reported=1;
    }
    pthread_mutex_unlock(&lock);
}
