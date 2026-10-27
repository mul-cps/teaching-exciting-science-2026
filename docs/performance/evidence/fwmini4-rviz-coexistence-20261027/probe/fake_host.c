#define _GNU_SOURCE
#include <GL/glx.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <assert.h>
void fake_real_clock(void);void fake_time(uint64_t);void fake_resize(void);uint64_t fake_swaps(void),fake_queries(void);
int fake_arguments(Display*,GLXDrawable);
static void swap_checked(Display*d,GLXDrawable x){glXSwapBuffers(d,x);assert(fake_arguments(d,x));}
void audit_enable(int);uint64_t audit_count(void);
static Display *display=(Display*)(uintptr_t)1;
static uint64_t stamp(void){struct timespec t;assert(!clock_gettime(CLOCK_MONOTONIC_RAW,&t));return (uint64_t)t.tv_sec*1000000000ULL+t.tv_nsec;}
int main(int argc,char**argv){
 const char *mode=argc>1?argv[1]:"separate";
 if(!strcmp(mode,"bench")){
  fake_real_clock();const unsigned n=500000;for(unsigned i=0;i<10000;i++)glXSwapBuffers(display,11);
  uint64_t before=fake_swaps(),queries=fake_queries(),a=audit_count();audit_enable(1);
  uint64_t t=stamp();for(unsigned i=0;i<n;i++)glXSwapBuffers(display,11);uint64_t dt=stamp()-t;
  audit_enable(0);assert(fake_swaps()-before==n);assert(audit_count()==a);assert(fake_queries()==queries);
  printf("{\"calls\":%u,\"elapsed_ns\":%llu,\"ns_per_call\":%.6f,\"hot_heap_allocations\":%llu,\"hot_metadata_queries\":%llu}\n",n,(unsigned long long)dt,(double)dt/n,(unsigned long long)(audit_count()-a),(unsigned long long)(fake_queries()-queries));
 }else if(!strcmp(mode,"bounds")){
  for(unsigned i=1;i<=20;i++)swap_checked(display,i);
  fake_time(2000000000ULL);glXSwapBuffers(display,1);assert(fake_swaps()==21);
 }else if(!strcmp(mode,"displays")){
  swap_checked(display,11);swap_checked((Display*)(uintptr_t)2,11);assert(fake_swaps()==2);
 }else{
  swap_checked(display,11);audit_enable(1);fake_time(1250000000ULL);swap_checked(display,22);
  fake_time(1500000000ULL);swap_checked(display,11);fake_resize();
  fake_time(2000000000ULL);swap_checked(display,11);fake_time(2250000000ULL);swap_checked(display,22);audit_enable(0);assert(audit_count()==0);assert(fake_swaps()==5);
 }
 return 0;
}
