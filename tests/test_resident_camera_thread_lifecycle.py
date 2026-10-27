"""Actual camera lifecycle bodies with CPU threads; no ROS, SDK or GPU."""
from pathlib import Path
import shutil
import subprocess

P = Path(__file__).resolve().parents[1] / 'ros2/src/so101_station_vulkan'


def body(source, signature):
    begin = source.index('{', source.index(signature))
    depth = 1
    for end in range(begin + 1, len(source)):
        depth += (source[end] == '{') - (source[end] == '}')
        if not depth:
            return source[begin + 1:end]
    raise AssertionError('unbalanced method')


def test_actual_shutdown_and_partial_thread_start_failure(tmp_path):
    source = (P / 'src/camera_component.cpp').read_text()
    shutdown = body(source, 'void shutdown_threads()')
    start = body(source, 'void start_threads()').replace('std::thread(', 'TestThread(')
    code = r'''
#include "so101_station_vulkan/latest_native_scheduler.hpp"
#include <thread>
#include <cassert>
#include <atomic>
#include <string>
#include <stdexcept>
using namespace so101_station_vulkan;
namespace rs2 {struct frameset {int id=0;explicit operator bool()const{return id!=0;}};}
struct Metadata {std::string serial;};
struct TestThread {
 static inline int created=0,fail_at=0;
 std::thread thread;
 TestThread()=default;
 template<class F>explicit TestThread(F f){if(++created==fail_at)throw std::runtime_error("injected thread construction failure");thread=std::thread(f);}
 TestThread(TestThread&&)=default;TestThread& operator=(TestThread&&)=default;
 bool joinable(){return thread.joinable();}void join(){thread.join();}
};
struct Owner {std::atomic<bool> revoked{false};void revoke(){revoked=true;}};
struct Harness {
 Owner ownership_;std::atomic<bool> stopping_{false},capture_live{false},worker_live{false};
 std::atomic<unsigned> capture_calls{0},worker_calls{0};unsigned stop_calls=0;
 TestThread capture_thread_,worker_thread_;LatestNativeScheduler<rs2::frameset,Metadata> scheduler_;
 bool running_=true;
 void capture_loop(){capture_live=true;while(!stopping_.load()){++capture_calls;std::this_thread::yield();}capture_live=false;}
 void worker_loop(){worker_live=true;while(!stopping_.load()){++worker_calls;std::this_thread::yield();}worker_live=false;}
 void stop_owned_camera(){assert(!capture_live.load()&&!capture_thread_.joinable());if(running_){++stop_calls;running_=false;}}
 void shutdown_threads()noexcept {SHUTDOWN}
 void start_threads(){START}
 ~Harness(){shutdown_threads();assert(!worker_live.load()&&!worker_thread_.joinable());}
};
int main(){
 for(int failure=0;failure<=2;++failure){
  TestThread::created=0;TestThread::fail_at=failure;Harness h;bool threw=false;
  try{h.start_threads();}catch(const std::runtime_error&){threw=true;}
  assert(threw==(failure!=0));
  if(!failure){while(!h.capture_calls.load()||!h.worker_calls.load())std::this_thread::yield();}
  h.shutdown_threads();assert(h.ownership_.revoked&&h.scheduler_.stopped());
  assert(!h.capture_live&&!h.worker_live&&!h.capture_thread_.joinable()&&!h.worker_thread_.joinable());
  assert(h.stop_calls==1);h.shutdown_threads();assert(h.stop_calls==1);
 }
}
'''.replace('SHUTDOWN', shutdown).replace('START', start)
    path = tmp_path / 'lifecycle.cpp'
    path.write_text(code)
    binary = tmp_path / 'lifecycle'
    built = subprocess.run([shutil.which('c++'), '-std=c++17', '-O2', '-pthread', '-Wall', '-Wextra', '-Werror', '-I', str(P / 'include'), str(path), '-o', str(binary)], capture_output=True, text=True)
    assert built.returncode == 0, built.stderr
    subprocess.run([str(binary)], check=True, timeout=10)


def test_capture_is_independent_of_compute_publication_and_diagnostic_contention():
    source = (P / 'src/camera_component.cpp').read_text()
    capture = body(source, 'void capture_loop()') + body(source, 'void capture_pending(')
    failure = body(source, 'void record_failure(')
    for forbidden in ('bridge_->', 'copy_raw_planes', 'copy_vendor_planes', 'vendor_alignment_.process', 'publish(', 'snapshot(', 'try_snapshot(', 'publication_gate()', '.join(', '.stop(', 'std::mutex', 'lock_guard', 'unique_lock'):
        assert forbidden not in capture + failure, forbidden
    assert 'ownership_.revoke()' in failure
    assert 'fault_ready_.store(true,std::memory_order_release)' in failure
    assert 'received_ros_ns=uint64_t(now().nanoseconds())' in capture
    assert 'source_admission_.observe(last_input_)' in capture
    assert 'scheduler_.capture_service' in capture
    worker = body(source, 'void worker_loop()')
    assert worker.index('if(processing)') < worker.index('scheduler_.take(')
    assert worker.index('frames_->publish') < worker.index('scheduler_.complete(')
    assert 'metadata.publication_session_token' in worker
    assert 'scheduler_.try_snapshot(status_counts_,8)' in body(source, 'void report_status()')
    assert 'counter_snapshot_fresh=%s; counter_snapshot_age_ms=%.3f' in source
    assert 'frames_=create_publisher<FrameLease>("/station/resident/frame",rclcpp::QoS(1))' in source
    assert 'rclcpp::QoS(1)' in source[source.index('create_subscription<FrameLease>'):]
