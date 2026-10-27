"""Private SOURCEHOLD candidate tests: no tracked edits or SDK operations."""
from pathlib import Path
import importlib.util
import json
import re
import subprocess

ROOT=Path('/home/bjoern/git/teaching-exciting-science-2026/.worktrees/beginner-vulkan-20261022')
P=ROOT/'ros2/src/so101_station_vulkan'
OUT=Path('/tmp/latest-native-final-resource-review-20261027')


def body(source, signature):
    begin=source.index('{',source.index(signature));depth=1
    for end in range(begin+1,len(source)):
        depth+=(source[end]=='{')-(source[end]=='}')
        if not depth:return source[begin+1:end]
    raise AssertionError(signature)


def capture_guard(camera, gate):
    # Sleep-until2ms is the explicit outer polling cadence, not shared-worker
    # backpressure. The actual intake helpers cannot sleep/wait/notify or copy.
    forbidden=r'\b(?:new|malloc|calloc|realloc|memcpy|memmove|serialize|vkDeviceWaitIdle|vkQueueWaitIdle)\b|bridge_|\.publish\s*\(|->publish\s*\(|copy_raw_planes|copy_vendor_planes|vendor_alignment_|\.snapshot\s*\(|try_snapshot\s*\(|(?:std::)?(?:lock_guard|unique_lock|scoped_lock)|\.lock\s*\(|\.wait(?:_for|_until)?\s*\(|\.(?:resize|reserve|push_back|emplace_back)\s*\('
    for signature in ('void capture_loop(', 'void capture_pending(', 'void record_failure(', 'void original_metadata(', 'void verify_original(', 'CameraSourceObservation inspect_original('):
        code=body(camera,signature)
        assert not re.search(forbidden,code), signature
        if signature!='void capture_loop(':assert not re.search(r'\b(?:sleep_until|sleep_for|yield)\s*\(',code)
    failure=body(camera,'void record_failure(')
    assert 'ownership_.revoke()' in failure and 'fault_claimed_.compare_exchange_strong' in failure
    assert 'publication_gate()' not in failure
    report=body(camera,'void report_status(')
    assert 'scheduler_.try_snapshot(status_counts_,8)' in report
    assert 'if(snapshot_fresh)status_counts_ns_=sampled_ns' in report
    assert 'counter_snapshot_fresh=' in report and 'counter_snapshot_age_ms=' in report
    revoke=body(gate,'bool revoke(')
    assert 'compare_exchange_strong(token,0,std::memory_order_acq_rel)' in revoke
    assert not re.search(r'lock|wait|new|malloc|publish|invalidate',revoke)


def test_actual_capture_static_resource_boundary_and_mutants():
    camera=(P/'src/camera_component.cpp').read_text();gate=(P/'include/so101_station_vulkan/raw_camera_admission.hpp').read_text()
    capture_guard(camera,gate)
    for addition in ('std::lock_guard<std::mutex> lock(mutex_);','bridge_->collect(frame);','memcpy(a,b,n);','frames_->publish(frame);','scheduler_.snapshot();','publication_gate().invalidate(this,1);'):
        c=camera.replace('void record_failure(std::exception_ptr error,const FrameMetadata& metadata)noexcept{','void record_failure(std::exception_ptr error,const FrameMetadata& metadata)noexcept{'+addition)
        try:capture_guard(c,gate)
        except AssertionError:pass
        else:raise AssertionError(addition)
    for addition in ('std::lock_guard<std::mutex> lock(mutex_);','publish();','wait();'):
        g=gate.replace('bool revoke(uint64_t token)noexcept {','bool revoke(uint64_t token)noexcept {'+addition)
        try:capture_guard(camera,g)
        except AssertionError:pass
        else:raise AssertionError(addition)


def test_actual_bounded_observer_stale_sample_under_writer(tmp_path):
    spec=importlib.util.spec_from_file_location('tracked',ROOT/'tests/test_latest_native_scheduler_perf_review.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    cpp=module.CPP.replace('thread_local bool inside=false;','thread_local bool inside=false;double bounded_snapshot_ns=0;')
    anchor='auto before=allocations.load();auto t=Clock::now();for(unsigned i=1;i<=N;i++){Handle n(i);'
    replacement='''typename decltype(q)::Snapshot prior;prior.received=777;prior.unique=778;
 auto mark=allocations.load();auto timer=Clock::now();for(unsigned i=0;i<N;i++)require(!q.try_snapshot(prior,8));bounded_snapshot_ns=ns(timer)/N;
 require(prior.received==777&&prior.unique==778&&allocations.load()==mark);
 auto before=allocations.load();auto t=Clock::now();for(unsigned i=1;i<=N;i++){Handle n(i);'''
    assert anchor in cpp
    cpp=cpp.replace(anchor,replacement)
    anchor='<<c.contended<<",\\\"hot_heap_allocations'
    replacement='<<c.contended<<",\\\"bounded_snapshot_stale_ns\\\":"<<bounded_snapshot_ns<<",\\\"hot_heap_allocations'
    assert anchor in cpp
    cpp=cpp.replace(anchor,replacement)
    source=tmp_path/'bounded.cpp';source.write_text(cpp);binary=tmp_path/'bounded'
    subprocess.run(['c++','-std=c++17','-O2','-pthread','-Wall','-Wextra','-Werror','-I',str(P/'include'),str(source),'-o',str(binary)],check=True,capture_output=True,text=True)
    result=subprocess.run([str(binary)],check=True,capture_output=True,text=True,timeout=30)
    report=json.loads(result.stdout);assert report['bounded_snapshot_stale_ns']>0 and report['hot_heap_allocations']==0
    (OUT/'bounded-observer.cpp').write_text(cpp);(OUT/'bounded-observer.json').write_text(json.dumps(report,indent=2)+'\n')
