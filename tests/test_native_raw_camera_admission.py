"""Cold admission boundaries: source guards complement CPU software SDK tests.

These checks never construct a camera or Vulkan Context and make no physical
qualification claim. Procedural C++ tests exercise identity/probe/fallback logic.
"""
from pathlib import Path
import re
import unittest

P = Path(__file__).resolve().parents[1] / 'ros2/src/so101_station_vulkan'
CAMERA = (P / 'src/camera_component.cpp').read_text()
ADMISSION = (P / 'src/raw_camera_admission.cpp').read_text()


def body(name):
    match = re.search(r'\b' + re.escape(name) + r'\([^;]*?\)\s*(?:(?:const|noexcept)\s*)?\{', CAMERA)
    assert match, name
    start = match.end()
    depth = 1
    for end in range(start, len(CAMERA)):
        depth += (CAMERA[end] == '{') - (CAMERA[end] == '}')
        if not depth:
            return CAMERA[start:end]
    raise AssertionError(name)


class RawCameraAdmission(unittest.TestCase):
    def test_every_owned_camera_start_stabilizes_before_source_admission(self):
        # Bypassing startup on CPU or fallback would admit the observed native
        # counter resets as live outputs. The CPU gate test exercises the trace;
        # this source guard pins its placement at both actual capture starts.
        ctor = body('CameraComponent')
        self.assertIn('wait_startup_stable();', ctor)
        first = ctor.index('wait_startup_stable();')
        self.assertLess(ctor.index('resolved and active camera calibration differ'), first)
        self.assertLess(first, ctor.index('cold_live_selfcheck(std::move(cold))'))
        restart = ctor[ctor.index('if(!raw&&!running_)'):ctor.index('if(!raw)bridge_=make_bridge(false)')]
        self.assertIn('wait_startup_stable();', restart)
        self.assertLess(restart.index('source_.alignment.generation='), restart.index('wait_startup_stable();'))
        seed = ctor.index('bridge_->accept_source(previous_color_,previous_depth_)')
        self.assertLess(seed, ctor.index('publication_gate().admit'))
        warm = body('wait_startup_stable')
        self.assertIn('source_admitted=false', warm)
        self.assertIn('execution_eligible=false', warm)
        self.assertIn('gate.check_deadline(', warm)
        self.assertIn('inspect_original(native)', warm)
        self.assertIn('gate.observe(', warm)
        # A startup failure must release this owned capture before the caller's
        # prepared cold bridge unwinds and drains its GPU resources.
        self.assertIn('catch(...){publication_gate().invalidate(this,ownership_.token());stop_owned_camera();throw;}', warm)
        self.assertNotIn('frames_->publish', warm)
        self.assertNotIn('bridge_->', warm)
        self.assertNotIn('camera_generation(', warm)
        cold = body('cold_live_selfcheck')
        self.assertNotIn('auto warmup=', cold)
        self.assertIn('cold->accept_source(previous_color_,previous_depth_)', cold)
        self.assertLess(cold.index('cold->accept_source(previous_color_,previous_depth_)'), cold.index('pass<3'))
        self.assertLess(cold.index('gate.verify('), cold.index('remember_source(metadata)'))

    def test_raw_callback_contains_only_native_copies(self):
        raw = body('copy_raw_planes')
        self.assertEqual(raw.count('copy_rows('), 2)
        self.assertIn('native.get_color_frame()', raw)
        self.assertIn('native.get_depth_frame()', raw)
        self.assertIn('frame.depth.metadata()=metadata', raw)
        self.assertIn('slot.metadata=metadata', raw)
        for forbidden in ('process(', 'diagnostic_', 'alignment_reference(', 'Context::',
                          'make_unique', 'new ', 'resize(', 'reserve(', 'camera_generation('):
            self.assertNotIn(forbidden, raw)
        capture = body('worker_loop')
        self.assertIn('if(bridge_->gpu_alignment())copy_raw_planes', capture)
        self.assertIn('else copy_vendor_planes', capture)
        self.assertLess(capture.index('scheduler_.take'), capture.index('bridge_->acquire'))
        self.assertLess(capture.index('bridge_->acquire'), capture.index('bridge_->offer'))
        self.assertLess(body('capture_pending').index('verify_original'), body('capture_pending').index('scheduler_.accept'))
        for name in ('tick', 'capture_pending', 'copy_raw_planes', 'verify_original'):
            for forbidden in ('camera_generation(', 'probe_loaded_sdk_alignment(',
                              'diagnostic_', 'wait_for_frames(', 'make_unique', 'resize('):
                self.assertNotIn(forbidden, body(name))

    def test_periodic_status_and_plane_copies_are_bounded(self):
        copy = body('copy_rows')
        self.assertEqual(copy.count('source.get_data()'), 1)
        self.assertEqual(copy.count('source.get_stride_in_bytes()'), 1)
        self.assertIn('if(stride==int(row_bytes))', copy)
        self.assertIn('size_t(row_bytes)*unsigned(height)', copy)
        self.assertIn('capacity%row_bytes||size_t(height)!=capacity/row_bytes', copy)
        status = body('report_status')
        self.assertIn('char text[4096]', status)
        self.assertIn('std::snprintf(text,sizeof(text)', status)
        self.assertIn('status_message_.data.assign', status)
        self.assertNotIn('std::to_string', status)
        self.assertNotIn('publish_status(', status)
        self.assertIn('status_message_.data.reserve(4096)', body('CameraComponent'))

    def test_cold_gate_requires_original_cohort_all_three_comparisons(self):
        cold = body('cold_live_selfcheck')
        self.assertIn('pass<3', cold)
        self.assertIn('vendor_alignment_.process(original)', cold)
        self.assertLess(cold.index('raw_reference=frame.depth.upload()'), cold.index('cold->dispatch_pending()'))
        self.assertIn('alignment_reference(source_.alignment,raw_reference,', cold)
        self.assertIn('cpu.metadata=metadata;cpu.calibration=core_calibration()', cold)
        self.assertIn('diagnostic_depth()', cold)
        self.assertIn('diagnostic_xyz()', cold)
        self.assertIn('gate.verify(metadata,raw_reference,bytes+color_pixels()*3,completed.depth.diagnostic_depth()', cold)
        self.assertIn('aligned[i]!=vendor||reference[i]!=vendor', ADMISSION)
        self.assertIn('>1e-7f', ADMISSION)
        self.assertIn('std::isfinite(actual_xyz[4*i+a])', ADMISSION)
        self.assertIn('gpu.cpu_components[i]==cpu.cpu_components[i]', ADMISSION)
        self.assertIn('gate.require_complete()', cold)
        self.assertIn('if(failed_||passes_!=3)', ADMISSION)
        self.assertIn('native_nonzero<24||aligned_nonzero<24', ADMISSION)
        self.assertIn('catch(...){publication_gate().invalidate(this,ownership_.token());stop_owned_camera();throw;}', cold)
        self.assertIn('std::chrono::steady_clock::now()+10s', cold)
        ctor = body('CameraComponent')
        for before, after in [('probe_loaded_sdk_alignment()', 'camera_.start(config)'),
                              ('raw_admission_failure(probe,source_)', 'camera_.start(config)'),
                              ('cold=make_bridge(true,true)', 'camera_.start(config)'),
                              ('cold_live_selfcheck(std::move(cold))', 'frames_=create_publisher'),
                              ('cold_live_selfcheck(std::move(cold))', 'create_wall_timer')]:
            self.assertLess(ctor.index(before), ctor.index(after))
        self.assertIn('catch(...){shutdown_threads();publication_gate().invalidate(this,ownership_.token());throw;}', ctor)
        self.assertIn('if(!raw&&!running_)', ctor)
        self.assertIn('CPU fallback restart source calibration differs from resolved profiles', ctor)
        self.assertIn('ownership_(publication_gate(),this)', CAMERA)
        self.assertIn('require_raw_or_cpu_fallback(mode,fallback_reason_)', ctor)

    def test_unknown_is_latched_with_original_receipt(self):
        tick = body('tick')
        capture_loop = body('capture_loop')
        self.assertLess(capture_loop.index('received_ros_ns='), capture_loop.index('capture_pending'))
        self.assertNotIn('poll_for_frames', tick)
        self.assertNotIn('bridge_->', tick)
        capture = body('capture_pending')
        self.assertIn('source_admission_.observe', capture)
        failed = body('latch_failure')
        self.assertIn('shutdown_threads()', failed)
        self.assertIn('publication_gate().finalize_unknown(this,ownership_.token(),[&]{failures_->publish(static_cast<const std_msgs::msg::String&>(failure_diagnostic_));})', failed)
        self.assertLess(body('shutdown_threads').index('ownership_.revoke()'), body('shutdown_threads').index('stop_owned_camera()'))
        self.assertIn('publication_gate().publish(slot.metadata.calibration_generation', CAMERA)
        self.assertIn('m.timestamp_ns', failed)
        self.assertIn('m.frame_number', failed)
        self.assertIn('m.depth_frame_number', failed)
        self.assertIn('execution_eligible', failed)
        self.assertNotIn('now()', failed)
        self.assertNotIn('make_bridge', failed)

    def test_success_and_unknown_share_one_current_owner_dds_writer(self):
        self.assertEqual(CAMERA.count('"/station/resident/measurements"'), 1)
        ctor = body('CameraComponent')
        self.assertIn('failures_=create_publisher<std_msgs::msg::String>("/station/resident/measurements"', ctor)
        self.assertIn('publication_gate().invalidate(this,ownership_.token(),[&]{measurement_writer()=failures_;})', ctor)
        self.assertLess(ctor.index('measurement_writer()=failures_'), ctor.index('publication_gate().admit'))
        objects = CAMERA[CAMERA.index('class ObjectsComponent'):]
        self.assertNotIn('create_publisher', objects)
        self.assertNotIn('Publisher<std_msgs::msg::String>::SharedPtr', objects)
        self.assertIn('publication_gate().publish(slot.metadata.calibration_generation,slot.metadata.publication_session_token,[&]{', objects)
        self.assertIn('auto measurements_=measurement_writer().lock();', objects)
        self.assertIn('if(measurements_)measurements_->publish', objects)
        self.assertIn('publication_gate().finalize_unknown(this,ownership_.token(),[&]{failures_->publish(static_cast<const std_msgs::msg::String&>(failure_diagnostic_));})', body('latch_failure'))

    def test_software_probe_cannot_be_manually_selected(self):
        self.assertIn('for(unsigned fixture:{2u,3u,4u})', ADMISSION)
        self.assertIn('reference[i]==actual[i]', ADMISSION)
        self.assertIn('reference[i]!=actual[i]', ADMISSION)
        self.assertIn('AlignmentConvention::Sse', ADMISSION)
        self.assertIn('only identified scalar convention is qualified', ADMISSION)
        self.assertNotIn('Context::', ADMISSION)
        self.assertNotIn('rs2::pipeline', ADMISSION)
        self.assertEqual(CAMERA.count('declare_parameter<'), 3)
        self.assertNotIn('set_option(', CAMERA)


if __name__ == '__main__':
    unittest.main()
