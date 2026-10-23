"""Independent metadata-only benchmark guards; never imports camera libraries."""
import ast
import importlib.util
import json
from pathlib import Path
import statistics
import time

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'tools/benchmark_realsense_profiles.py'
SPEC = importlib.util.spec_from_file_location('profile_benchmark_perf_review', SOURCE)
b = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(b)


def test_metadata_only_bounded_queue_and_storage():
    source = SOURCE.read_text()
    tree = ast.parse(source)
    attributes = {node.func.attr for node in ast.walk(tree)
                  if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)}
    assert not attributes & {
        'get_data', 'tobytes', 'asarray', 'array', 'resize', 'copy',
        'set_option', 'set_region_of_interest', 'hardware_reset',
        'enable_record_to_file', 'save_to_ply', 'imshow', 'imwrite', 'VideoWriter',
    }
    queue_calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
                   and isinstance(node.func, ast.Attribute) and node.func.attr == 'frame_queue']
    assert len(queue_calls) == 1 and len(queue_calls[0].args) == 1
    assert isinstance(queue_calls[0].args[0], ast.Constant) and queue_calls[0].args[0].value == 1
    starts = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
              and isinstance(node.func, ast.Attribute) and node.func.attr == 'start']
    assert len(starts) == 1 and isinstance(starts[0].args[1], ast.Name) and starts[0].args[1].id == 'queue'
    assert not any(isinstance(node, ast.Lambda) for node in ast.walk(tree))
    assert not any(isinstance(node, ast.While) and any(
        isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute)
        and call.func.attr == 'poll_for_frame' for call in ast.walk(node.test))
        for node in ast.walk(tree)), 'queue drain must have an explicit iteration bound'
    assert '[None] * capacity' in source and 'if count >= capacity:' in source
    assert b.STEADY_S == 8 and b.WARMUP_S == 2 and b.TRIAL_TIMEOUT_S == 35
    assert b.WAIT_MS == 3000 and len(b.make_plan()) == 18
    assert 'timeout=TRIAL_TIMEOUT_S' in source
    assert 'if started:' in source and 'pipe.stop()' in source


def test_metadata_reads_preserve_stream_clock_domain():
    class Frame:
        def get_frame_number(self): return 17
        def get_timestamp(self): return 123.25
        def get_frame_timestamp_domain(self): return 'hardware_clock'
    assert b.frame_metadata(Frame()) == dict(id=17, timestamp_ms=123.25, domain='hardware_clock')
    assert len(b.PROFILES) == 3
    assert b.PROFILES['maximum'][0] == b.profile('color', 1920, 1080, 'rgb8')
    assert b.PROFILES['maximum'][1] == b.profile('depth', 1280, 720, 'z16')
    # Enumerating and hashing binaries occurs outside timed metadata extraction.
    frame_node = next(n for n in ast.parse(SOURCE.read_text()).body
                      if isinstance(n, ast.FunctionDef) and n.name == 'frame_metadata')
    assert not any(isinstance(n, (ast.For, ast.While)) for n in ast.walk(frame_node))
    names = {n.func.attr for n in ast.walk(frame_node)
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)}
    assert names == {'get_frame_number', 'get_timestamp', 'get_frame_timestamp_domain'}


def instrumentation_microbenchmark():
    """Actual metadata/statistics helpers on fake frames; excludes SDK call cost."""
    class Frame:
        def __init__(self, number): self.number = number
        def get_frame_number(self): return self.number
        def get_timestamp(self): return self.number * (1000 / 30)
        def get_frame_timestamp_domain(self): return 'hardware_clock'
    depth, color = Frame(100), Frame(103)
    count = 20000
    timing = []
    checksum = 0
    for _ in range(7):
        start = time.perf_counter_ns()
        for index in range(count):
            row = dict(depth=b.frame_metadata(depth), color=b.frame_metadata(color))
            checksum += row['depth']['id'] + row['color']['id']
        timing.append((time.perf_counter_ns() - start) / count)
    # Gaps and duplicates deliberately present in the bounded eight-second cohort.
    rows = [dict(depth=b.frame_metadata(Frame(i - (i % 37 == 0))),
                 color=b.frame_metadata(Frame(i + 3))) for i in range(240)]
    stats = []
    for _ in range(7):
        start = time.perf_counter_ns()
        for _ in range(100):
            for key in ('depth', 'color'):
                result = b.stream_summary([row[key] for row in rows])
                checksum += result['unique_ids']
            checksum += b.cohort_summary(rows)['unique_cohorts']
        stats.append((time.perf_counter_ns() - start) / 100)
    return dict(cpu_only=True, fake_frames=True, sdk_call_cost_included=False,
                samples_per_repeat=count, repeats=7, statistic='median',
                pair_metadata_ns=statistics.median(timing),
                statistics_240_cohorts_ns=statistics.median(stats), checksum=checksum)


def test_actual_metadata_helpers_fake_frame_microbenchmark():
    report = instrumentation_microbenchmark()
    assert report['checksum'] > 0
    assert report['pair_metadata_ns'] > 0 and report['statistics_240_cohorts_ns'] > 0
    print(json.dumps(report, sort_keys=True))


def test_last_alignment_completion_uses_actual_elapsed_denominator():
    from unittest.mock import MagicMock, patch
    rs, pipe, cfg, dev, queue = [MagicMock() for _ in range(5)]
    pair = b.PROFILES['baseline']
    rs.pipeline.return_value = pipe
    rs.frame_queue.return_value = queue
    resolved = cfg.resolve.return_value
    resolved.get_streams.return_value = pair
    resolved.get_device.return_value.get_info.return_value = 'fixture-private'
    pipe.start.return_value.get_streams.return_value = pair
    def frames(number):
        result = MagicMock()
        for which, prof in [('color', pair[0]), ('depth', pair[1])]:
            frame = getattr(result, 'get_' + which + '_frame').return_value
            frame.get_profile.return_value = prof
            frame.get_frame_number.return_value = number
            frame.get_timestamp.return_value = number * (1000 / 30)
            frame.get_frame_timestamp_domain.return_value = 'hardware_clock'
            frame.get_units.return_value = .001
        return result
    first, last = frames(1), frames(2)
    queue.wait_for_frame.side_effect = [MagicMock(as_frameset=lambda:first), MagicMock(as_frameset=lambda:last)]
    rs.align.return_value.process.side_effect = lambda incoming:incoming
    # Last arrival is inside8s; its alignment completes after8s. Receipt-only
    # throughput cannot count completion against an invented exact8s denominator.
    wall = [0, 0, 0, 0, .1, .1, 1, 1, 1, 7.9, 7.9, 8.2, 8.2, 8.2, 8.25]
    cpu = [100, 100.1, 100.2, 100.3, 101, 101.1]
    with patch.object(b, 'refuse_busy', return_value={}), patch.object(b, 'device', return_value=(dev, 'fixture-private')), patch.object(b, 'configure', return_value=cfg), patch.object(b, 'sdk_profile', side_effect=lambda profile:profile), patch.object(b, 'options', return_value=[]), patch.object(b, 'provenance', return_value={}), patch.object(b, 'WARMUP_S', 0), patch.object(b.time, 'monotonic', side_effect=wall), patch.object(b.time, 'process_time', side_effect=cpu):
        result = b.trial_run(rs, dict(profile='baseline', mode='sdk_align', repeat=0))
    metrics = result['metrics']
    assert metrics['cohorts'] == 2 and metrics['completed_after_window'] == 1
    assert metrics['steady_wall_s'] == 8.25
    assert abs(metrics['logical_cpu_demand'] - 1.1 / 8.25) < 1e-12
    assert metrics['completed_cohorts_per_steady_elapsed_s'] == 2 / 8.25
    assert result['samples'][-1]['host_received_s'] == 7.9
    assert result['samples'][-1]['host_completed_s'] == 8.2
    assert all(r['host_completed_s'] >= r['host_received_s'] for r in result['samples'])
    pipe.stop.assert_called_once()
