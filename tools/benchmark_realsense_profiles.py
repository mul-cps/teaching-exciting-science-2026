#!/usr/bin/env python3
"""Bounded, metadata-only D435i RGB-D throughput experiment; capture is opt-in.

No pixels leave SDK storage. Counter gaps describe observed cohorts, not total
hardware loss. Device timestamps are never compared to a host clock origin.
"""
import argparse
import hashlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
import platform
import random
import statistics
import subprocess
import sys
import time


def profile(stream, width, height, fmt):
    return dict(stream=stream, width=width, height=height, format=fmt, fps=30)


PROFILES = {
    'baseline': [profile('color', 640, 480, 'rgb8'), profile('depth', 640, 480, 'z16')],
    'same_grid': [profile('color', 1280, 720, 'rgb8'), profile('depth', 1280, 720, 'z16')],
    'maximum': [profile('color', 1920, 1080, 'rgb8'), profile('depth', 1280, 720, 'z16')],
}
SEED = 20261023
WARMUP_S = 2.
STEADY_S = 8.
WAIT_MS = 3000
TRIAL_TIMEOUT_S = 35


def canonical(p):
    return tuple(p[k] for k in ('stream', 'width', 'height', 'format', 'fps'))


def verify_profiles(expected, actual):
    if sorted(map(canonical, expected)) != sorted(map(canonical, actual)):
        raise ValueError('resolved profiles differ from exact requested profiles')


def select_profiles(offered):
    available = set(map(canonical, offered))
    for name, pair in PROFILES.items():
        if any(canonical(p) not in available for p in pair):
            raise ValueError('exact requested profile unavailable: ' + name)
    return PROFILES


def make_plan():
    trials = [dict(profile=p, mode=m, repeat=r) for p in PROFILES for m in ('capture', 'sdk_align') for r in range(3)]
    random.Random(SEED).shuffle(trials)
    return trials


def quantile(values, fraction):
    values = sorted(values)
    if not values:
        return None
    pos = (len(values) - 1) * fraction
    lo = int(pos)
    return values[lo] + (values[math.ceil(pos)] - values[lo]) * (pos - lo)


def distribution(values):
    return dict(count=len(values), median=quantile(values, .5), p95=quantile(values, .95), maximum=max(values) if values else None)


def stream_summary(values):
    repeats = gaps = resets = changes = 0
    deltas = []
    for old, new in zip(values, values[1:]):
        diff = new['id'] - old['id']
        repeats += diff == 0
        gaps += max(0, diff - 1)
        resets += diff < 0
        changes += old['domain'] != new['domain']
        if old['domain'] == new['domain'] and new['timestamp_ms'] > old['timestamp_ms']:
            deltas.append(new['timestamp_ms'] - old['timestamp_ms'])
    return dict(unique_ids=len({v['id'] for v in values}), repeats=repeats,
                observed_counter_gaps=gaps, counter_resets=resets, domain_changes=changes,
                domains=sorted({v['domain'] for v in values}),
                same_domain_positive_timestamp_delta_ms=distribution(deltas))


def cohort_summary(rows):
    unique = len({(r['depth']['id'], r['color']['id']) for r in rows})
    return dict(unique_cohorts=unique, repeated_cohorts=len(rows)-unique)


def aggregate(trials):
    groups = {}
    for trial in trials:
        if trial['status'] != 'ok':
            continue
        key = trial['profile'] + '/' + trial['mode']
        groups.setdefault(key, []).append(trial)
    return {key: dict(successful_trials=len(group), median_of_trial_p95_align_wall_ms=statistics.median(
        t['metrics']['align_wall_ms']['p95'] for t in group)) for key, group in groups.items()}


def sdk_profile(p):
    v = p.as_video_stream_profile()
    return dict(stream=str(p.stream_type()).split('.')[-1], format=str(p.format()).split('.')[-1],
                width=v.width(), height=v.height(), fps=p.fps())


def device(rs):
    devices = [d for d in rs.context().query_devices() if d.get_info(rs.camera_info.product_id).lower() == '0b3a']
    if len(devices) != 1:
        raise RuntimeError('require exactly one USB PID 0b3a D435i')
    d = devices[0]
    serial = d.get_info(rs.camera_info.serial_number)
    if os.environ.get('D435I_EXPECTED_SERIAL', serial) != serial:
        raise RuntimeError('camera identity changed')
    return d, serial


def enumerate_profiles(d):
    out = []
    for sensor in d.query_sensors():
        for p in sensor.get_stream_profiles():
            if p.is_video_stream_profile():
                out.append(sdk_profile(p))
    return out


def camera_nodes():
    nodes = set()
    for root in Path('/sys/bus/usb/devices').glob('*'):
        try:
            if (root/'idVendor').read_text().strip() == '8086' and (root/'idProduct').read_text().strip() == '0b3a':
                nodes.add('/dev/bus/usb/%03d/%03d' % (int((root/'busnum').read_text()), int((root/'devnum').read_text())))
                real = root.resolve()
                for video in Path('/sys/class/video4linux').glob('*'):
                    if real in (video/'device').resolve().parents:
                        nodes.add('/dev/' + video.name)
        except FileNotFoundError:
            continue
    if not nodes:
        raise RuntimeError('no identified D435i USB nodes')
    return nodes


def refuse_busy():
    targets = camera_nodes()
    owners = set()
    opaque = 0
    for process in Path('/proc').glob('[0-9]*'):
        if int(process.name) == os.getpid():
            continue
        try:
            for fd in (process/'fd').iterdir():
                try:
                    if os.readlink(fd) in targets:
                        owners.add(int(process.name))
                except FileNotFoundError:
                    pass
        except FileNotFoundError:
            pass
        except PermissionError:
            opaque += 1
    if owners:
        raise RuntimeError('camera already owned by PID(s): ' + ','.join(map(str, sorted(owners))))
    return dict(visible_camera_owners=[], opaque_process_count=opaque,
                limitation='opaque process FDs not inspected; SDK exclusive-open refusal guards remaining ownership')


def provenance(rs, d):
    native = []
    for path in sorted({line.split()[-1] for line in Path('/proc/self/maps').read_text().splitlines()
                        if '/' in line and ('realsense' in line or 'libusb' in line)}):
        p = Path(path)
        if p.is_file():
            native.append(dict(name=p.name, sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
    module = Path(rs.__file__)
    try:
        version = importlib.metadata.version('pyrealsense2')
    except importlib.metadata.PackageNotFoundError:
        version = getattr(rs, '__version__', 'unavailable')
    cpu = next((line.split(':',1)[1].strip() for line in Path('/proc/cpuinfo').read_text().splitlines() if line.startswith('model name')), platform.processor())
    return dict(python=sys.version, module_name=module.name, module_sha256=hashlib.sha256(module.read_bytes()).hexdigest(),
                package_version=version, sdk_version=getattr(rs, '__version__', version), native_libraries=native,
                host_cpu=cpu, logical_cpus=os.cpu_count(), usb_type=d.get_info(rs.camera_info.usb_type_descriptor),
                firmware=d.get_info(rs.camera_info.firmware_version))


def configure(rs, serial, name):
    cfg = rs.config()
    cfg.enable_device(serial)
    for p in PROFILES[name]:
        cfg.enable_stream(getattr(rs.stream,p['stream']), p['width'], p['height'], getattr(rs.format,p['format']), p['fps'])
    return cfg


def options(rs, d):
    result = []
    for sensor in d.query_sensors():
        values = {}
        for name in ('enable_auto_exposure', 'exposure', 'frames_queue_size'):
            opt = getattr(rs.option, name)
            if sensor.supports(opt):
                try:
                    values[name] = sensor.get_option(opt)
                except RuntimeError:
                    values[name] = 'query unavailable'
        result.append(dict(sensor=sensor.get_info(rs.camera_info.name), options=values))
    return result


def frame_metadata(frame):
    return dict(id=frame.get_frame_number(), timestamp_ms=frame.get_timestamp(), domain=str(frame.get_frame_timestamp_domain()))


def trial_run(rs, spec):
    ownership = refuse_busy()
    d, serial = device(rs)
    pipe = rs.pipeline()
    cfg = configure(rs, serial, spec['profile'])
    resolved = cfg.resolve(rs.pipeline_wrapper(pipe))
    verify_profiles(PROFILES[spec['profile']], [sdk_profile(p) for p in resolved.get_streams()])
    if resolved.get_device().get_info(rs.camera_info.serial_number) != serial:
        raise RuntimeError('resolve selected a different camera')
    queue = rs.frame_queue(1)
    align = rs.align(rs.stream.color) if spec['mode'] == 'sdk_align' else None
    started = False
    rows = []
    # Preallocated bounded reference slots; metadata dictionaries remain hot-path instrumentation.
    capacity = math.ceil(30 * (STEADY_S + WAIT_MS/1000)) + 128
    slots = [None] * capacity
    count = 0
    try:
        active = pipe.start(cfg, queue)
        started = True
        verify_profiles(PROFILES[spec['profile']], [sdk_profile(p) for p in active.get_streams()])
        startup_options = options(rs, active.get_device())
        warmup_end = time.monotonic() + WARMUP_S
        while time.monotonic() < warmup_end:
            queue.wait_for_frame(WAIT_MS)
        # Capacity one: discard at most one queued warmup cohort.
        queue.poll_for_frame()
        start = time.monotonic()
        cpu_start = time.process_time()
        while time.monotonic() - start < STEADY_S:
            frames = queue.wait_for_frame(WAIT_MS).as_frameset()
            received = time.monotonic()
            # A late frame is excluded from the fixed steady-state window.
            if received - start >= STEADY_S:
                break
            depth, color = frames.get_depth_frame(), frames.get_color_frame()
            if not depth or not color:
                raise RuntimeError('incomplete RGB-D cohort')
            if count == 0:
                verify_profiles(PROFILES[spec['profile']], [sdk_profile(color.get_profile()), sdk_profile(depth.get_profile())])
            row = dict(host_received_s=received-start, depth=frame_metadata(depth), color=frame_metadata(color), align_wall_ms=0., align_process_cpu_ms=0.)
            if align is not None:
                wall, cpu = time.monotonic(), time.process_time()
                aligned = align.process(frames)
                row['align_process_cpu_ms'] = (time.process_time()-cpu)*1000
                row['align_wall_ms'] = (time.monotonic()-wall)*1000
                ad, ac = aligned.get_depth_frame(), aligned.get_color_frame()
                expected = PROFILES[spec['profile']][0]
                got = sdk_profile(ad.get_profile())
                if got['width'] != expected['width'] or got['height'] != expected['height'] or got['format'] != 'z16':
                    raise RuntimeError('aligned Z16 grid differs from color grid')
                verify_profiles([expected], [sdk_profile(ac.get_profile())])
                if ad.get_units() != depth.get_units() or ac.get_frame_number() != color.get_frame_number() or ad.get_frame_number() != depth.get_frame_number():
                    raise RuntimeError('alignment changed native units or source cohort IDs')
            row['host_completed_s'] = time.monotonic()-start
            if count >= capacity:
                raise RuntimeError('bounded diagnostic capacity exceeded')
            slots[count] = row
            count += 1
        elapsed, cpu = time.monotonic()-start, time.process_time()-cpu_start
        steady_options = options(rs, active.get_device())
        rows = slots[:count]
        if len(rows) < 2:
            raise RuntimeError('fewer than two steady cohorts')
        observed_span = rows[-1]['host_received_s'] - rows[0]['host_received_s']
        streams = {key: stream_summary([r[key] for r in rows]) for key in ('depth','color')}
        metrics = dict(cohorts=len(rows), steady_wall_s=elapsed, process_cpu_s=cpu,
                       process_cpu_ms_per_cohort=1000*cpu/len(rows), logical_cpu_demand=cpu/elapsed,
                       observed_cohort_hz=(len(rows)-1)/observed_span,
                       completed_cohorts_per_steady_elapsed_s=len(rows)/elapsed,
                       completed_after_window=sum(r['host_completed_s'] > STEADY_S for r in rows),
                       host_receive_to_completion_ms=distribution([(r['host_completed_s']-r['host_received_s'])*1000 for r in rows]),
                       unique_stream_hz={key:(val['unique_ids']-1)/observed_span for key,val in streams.items()},
                       align_wall_ms=distribution([r['align_wall_ms'] for r in rows]),
                       align_process_cpu_ms=distribution([r['align_process_cpu_ms'] for r in rows]),
                       host_interarrival_ms=distribution([(b['host_received_s']-a['host_received_s'])*1000 for a,b in zip(rows, rows[1:])]),
                       streams=streams, **cohort_summary(rows))
        return dict(**spec, status='ok', ownership=ownership, queue='SDK frame_queue capacity=1, no Python callback', diagnostic_capacity=capacity,
                    startup_options=startup_options, end_options=steady_options, metrics=metrics, samples=rows, provenance=provenance(rs,d))
    finally:
        if started:
            pipe.stop()


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')
    path.chmod(0o600)



# Dynamic ownership observations are intentionally outside the immutable protocol.
PROTOCOL_FIELDS = ('seed', 'warmup_s', 'steady_s', 'wait_budget_ms',
                   'trial_timeout_s', 'profiles', 'trials', 'limitations')


def refuse_existing_capture(destination, internal_trial=False):
    if internal_trial:
        if destination.exists():
            raise ValueError('refuse existing trial directory')
    elif (destination/'results.json').exists() or any(destination.glob('trial-[0-9]*')):
        raise ValueError('refuse existing results or trial directories')


def save_plan(destination, plan, capture=False):
    path = destination/'preregistered-plan.json'
    if path.exists():
        existing = json.loads(path.read_text())
        if any(existing.get(key) != plan[key] for key in PROTOCOL_FIELDS):
            raise ValueError('existing preregistered protocol differs; use a fresh output directory')
        # Preserve the original bytes, including original ownership observation.
    else:
        write_json(path, plan)
    write_json(destination/('capture-preflight.json' if capture else 'planning-preflight.json'),
               dict(ownership=plan['ownership'], physical_stream_started=False))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--enumerate-only', action='store_true')
    mode.add_argument('--plan-only', action='store_true')
    mode.add_argument('--capture', action='store_true')
    parser.add_argument('--output-dir', type=Path, default=Path('/tmp/d435i-profile-benchmark-20261023'))
    parser.add_argument('--trial-spec', help=argparse.SUPPRESS)
    args = parser.parse_args()
    destination = args.output_dir.resolve()
    if not destination.is_relative_to(Path('/tmp')):
        parser.error('output directory must be private under /tmp')
    if args.capture:
        refuse_existing_capture(destination, internal_trial=bool(args.trial_spec))
    destination.mkdir(mode=0o700, parents=True, exist_ok=True)
    destination.chmod(0o700)
    import pyrealsense2 as rs  # Host fixtures never import device libraries.
    if args.trial_spec:
        if not args.capture:
            parser.error('internal trial requires explicit --capture')
        spec = json.loads(args.trial_spec)
        try:
            result = trial_run(rs, spec)
        except Exception as error:
            message = str(error).replace(os.environ.get('D435I_EXPECTED_SERIAL', '\0'), '[redacted]')
            result = dict(**spec, status='failed', error_type=type(error).__name__, error=message[:1000])
        write_json(destination/'worker-result.json', result)
        return 0 if result['status'] == 'ok' else 1
    ownership = refuse_busy()
    d, serial = device(rs)
    offered = enumerate_profiles(d)
    write_json(destination/'enumeration.json', dict(profiles=offered, ownership=ownership, provenance=provenance(rs,d), physical_stream_started=False))
    if args.enumerate_only:
        return 0
    select_profiles(offered)
    for name in PROFILES:
        pipe = rs.pipeline()
        resolved = configure(rs,serial,name).resolve(rs.pipeline_wrapper(pipe))
        verify_profiles(PROFILES[name], [sdk_profile(p) for p in resolved.get_streams()])
        if resolved.get_device().get_info(rs.camera_info.serial_number) != serial:
            raise RuntimeError('resolve selected another camera')
    plan = dict(seed=SEED, warmup_s=WARMUP_S, steady_s=STEADY_S, wait_budget_ms=WAIT_MS,
                trial_timeout_s=TRIAL_TIMEOUT_S, ownership=ownership, profiles=PROFILES, trials=make_plan(), physical_stream_started=False,
                limitations=['SDK CPU alignment; native GPU profiles above 640x480 unsupported',
                             'no geometry or physical station acceptance', 'counter gaps describe observed cohorts, not guaranteed hardware loss',
                             'host times are monotonic; no capture age inferred from device clock',
                             'process CPU includes SDK threads, RGB conversion and metadata instrumentation',
                             'aggregate uses median of per-trial p95, not pooled quantiles'])
    save_plan(destination, plan, capture=args.capture)
    print('Exact profiles resolved; preregistered 18 trials at ' + str(destination), flush=True)
    if args.plan_only:
        return 0
    # Drop all parent SDK objects before workers acquire exclusive device access.
    del resolved, pipe, d
    import gc
    gc.collect()
    results = []
    worker_env = dict(os.environ, D435I_EXPECTED_SERIAL=serial)
    for index, spec in enumerate(plan['trials']):
        trial_dir = destination / ('trial-%02d' % index)
        command = [sys.executable, str(Path(__file__).resolve()), '--capture', '--trial-spec', json.dumps(spec), '--output-dir', str(trial_dir)]
        try:
            completed = subprocess.run(command, env=worker_env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=TRIAL_TIMEOUT_S)
            result = json.loads((trial_dir/'worker-result.json').read_text())
        except (subprocess.TimeoutExpired, FileNotFoundError, json.JSONDecodeError):
            result = dict(**spec, status='failed', error='worker timeout or missing result')
        results.append(result)
        write_json(destination/'results.json', dict(trials=results, aggregate=aggregate(results)))
        print('Trial %d/18: %s %s %s' % (index+1,spec['profile'],spec['mode'],result['status']), flush=True)
    return 0 if all(t['status']=='ok' for t in results) else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except Exception as error:
        # Metadata/resolve SDK exceptions can contain device identity.
        print('Benchmark preflight failed: ' + type(error).__name__, file=sys.stderr)
        raise SystemExit(1) from None
