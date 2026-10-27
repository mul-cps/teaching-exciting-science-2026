#!/usr/bin/env python3
"""Offline same-host RViz/camera coexistence evidence; never runs native code."""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import statistics

PROFILES = ('vga', '720p', '1080p')
PROFILE_EXTENTS = {'vga': [640, 480, 640, 480], '720p': [1280, 720, 1280, 720],
                   '1080p': [1920, 1080, 1280, 720]}
SOFTWARE = ('llvmpipe', 'softpipe', 'swrast', 'software rasterizer', 'swiftshader', 'gdi generic')


def number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('expected finite number')
    return value


def load(path):
    return json.loads(Path(path).read_text(), parse_constant=lambda x: (_ for _ in ()).throw(ValueError('nonfinite JSON '+x)))


def lines(path, mixed=False):
    rows, errors = [], []
    for i, line in enumerate(Path(path).read_text().splitlines(), 1):
        if not line.strip() or (mixed and not line.lstrip().startswith('{')):
            continue
        try:
            row = json.loads(line, parse_constant=lambda x: (_ for _ in ()).throw(ValueError('nonfinite JSON '+x)))
            if not isinstance(row, dict):
                raise ValueError('expected object')
            rows.append(row)
        except (ValueError, TypeError) as e:
            errors.append(f'line {i}: {e}')
    return rows, errors


def perception_summary(data, runner, mode, profile):
    errors = []
    expected_extents = PROFILE_EXTENTS[profile]
    if runner.get('profile_requested') != profile or runner.get('mode') != mode:
        errors.append('runner requested profile/mode does not match keyed input path')
    if runner.get('profile_matches') is not True or runner.get('actual_extents') != expected_extents:
        errors.append('runner profile safety/extents do not match requested profile')
    if data.get('mode_requested') != mode or data.get('errors') != []:
        errors.append('collector requested mode mismatch or nonempty/missing errors')
    expected_alignment = 'raw_vulkan' if mode == 'vulkan' else 'vendor_cpu'
    matches = [r for r in data['status_samples'] if r.get('text', '').startswith('experimental')
               and r.get('fields', {}).get('backend') == mode
               and r.get('fields', {}).get('alignment') == expected_alignment]
    if not matches:
        return {'errors': ['no first experimental status with expected backend/alignment'], 'window': None}
    begin = number(matches[0]['receipt_monotonic'])
    seconds = number(data['steady_seconds'])
    if seconds <= 0:
        raise ValueError('nonpositive collector steady interval')
    end = begin + seconds
    measurements = data['measurements']
    if not measurements:
        raise ValueError('no measured perception outputs')
    if any(not begin <= number(r['receipt_monotonic']) <= end for r in measurements):
        errors.append('measurement receipt outside inferred collector interval (no epsilon)')
    windows = {}
    for key in ('source_frame', 'depth_source_frame'):
        ids = [int(r['fields'][key]) for r in measurements]
        windows[key] = dict(first=ids[0], last=ids[-1], unique_ids=len(set(ids)),
                            missing_between_first_last=sum(max(0, b-a-1) for a, b in zip(ids, ids[1:])),
                            duplicates=sum(b == a for a, b in zip(ids, ids[1:])),
                            regressions=sum(b < a for a, b in zip(ids, ids[1:])))
        if windows[key]['duplicates'] or windows[key]['regressions']:
            errors.append(key+' duplicate/regressing measured IDs')
    n = windows['source_frame']['unique_ids']
    ages = sorted(number(r['collector_received_age_ms']) for r in measurements)
    p95 = ages[int(.95*(len(ages)-1))]
    cpu_ms = number(data['container_cpu_ms_proc_stat'])
    if cpu_ms < 0:
        raise ValueError('negative perception CPU')
    hz, cpu_output = n/seconds, cpu_ms/n
    for key, value in [('unique_measured_outputs', n), ('output_hz', hz),
                       ('container_cpu_ms_per_unique_output', cpu_output), ('collector_received_age_p95_ms', p95)]:
        if not math.isclose(number(data[key]), value, rel_tol=1e-9, abs_tol=1e-7):
            errors.append('collector summary mismatch: '+key)
    if not data.get('passed') or not runner.get('passed'):
        errors.append('collector/runner safety did not pass')
    if runner.get('logical_cpus') != 4:
        errors.append('four-logical-CPU denominator not verified')
    statuses = [r for r in data['status_samples'] if begin <= number(r['receipt_monotonic']) <= end
                and r.get('text', '').startswith('experimental')]
    counts = []
    for r in statuses:
        f = r.get('fields', {})
        extents = [f.get(k) for k in ('color_width', 'color_height', 'depth_width', 'depth_height')]
        if f.get('profile') != profile or extents != list(map(str, expected_extents)):
            errors.append('status profile/extents mismatch in steady interval')
        if f.get('backend') != mode or f.get('alignment') != expected_alignment:
            errors.append('backend/alignment changed in steady interval')
        if 'vulkan_allocation_count' in f:
            counts.append(int(f['vulkan_allocation_count']))
    distinct = sorted(set(counts))
    stable = bool(counts) and len(distinct) == 1 and (mode != 'cpu' or distinct == [0])
    if not stable:
        errors.append('missing/changing allocation safety counts')
    if data.get('execution_eligible') or data.get('full_station_qualified'):
        errors.append('unexpected upstream station qualification flag')
    return dict(errors=errors, container_pid=data.get('container_pid'),
                window=dict(begin_monotonic_approx=begin, end_monotonic_approx=end, steady_seconds=seconds,
                            method='first matching experimental status receipt + reported steady duration; actual collector begin immediately follows status receipt',
                            begin_offset_uncertainty='microseconds expected but not recorded/measured; no exact bound asserted'),
                metrics=dict(unique_output_hz=hz, unique_outputs=n, CPU_ms=cpu_ms,
                             total_four_core_CPU_percent=25*cpu_ms/1000/seconds, CPU_ms_per_output=cpu_output,
                             receipt_age_p95_ms=p95, age_semantics='host receipt ROS time to collector receipt; not exposure age or processing latency',
                             p95_method='sorted samples, floor(0.95*(n-1)); same collector estimator'),
                source_ID_windows=windows, allocation_safety=dict(stable=stable, counts=distinct, samples=len(counts)),
                physical_camera_capture=data.get('physical_camera_capture'),
                producer_periodic_counters_not_used_for_throughput=True)


def glx_summary(rows, begin, end):
    errors, groups, metadata = [], defaultdict(list), defaultdict(list)
    excluded = 0
    for r in rows:
        event = r.get('event')
        if event in ('slot_limit', 'symbol_error', 'clock_error'):
            errors.append('probe error: '+event)
        elif event == 'renderer':
            metadata[(r['display'], r['drawable'])].append(r)
        elif event == 'interval':
            start, finish = number(r['start_ns']), number(r['end_ns'])
            if start < begin*1e9 or finish > end*1e9:
                excluded += 1
                continue
            elapsed = number(r['elapsed_ns'])
            count = number(r['interval_swaps'])
            if finish <= start or elapsed != finish-start or count <= 0 or int(count) != count:
                errors.append('invalid/zero GLX interval')
                continue
            if int(r['width']) <= 0 or int(r['height']) <= 0:
                errors.append('zero GLX dimensions')
                continue
            groups[(r['display'], r['drawable'])].append(r)
    viewports = []
    for key, intervals in sorted(groups.items()):
        intervals = sorted(intervals, key=lambda r: r['start_ns'])
        local_errors = []
        if any(b['start_ns'] < a['end_ns'] for a, b in zip(intervals, intervals[1:])):
            local_errors.append('overlapping/duplicate intervals')
        meta = [m for m in metadata[key] if number(m['monotonic_ns']) <= intervals[0]['start_ns']]
        first = max(meta, key=lambda m: m['monotonic_ns']) if meta else None
        if not first or not first.get('metadata_complete'):
            local_errors.append('missing/incomplete renderer metadata')
        if first:
            if not first.get('direct_known') or first.get('direct') is not True:
                local_errors.append('direct rendering unverified')
            if any(s in str(first.get('renderer', '')).lower() for s in SOFTWARE):
                local_errors.append('software renderer')
            if not all(first.get(k) for k in ('vendor', 'renderer', 'version')):
                local_errors.append('missing renderer strings')
        seconds = sum(r['elapsed_ns'] for r in intervals)/1e9
        count = sum(r['interval_swaps'] for r in intervals)
        fps = [r['interval_swaps']/(r['elapsed_ns']/1e9) for r in intervals]
        dimensions = sorted({(int(r['width']), int(r['height'])) for r in intervals})
        viewport = dict(display=key[0], drawable=key[1], renderer_record=first,
                        dimensions=[dict(width=w, height=h) for w, h in dimensions],
                        largest_actual_area=max(w*h for w, h in dimensions),
                        interval_count=len(intervals), covered_seconds=seconds, interval_swaps=count,
                        weighted_swap_hz=count/seconds, min_interval_swap_hz=min(fps), max_interval_swap_hz=max(fps),
                        intervals=intervals, errors=local_errors, telemetry_valid=not local_errors)
        viewports.append(viewport)
        errors.extend(f'drawable {key}: {e}' for e in local_errors)
    if not viewports:
        errors.append('no fully contained GLX intervals; zero observations is invalid, not zero FPS')
    primary = max(viewports, key=lambda r: r['largest_actual_area']) if viewports else None
    return dict(errors=errors, viewports=viewports, primary_viewport=primary, excluded_not_fully_contained_intervals=excluded,
                selection='largest actual steady-interval drawable area; rates never summed across drawables',
                semantics='real glXSwapBuffers return + CPU bookkeeping, not physical scanout or new camera pixels')


def engines(first, last):
    result, warnings = {}, []
    before, after = first.get('drm_clients', {}), last.get('drm_clients', {})
    if set(before) != set(after):
        warnings.append('DRM client set changed; absent endpoint counters are not zero')
    for client in sorted(set(before) & set(after)):
        a, b = before[client], after[client]
        deltas = {}
        for key in set(a) | set(b):
            if not key.startswith('drm-engine-') or key.startswith('drm-engine-capacity-'):
                continue
            if key not in a or key not in b or not a[key].endswith(' ns') or not b[key].endswith(' ns'):
                warnings.append('missing/unrecognized engine counter '+client+'/'+key)
                continue
            delta = int(b[key].split()[0])-int(a[key].split()[0])
            if delta < 0:
                warnings.append('regressing engine counter '+client+'/'+key)
                continue
            deltas[key.removeprefix('drm-engine-')] = delta
        result[client] = dict(engine_delta_ns=deltas, start_fields=a, end_fields=b)
    return result, warnings


def observer_summary(rows, begin, end, perception_pid):
    errors = []
    rows = sorted(rows, key=lambda r: number(r['monotonic']))
    if len(rows) < 2:
        return dict(errors=['fewer than two observer rows'], window=None, processes={}, external_targets_total_four_core_CPU_percent=None)
    left = min(range(len(rows)), key=lambda i: abs(rows[i]['monotonic']-begin))
    right = min(range(len(rows)), key=lambda i: abs(rows[i]['monotonic']-end))
    a, b = rows[left], rows[right]
    duration = b['monotonic']-a['monotonic']
    if duration <= 0:
        return dict(errors=['nearest observer endpoints do not form positive window'], window=None, processes={}, external_targets_total_four_core_CPU_percent=None)
    selected = rows[left:right+1]
    if a['monotonic'] > end or b['monotonic'] < begin:
        errors.append('observer window does not overlap collector')
    rates = {r['clock_ticks_per_second'] for r in selected}
    if len(rates) != 1 or number(a['clock_ticks_per_second']) <= 0:
        raise ValueError('invalid/changing observer clock tick rate')
    hz = a['clock_ticks_per_second']
    processes, external, used_pid = {}, [], set()
    for name in sorted({n for r in selected for n in r['processes']}):
        states = [r['processes'].get(name, {'missing': True}) for r in selected]
        reasons = []
        if any(s.get('missing') for s in states):
            reasons.append('missing process sample')
        if not reasons:
            if len({(s['pid'], s['starttime_ticks']) for s in states}) != 1:
                reasons.append('PID/starttime changed')
            if any(y['cpu_ticks'] < x['cpu_ticks'] for x, y in zip(states, states[1:])):
                reasons.append('CPU counter regressed')
        if reasons:
            processes[name] = dict(errors=reasons, CPU_ms=None, total_four_core_CPU_percent=None)
            errors.extend(name+': '+e for e in reasons)
            continue
        first, last = states[0], states[-1]
        cpu_ms = (last['cpu_ticks']-first['cpu_ticks'])*1000/hz
        clients, warnings = engines(first, last)
        processes[name] = dict(pid=first['pid'], starttime_ticks=first['starttime_ticks'], errors=[],
                               CPU_ms=cpu_ms, total_four_core_CPU_percent=25*cpu_ms/1000/duration,
                               RSS_end_pages=last['rss_pages'], RSS_max_sampled_pages=max(s['rss_pages'] for s in states),
                               RSS_end_MiB_assuming_4096_byte_pages=last['rss_pages']/256,
                               RSS_page_size_note='sampler did not record page size; pages authoritative, MiB assumes4096 bytes',
                               DRM_clients=clients, DRM_accounting_warnings=warnings)
        if first['pid'] != perception_pid and first['pid'] not in used_pid:
            external.append(processes[name]['total_four_core_CPU_percent'])
            used_pid.add(first['pid'])
    host = None
    if len(a['host_cpu_ticks']) != 8 or len(b['host_cpu_ticks']) != 8:
        errors.append('host /proc/stat must contain first eight counters')
    else:
        deltas = [y-x for x, y in zip(a['host_cpu_ticks'], b['host_cpu_ticks'])]
        if any(x < 0 for x in deltas) or sum(deltas) <= 0:
            errors.append('invalid host CPU counter delta')
        else:
            host = 100*(sum(deltas)-deltas[3]-deltas[4])/sum(deltas)
    costs = [number(r['sampling_cost_seconds']) for r in selected]
    if any(c < 0 for c in costs):
        raise ValueError('negative sampling cost')
    return dict(errors=errors, window=dict(begin_monotonic=a['monotonic'], end_monotonic=b['monotonic'], seconds=duration,
                                          begin_offset_seconds=a['monotonic']-begin, end_offset_seconds=b['monotonic']-end,
                                          method='nearest sampled endpoint to each approximate collector bound', rows=len(selected)),
                processes=processes, external_targets_total_four_core_CPU_percent=sum(external) if not errors else None,
                external_target_PID_set=sorted(used_pid), host_busy_percent=host,
                host_busy_method='first eight /proc/stat deltas, busy=total-idle-iowait; includes all tasks including sampler',
                observer_process_CPU_included_in_target_sum=False,
                observer_process_CPU_note='sampler PID not recorded among targets; its own CPU is excluded from target sums, included in host busy',
                sampling_cost_seconds=dict(max=max(costs), mean=statistics.mean(costs), JSON_write_excluded=True),
                engine_semantics='per process and client ID counter deltas; not wall time, occupancy percentage, power or sum across processes')


def analyze(base):
    base = Path(base)
    runs = {}
    for profile in PROFILES:
        for lane, mode in [('vulkan', 'vulkan'), ('system-cpu', 'cpu')]:
            name = profile+'/'+mode
            inputs = dict(collector=str(base/lane/'live'/profile/(mode+'.json')),
                          runner=str(base/lane/'live'/profile/(mode+'.runner.json')),
                          GLX=str(base/'logs'/('rviz-'+profile+'.log')),
                          observer=str(base/'logs'/('processes-'+profile+'.jsonl')))
            out = dict(inputs=inputs, errors=[], qualified=False, execution_eligible=False)
            try:
                p = perception_summary(load(inputs['collector']), load(inputs['runner']), mode, profile)
                out['perception'] = p
                out['errors'].extend(p['errors'])
                if p['window'] is not None:
                    begin, end = p['window']['begin_monotonic_approx'], p['window']['end_monotonic_approx']
                    gl, ge = lines(inputs['GLX'], mixed=True)
                    obs, oe = lines(inputs['observer'])
                    out['GLX'] = glx_summary(gl, begin, end)
                    out['observer'] = observer_summary(obs, begin, end, p['container_pid'])
                    out['errors'].extend(['GLX parse: '+x for x in ge]+['observer parse: '+x for x in oe])
                    out['errors'].extend(out['GLX']['errors']+out['observer']['errors'])
                    external = out['observer']['external_targets_total_four_core_CPU_percent']
                    out['approximate_combined_CPU'] = dict(different_accounting_windows=True,
                        total_four_core_CPU_percent=p['metrics']['total_four_core_CPU_percent']+external if external is not None else None,
                        perception_percent=p['metrics']['total_four_core_CPU_percent'], external_targets_percent=external,
                        meaning='approximate sum: perception collector CPU plus deduplicated external targeted-process CPU over separate nearest-row observer window; not same-window accounting; observer excluded')
                out['analysis_valid'] = not out['errors']
            except (OSError, KeyError, ValueError, TypeError, ZeroDivisionError) as e:
                out['errors'].append(type(e).__name__+': '+str(e))
                out['analysis_valid'] = False
            runs[name] = out
    return dict(schema='rviz-coexistence-offline-v1', qualified=False, execution_eligible=False,
                analysis_valid=all(r['analysis_valid'] for r in runs.values()), runs=runs,
                RViz_uses_actual_camera_RGB=True, matched_camera_pixels_between_modes=False,
                scope='offline six runs sharing one persistent RViz/sampler log per profile; GLX and camera windows selected separately per mode',
                limits=['RViz actually displays camera RGB; robot poses/world cloud/brick markers are synthetic',
                        'swap return is not physical scanout or new-camera-image presentation; no FPS inferred from latency percentiles',
                        'perception and observer windows differ; combined CPU is approximate and excludes sampler CPU',
                        'same-host monotonic clock assumed; collector begin inferred from status receipt, not exact',
                        'no full station, power, geometry, control, physical or performance acceptance implied'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = analyze(args.base)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    return 0 if result['analysis_valid'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
