"""Read-only prerequisite inventory. Availability never certifies physical motion."""
import contextlib
import hashlib
import io
import json
import os
from pathlib import Path
import re
import subprocess


def probe(argv):
    return subprocess.run(argv, capture_output=True, text=True, check=True, timeout=5).stdout


def live_status(args, entry):
    """One bounded service-client child in the validated prepared ROS environment."""
    if args.profile != 'fwmini4':
        raise RuntimeError('Prepared native API wrapper unavailable')
    manifest = entry.manifest(args)
    source = ("import json\nfrom workshop.blockly_client import ROSProgramTransport\n"
              "client=ROSProgramTransport()\ntry:\n print(json.dumps(client('status','{}',2.0)))\n"
              "finally:\n client.close()\n")
    env = dict(os.environ, ROS_DOMAIN_ID=str(manifest['domain']), ROS_LOCALHOST_ONLY='1')
    reply = subprocess.run([manifest['wrapper'], 'ros-vulkan', 'python', '-c', source],
        cwd=entry.ROOT, env=env, capture_output=True, text=True, timeout=8, check=True)
    value = json.loads(reply.stdout)
    if not isinstance(value, dict) or value.get('physical') is not True:
        raise ValueError('Physical station status missing')
    return value


def collect(args, entry, *, status_provider=None, run_probe=probe, usb_root=Path('/sys/bus/usb/devices')):
    checks = []
    identity_failure = None
    dependent = {'preflight', 'runtime_paths', 'follower_calibration', 'serial_ports',
                 'live_api', 'ev3_reference', 'jaw_calibration', 'tcp_calibration',
                 'recordings', 'taught_skills'}
    def add(component, state, reason):
        if identity_failure is not None and component in dependent:
            state, reason = 'unknown', 'Not evaluated because manifest_identity failed: ' + identity_failure
        checks.append(dict(component=component, state=state, reason=reason))
    if getattr(args, 'profile', None) == 'fwmini4':
        # Inspect only bounded public identity metadata before the authoritative
        # validator: it deliberately refuses foreign roots before config parsing.
        try:
            with args.manifest.open('rb') as manifest_file:
                manifest_bytes = manifest_file.read(65537)
            if len(manifest_bytes) > 65536:
                raise ValueError('manifest exceeds64KiB')
            value = json.loads(manifest_bytes)
            if (not isinstance(value, dict) or not isinstance(value.get('root'), str)
                    or not Path(value['root']).is_absolute()):
                raise ValueError('manifest root is invalid')
        except (OSError, ValueError, TypeError):
            identity_failure = 'Manifest unavailable or malformed; requested checkout ' + str(entry.ROOT)
        else:
            if value['root'] != str(entry.ROOT):
                identity_failure = ('Requested checkout ' + str(entry.ROOT) + '; manifest root ' +
                                    value['root'] + '; ownership mismatch; source ownership not verified')
            else:
                try:
                    entry.manifest(args)  # Preserve schema, source cohort and ownership validation.
                except (OSError, ValueError, RuntimeError, KeyError, TypeError) as error:
                    identity_failure = ('Manifest identity/source validation failed (' +
                                        type(error).__name__ + '); requested checkout ' + str(entry.ROOT))
        add('manifest_identity', 'blocked' if identity_failure else 'available',
            identity_failure or 'Manifest ownership and source validation passed for requested checkout ' + str(entry.ROOT))
    if identity_failure is not None:
        for component in ('preflight', 'runtime_paths', 'follower_calibration', 'serial_ports'):
            add(component, 'unknown', '')
    else:
        # Keep the startup validator and hash gate authoritative, while continuing
        # independent diagnostics after its first failure. Never copy private config.
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                entry.preflight(args)
            add('preflight', 'available', 'Existing local startup prerequisite check passed')
        except Exception as error:
            add('preflight', 'blocked', 'Existing startup prerequisite check failed (' + type(error).__name__ + '); inspect startup configuration and paths')
        parsed = None
        try:
            parsed = entry.process.parse_arguments(entry.lifecycle_command(args, 'start')[2:])
            required = [Path(entry.process.wrapper(parsed)[0]), parsed.python,
                        parsed.calibration_dir / 'hs_rig01_leader.json',
                        entry.ROOT / 'ros2/.cache/realsense-cpu-texture-bounds/install']
            missing = sum(not path.exists() for path in required)
            add('runtime_paths', 'blocked' if missing else 'available', f'{missing} required wrapper/Python/leader-calibration/camera-overlay paths missing')
        except Exception:
            add('runtime_paths', 'blocked', 'Prepared lifecycle configuration unavailable or invalid')
        settings = None
        try:
            if parsed is None: raise ValueError('No config')
            settings = json.loads(parsed.config.read_text())
            arm = settings['arm']; calibration = Path(arm['calibration']['path'])
            if not calibration.is_absolute(): calibration = parsed.config.parent / calibration
            matches = hashlib.sha256(calibration.read_bytes()).hexdigest() == arm['calibration']['sha256']
            add('follower_calibration', 'available' if matches else 'blocked',
                'Follower calibration hash matches configured identity' if matches else 'Follower calibration hash mismatch')
        except Exception:
            add('follower_calibration', 'blocked', 'Follower calibration/configuration missing or invalid')
        try:
            ports = entry.configured_ports(settings)
            missing = sum(not port.exists() for port in ports)
            add('serial_ports', 'blocked' if missing else 'available',
                f'{missing} configured serial paths missing; presence only, no device opened')
        except Exception:
            add('serial_ports', 'blocked', 'Exact configured serial paths unavailable or invalid; no device opened')
    for component, identity in (('camera_usb', ('8086', '0b3a')), ('ev3_usb', ('0694', '0005'))):
        try:
            found = False
            for device in usb_root.iterdir():
                try:
                    if ((device / 'idVendor').read_text().strip().lower(),
                        (device / 'idProduct').read_text().strip().lower()) == identity: found = True
                except OSError: continue
            add(component, 'available' if found else 'blocked',
                ('Expected USB descriptor present' if found else 'Expected USB descriptor absent') +
                '; descriptor identity is not SDK serial or operational readiness')
        except OSError:
            add(component, 'unknown', 'USB descriptor inventory unavailable; no device opened')
    try:
        output = run_probe(['vulkaninfo', '--summary'])
        names = re.findall(r'deviceName\s*=\s*([^\n]+)', output)
        hardware = [name.strip() for name in names if not re.search('llvmpipe|lavapipe|software', name, re.I)]
        add('vulkan', 'available' if hardware else 'blocked',
            'Hardware Vulkan device enumerated; pipeline qualification pending' if hardware else 'No hardware Vulkan device enumerated; software Vulkan rejected')
    except Exception:
        add('vulkan', 'unknown', 'Bounded Vulkan summary unavailable; hardware not verified')
    try:
        output = run_probe(['ffmpeg', '-hide_banner', '-encoders'])
        families = [label for label, pattern in (('Vulkan', r'\b\w+_vulkan\b'),
            ('VAAPI', r'\b\w+_vaapi\b'), ('CPU', r'\b(?:libx264|libx265|libaom-av1|libsvtav1|mpeg4)\b')) if re.search(pattern, output)]
        add('codecs', 'available' if families else 'unknown',
            'Encoder enumeration: ' + (', '.join(families) or 'none recognized') + '; no encode/decode performed; profiles unqualified')
    except Exception:
        add('codecs', 'unknown', 'Bounded FFmpeg encoder enumeration unavailable')
    status = {}
    try:
        if identity_failure is not None:
            raise ValueError('Manifest identity not verified; no status client launched')
        status = (status_provider or (lambda: live_status(args, entry)))()
        if not isinstance(status, dict) or not status: raise ValueError('No status')
        add('live_api', 'available', 'Read-only station API status received; availability only')
    except Exception:
        add('live_api', 'unknown', 'Live API missing, invalid, or timed out; no owner or services started')
        status = {}
    backend = status.get('backend')
    backend = backend if isinstance(backend, dict) else {}
    streams = status.get('streams', backend.get('streams', {}))
    streams = streams if isinstance(streams, dict) else {}
    def fresh_data(topic):
        row = streams.get(topic, {})
        if not isinstance(row, dict): return {}
        age = row.get('age_s')
        data = row.get('data')
        return data if isinstance(data, dict) and type(age) in (int, float) and 0 <= age <= 2 else {}
    ev3 = fresh_data('/station/ev3/operator_status')
    refs = ev3.get('references')
    ready = ev3.get('fresh') is True and ev3.get('connected') is True and refs == {'sorter': 'home', 'feeder': 'eject'}
    add('ev3_reference', 'available' if ready else ('blocked' if ev3 else 'unknown'),
        'Fresh home/eject reference telemetry available; session/motion admission still required' if ready else 'Fresh connected home/eject reference telemetry unavailable')
    jaw = fresh_data('gripper_calibration')
    for component, flag in (('jaw_calibration', 'si_gap_calibrated'), ('tcp_calibration', 'tcp_calibrated')):
        value = jaw.get(flag, status.get(flag))
        add(component, 'available' if value is True else ('blocked' if value is False else 'unknown'),
            'Live calibration flag present; operation admission still required' if value is True else 'Qualified live calibration flag absent or false')
    recordings = status.get('recording_info')
    if isinstance(recordings, dict):
        invalid = sum(isinstance(r, dict) and (r.get('state') == 'invalid' or r.get('valid') is False) for r in recordings.values())
        pending = sum(not isinstance(r, dict) or r.get('state') != 'usable' or r.get('needs_revalidation') is True for r in recordings.values())
        add('recordings', 'unqualified' if pending else 'available',
            f'{len(recordings)} recordings; {invalid} invalid; {pending} require validation/revalidation; each replay requires fresh admission')
    else: add('recordings', 'unknown', 'Live recording_info unavailable')
    catalog = status.get('taught_skills')
    if isinstance(catalog, dict) and catalog.get('state') == 'missing':
        add('taught_skills', 'blocked', 'Private taught skill library is not configured')
    elif (isinstance(catalog, dict) and catalog.get('state') == 'configured'
            and isinstance(catalog.get('skills'), dict) and len(catalog['skills']) <= 64):
        add('taught_skills', 'unqualified',
            f"{len(catalog['skills'])} named manifests; commissioning and fresh identity/calibration/physical evidence gates still required")
    else:
        add('taught_skills', 'unknown', 'Live bounded taught skill catalog unavailable')
    add('gpu_raw_alignment', 'unqualified', 'Experimental raw depth/color GPU alignment remains unqualified for production deployment')
    add('codec_qualification', 'unqualified', 'Encoder enumeration cannot qualify a recording/encode/decode profile')
    add('octomap', 'unqualified', 'Opt-in 20 mm/2 Hz GPU-downsampled input; stream, self-filter and collision qualification pending')
    return dict(schema='so101-station-doctor-v1', checks=checks,
        prerequisites_ready=not any(row['state'] in ('blocked', 'unknown') for row in checks), qualified_for_motion=False)


def render(report, *, as_json=False):
    if as_json: return json.dumps(report, indent=2)
    return '\n'.join([f"{row['component']}: {row['state'].upper()} — {row['reason']}" for row in report['checks']] +
        [f"Prerequisites ready: {report['prerequisites_ready']}; qualified for motion: false"])
