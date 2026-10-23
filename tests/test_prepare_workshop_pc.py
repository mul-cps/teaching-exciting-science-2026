"""The classroom launcher must isolate Python and preserve authenticated loopback access."""
import importlib.util
import os
import json
import hashlib
import tempfile
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import unittest
from unittest.mock import patch, Mock

spec = importlib.util.spec_from_file_location('prepare_workshop_pc', Path(__file__).resolve().parents[1] / 'tools/prepare_workshop_pc.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class LauncherTests(unittest.TestCase):
    def test_environment_isolated_without_mutating_parent(self):
        with patch.dict(os.environ, {'PYTHONPATH': '/robotics', 'PYTHONHOME': '/conda', 'CONDA_PREFIX': '/conda', 'ROS_DOMAIN_ID': '177'}):
            env = module.clean_env()
            self.assertNotIn('PYTHONPATH', env)
            self.assertNotIn('PYTHONHOME', env)
            self.assertNotIn('CONDA_PREFIX', env)
            self.assertEqual(env['ROS_DOMAIN_ID'], '177')
            self.assertEqual(os.environ['PYTHONPATH'], '/robotics')

    def test_second_click_reuses_authenticated_server(self):
        server = '{"root_dir": "' + str(module.ROOT) + '", "hostname": "127.0.0.1", "url": "http://127.0.0.1:8888/", "token": "abc"}\n'
        with patch.object(Path, 'is_file', return_value=True), patch.object(module.subprocess, 'check_output', return_value=server), patch.object(module.webbrowser, 'open') as browser:
            module.launch('student')
        browser.assert_called_once_with('http://127.0.0.1:8888/lab/tree/notebooks/Robot_Candy_Factory_Student.ipynb?token=abc')

    def test_station_refuses_missing_identified_devices(self):
        with tempfile.TemporaryDirectory() as temporary:
            home = Path(temporary)
            state = home / '.local/share/exciting-science/station'
            state.mkdir(parents=True)
            calibration = state / 'follower.json'
            calibration.write_text('{}')
            digest = hashlib.sha256(calibration.read_bytes()).hexdigest()
            identity = {'vendor_id': 6790, 'product_id': 21971, 'serial': 'NOT_CONNECTED_TEST'}
            measured_home = state / 'home.json'
            measured_home.write_text(json.dumps({'calibration_sha256': digest, 'usb_identity': identity}))
            config = {'arm': {'calibration': {'path': str(calibration), 'sha256': digest}, 'home_file': str(measured_home), 'usb': identity}}
            (state / 'station-fwmini4-20261022.json').write_text(json.dumps(config))
            leader = home / 'git/soarm101-highschool-demos/.calibration/hs_rig01_leader.json'
            leader.parent.mkdir(parents=True)
            leader.write_text('{}')
            with patch.object(Path, 'home', return_value=home), self.assertRaisesRegex(SystemExit, 'Connect the identified follower'):
                module.station_command()
            state = home / 'workshop-runtime'
            state.mkdir()
            with patch.object(Path, 'home', return_value=home), patch.object(module, 'CONTRACT', state / 'joint-limit-contract.json'), patch.object(Path, 'exists', return_value=True), patch.object(Path, 'is_dir', return_value=True), patch.object(module, 'station_manifest', return_value=(Mock(), {'domain':76})) as validate:
                command = module.station_command()
                self.assertFalse((state / 'joint-limit-contract.json').is_file())
                self.assertEqual(command, [sys.executable, str(module.ROOT / 'tools/station_process.py'),
                                          'start', '--profile', 'fwmini4', '--deployment', 'user-units',
                                          '--color-mask-backend', 'vulkan', '--bootstrap-owner', '--domain', '76'])
                validate.assert_called_once_with(home / '.local/share/exciting-science/station/station-fwmini4-20261022.json')
                self.assertEqual(module.station_command('00000000-0000-0000-0000-000000000001')[-2:],
                                 ['--launch-id', '00000000-0000-0000-0000-000000000001'])

    def test_station_identity_and_camera_repair_guards_remain_required(self):
        for guard in ('calibration', 'home', 'leader', 'camera'):
            with self.subTest(guard=guard), tempfile.TemporaryDirectory() as temporary:
                home = Path(temporary)
                state = home / '.local/share/exciting-science/station'
                state.mkdir(parents=True)
                calibration = state / 'follower.json'
                calibration.write_text('{}')
                digest = hashlib.sha256(calibration.read_bytes()).hexdigest()
                identity = {'serial': 'TEST_FOLLOWER'}
                measured_home = state / 'home.json'
                measured_home.write_text(json.dumps({'calibration_sha256': digest, 'usb_identity': identity}))
                config = {'arm': {'calibration': {'path': str(calibration), 'sha256': digest}, 'home_file': str(measured_home), 'usb': identity}}
                (state / 'station-fwmini4-20261022.json').write_text(json.dumps(config))
                leader = home / 'git/soarm101-highschool-demos/.calibration/hs_rig01_leader.json'
                leader.parent.mkdir(parents=True)
                leader.write_text('{}')
                expected = {'calibration': 'calibration hash mismatch', 'home': 'home identity mismatch',
                            'leader': 'leader calibration missing', 'camera': 'Camera runtime repair is missing'}[guard]
                if guard == 'calibration': calibration.write_text('{"changed": true}')
                if guard == 'home': measured_home.write_text(json.dumps({'calibration_sha256': digest, 'usb_identity': {'serial': 'OTHER'}}))
                if guard == 'leader': leader.unlink()
                with patch.object(Path, 'home', return_value=home), patch.object(Path, 'exists', return_value=True), patch.object(Path, 'is_dir', return_value=False), patch.object(module, 'station_manifest') as validate, self.assertRaisesRegex(SystemExit, expected):
                    module.station_command()
                validate.assert_not_called()

    def test_station_preflight_only_prints_validated_command(self):
        command = ['native-start', '--bootstrap-owner']
        with patch.object(module.sys, 'argv', ['prepare_workshop_pc.py', 'station-preflight']), patch.object(module, 'station_command', return_value=command), patch.object(module.subprocess, 'run') as run, patch('builtins.print') as output:
            module.main()
        run.assert_not_called()
        output.assert_called_once_with('native-start --bootstrap-owner')

    def test_frontend_lock_is_held_through_start_rviz_and_stop(self):
        with tempfile.TemporaryDirectory() as temporary:
            state = Path(temporary)
            phases = []
            def checked_run(command, **kwargs):
                with (state / 'station-launch.lock').open('a') as other:
                    with self.assertRaises(BlockingIOError):
                        module.fcntl.flock(other, module.fcntl.LOCK_EX | module.fcntl.LOCK_NB)
                phases.append(command)
                return Mock(returncode=0)
            units = Mock()
            units.component_command.return_value = ['rviz-only']
            with patch.object(module, 'STATE', state), patch.object(module, 'station_command', return_value=['native-start']), patch.object(module, 'station_manifest', return_value=(units, {'domain':76})), patch.object(module.subprocess, 'run', side_effect=checked_run):
                module.start_station()
            self.assertEqual(len(phases), 3)

    def test_manifest_is_loaded_with_existing_source_identity_validator(self):
        manifest = {'root': str(module.ROOT), 'parameters': {'station_config': str(Path.home() / '.local/share/exciting-science/station/station-fwmini4-20261022.json')}}
        with patch('tools.install_station_user_units.load_manifest', return_value=manifest) as load:
            units, result = module.station_manifest()
            self.assertIs(result, manifest)
            self.assertTrue(callable(units.component_command))
            load.assert_called_once_with(Path.home() / '.local/share/exciting-science/station/fwmini4-units.json')
            manifest['root'] = '/another/checkout'
            with self.assertRaisesRegex(ValueError, 'checkout/configuration'):
                module.station_manifest()
            manifest['root'] = str(module.ROOT)
            manifest['parameters']['station_config'] = '/another/station.json'
            with self.assertRaisesRegex(ValueError, 'checkout/configuration'):
                module.station_manifest()

    def test_missing_or_stale_manifest_prevents_start(self):
        for error in (FileNotFoundError('missing recipe'), ValueError('source cohort changed')):
            with self.subTest(error=error), patch.object(module, 'station_command', return_value=['native-start']), patch.object(module, 'station_manifest', side_effect=error), patch.object(module.subprocess, 'run') as run, patch.object(module.sys.stdin, 'isatty', return_value=False), self.assertRaises(SystemExit):
                module.start_station()
            run.assert_not_called()

    def station_lifecycle(self, outcomes):
        units = Mock()
        units.component_command.return_value = ['reviewed-wrapper', 'component:=rviz']
        manifest = {'reviewed': True, 'domain':76}
        with patch.object(module.uuid, 'uuid4', return_value='00000000-0000-0000-0000-000000000001'), tempfile.TemporaryDirectory() as temporary, patch.object(module, 'STATE', Path(temporary)), patch.object(module, 'station_command', return_value=['native-start']) as start_command, patch.object(module, 'station_manifest', return_value=(units, manifest)), patch.object(module.subprocess, 'run', side_effect=outcomes) as run, patch.object(module.sys.stdin, 'isatty', return_value=False):
            error = None
            try:
                module.start_station()
            except SystemExit as caught:
                error = caught.code
            start_command.assert_called_once_with('00000000-0000-0000-0000-000000000001')
            units.component_command.assert_called_once_with(manifest, 'rviz')
            return error, [call.args[0] for call in run.call_args_list]

    def test_successful_native_start_runs_only_rviz_then_managed_stop(self):
        code, commands = self.station_lifecycle([Mock(returncode=0), Mock(returncode=0), Mock(returncode=0)])
        self.assertIsNone(code)
        self.assertEqual(commands, [['native-start'], ['reviewed-wrapper', 'component:=rviz'],
                                   [sys.executable, str(module.ROOT / 'tools/station_process.py'), 'stop',
                                    '--profile', 'fwmini4', '--deployment', 'user-units',
                                    '--domain', '76', '--expected-launch-id', '00000000-0000-0000-0000-000000000001']])

    def test_failed_start_never_stops_preexisting_station_or_launches_rviz(self):
        code, commands = self.station_lifecycle([Mock(returncode=2)])
        self.assertEqual(code, 2)
        self.assertEqual(commands, [['native-start']])

    def test_start_exception_or_interrupt_never_cleans_up_another_station(self):
        for error in (OSError('start unavailable'), KeyboardInterrupt()):
            with self.subTest(error=error), tempfile.TemporaryDirectory() as temporary, patch.object(module, 'STATE', Path(temporary)), patch.object(module, 'station_command', return_value=['native-start']), patch.object(module, 'station_manifest', return_value=(Mock(), {})), patch.object(module.subprocess, 'run', side_effect=error) as run, self.assertRaises(type(error)):
                module.start_station()
            run.assert_called_once_with(['native-start'], check=False)

    def test_rviz_failure_and_interrupt_still_use_managed_stop(self):
        for outcome, expected in ((Mock(returncode=3), 3), (FileNotFoundError('rviz unavailable'), 1), (KeyboardInterrupt(), 130)):
            with self.subTest(outcome=outcome):
                code, commands = self.station_lifecycle([Mock(returncode=0), outcome, Mock(returncode=0)])
                self.assertEqual(code, expected)
                self.assertEqual(commands[-1][2], 'stop')
                self.assertEqual(len(commands), 3)

    def test_failed_stop_surfaces_and_does_not_delete_managed_record(self):
        for outcome in (Mock(returncode=2), OSError('stop unavailable')):
            with self.subTest(outcome=outcome), patch.object(Path, 'unlink') as unlink:
                code, commands = self.station_lifecycle([Mock(returncode=0), Mock(returncode=0), outcome])
                self.assertNotEqual(code, 0)
                self.assertEqual(commands[-1][2], 'stop')
                unlink.assert_not_called()

    def test_duplicate_station_button_cannot_start_another_graph(self):
        with tempfile.TemporaryDirectory() as temporary, patch.object(module, 'STATE', Path(temporary)), patch.object(module, 'station_command', return_value=['existing-wrapper']), patch.object(module, 'station_manifest', return_value=(Mock(), {})), patch.object(module.fcntl, 'flock', side_effect=BlockingIOError), patch.object(module.subprocess, 'run') as run, patch.object(module.sys.stdin, 'isatty', return_value=False), self.assertRaises(SystemExit):
            module.start_station()
        run.assert_not_called()

    def test_blockly_reuses_only_matching_managed_mode(self):
        with tempfile.TemporaryDirectory() as temporary:
            state = Path(temporary)
            (state / 'blockly-server.json').write_text(json.dumps({'process': {'pid': 123}, 'root': str(module.ROOT), 'physical': False}))
            with patch.object(module, 'STATE', state), patch('tools.station_process.same_process', return_value=True), patch.object(module, 'blockly_status', return_value={'physical': False}), patch.object(module.subprocess, 'Popen') as launch, patch.object(module.webbrowser, 'open') as browser:
                module.blockly('blockly-simulation')
                browser.assert_called_once_with('http://127.0.0.1:8877')
                with self.assertRaisesRegex(RuntimeError, 'other mode'):
                    module.blockly('blockly-physical')
                launch.assert_not_called()

    def test_blockly_stale_pid_is_never_signalled(self):
        with tempfile.TemporaryDirectory() as temporary:
            state = Path(temporary)
            (state / 'blockly-server.json').write_text(json.dumps({'process': {'pid': 123}, 'root': str(module.ROOT), 'physical': True}))
            with patch.object(module, 'STATE', state), patch('tools.station_process.same_process', return_value=False), patch('tools.station_process.cleanup_session') as cleanup:
                module.blockly('blockly-close')
                cleanup.assert_not_called()
            self.assertFalse((state / 'blockly-server.json').exists())
            self.assertEqual(len(list(state.glob('*.stale-*.json'))), 1)

    def test_blockly_refuses_unmanaged_occupied_port(self):
        with tempfile.TemporaryDirectory() as temporary, patch.object(module, 'STATE', Path(temporary)), patch.object(module.socket, 'socket') as socket, patch.object(module.subprocess, 'Popen') as launch:
            socket.return_value.__enter__.return_value.bind.side_effect = OSError('busy')
            with self.assertRaisesRegex(RuntimeError, 'unmanaged server'):
                module.blockly('blockly-simulation')
            launch.assert_not_called()

    def test_blockly_can_reopen_after_tcp_timewait(self):
        original_socket = module.socket.socket
        listener = original_socket(); listener.setsockopt(module.socket.SOL_SOCKET, module.socket.SO_REUSEADDR, 1)
        listener.bind(('127.0.0.1', 0)); listener.listen(1); port = listener.getsockname()[1]
        client = original_socket(); client.connect(('127.0.0.1', port))
        connection, _ = listener.accept(); connection.close()
        self.assertEqual(client.recv(1), b'')
        client.close(); listener.close()
        with original_socket() as probe:
            with self.assertRaises(OSError): probe.bind(('127.0.0.1', port))
        class Probe:
            def __init__(self): self.socket = original_socket()
            def __enter__(self): return self
            def __exit__(self, *args): self.socket.close()
            def setsockopt(self, *args): self.socket.setsockopt(*args)
            def bind(self, address): self.socket.bind((address[0], port if address[1] == 8877 else 0))
        identity = {'pid': 123, 'starttime': '10', 'pgrp': 123, 'session': 123}
        with tempfile.TemporaryDirectory() as temporary, patch.object(module, 'STATE', Path(temporary)), patch.object(module.socket, 'socket', Probe), patch('tools.station_process.process_identity', return_value=identity), patch.object(module, 'blockly_status', return_value={'physical': False}), patch.object(module.subprocess, 'Popen') as launch, patch.object(module.webbrowser, 'open'):
            launch.return_value.poll.return_value = None
            module.blockly('blockly-simulation')
            launch.assert_called_once()

    def test_blockly_reuse_address_still_refuses_active_listener(self):
        original_socket = module.socket.socket
        listener = original_socket(); listener.setsockopt(module.socket.SOL_SOCKET, module.socket.SO_REUSEADDR, 1)
        listener.bind(('127.0.0.1', 0)); listener.listen(1); port = listener.getsockname()[1]
        class Probe:
            def __init__(self): self.socket = original_socket()
            def __enter__(self): return self
            def __exit__(self, *args): self.socket.close()
            def setsockopt(self, *args): self.socket.setsockopt(*args)
            def bind(self, address): self.socket.bind((address[0], port))
        try:
            with tempfile.TemporaryDirectory() as temporary, patch.object(module, 'STATE', Path(temporary)), patch.object(module.socket, 'socket', Probe), patch.object(module.subprocess, 'Popen') as launch:
                with self.assertRaisesRegex(RuntimeError, 'unmanaged server'):
                    module.blockly('blockly-simulation')
                launch.assert_not_called()
        finally:
            listener.close()

    def test_blockly_physical_launch_is_explicit_and_uses_station_domain(self):
        identity = {'pid': 123, 'starttime': '10', 'pgrp': 123, 'session': 123}
        with tempfile.TemporaryDirectory() as temporary, patch.object(module, 'STATE', Path(temporary)), patch.object(module.socket, 'socket'), patch('tools.station_process.process_identity', return_value=identity), patch.object(module, 'blockly_status', return_value={'physical': True}), patch.object(module, 'station_manifest', return_value=(Mock(), {'domain':76})), patch.object(module.subprocess, 'Popen') as launch, patch.object(module.webbrowser, 'open'):
            launch.return_value.poll.return_value = None
            module.blockly('blockly-physical')
            command = launch.call_args.args[0]
            self.assertEqual(command[1:3], ['ros-vulkan', 'python'])
            self.assertIn('--physical', command)
            self.assertIn('--instructor', command)
            self.assertEqual(launch.call_args.kwargs['env']['ROS_DOMAIN_ID'], '76')
            self.assertTrue(launch.call_args.kwargs['start_new_session'])

    def test_blockly_probe_has_absolute_deadline_for_slow_drip(self):
        class Slow(BaseHTTPRequestHandler):
            def log_message(self, *args): pass
            def do_GET(self):
                self.send_response(200)
                self.send_header('Content-Length', '100')
                self.end_headers()
                try:
                    for _ in range(100):
                        self.wfile.write(b'x'); self.wfile.flush(); time.sleep(.1)
                except OSError:
                    pass
        server = ThreadingHTTPServer(('127.0.0.1', 0), Slow)
        thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
        try:
            started = time.monotonic()
            with self.assertRaisesRegex(RuntimeError, 'absolute two-second deadline'):
                module.blockly_status(server.server_port)
            self.assertLess(time.monotonic() - started, 4)
        finally:
            server.shutdown(); server.server_close(); thread.join(2)

    def test_blockly_probe_refuses_redirect_and_ignores_proxy(self):
        redirected = []
        class Redirect(BaseHTTPRequestHandler):
            def log_message(self, *args): pass
            def do_GET(self):
                if self.path != '/': redirected.append(self.path)
                self.send_response(302)
                self.send_header('Location', '/unexpected')
                self.end_headers()
        server = ThreadingHTTPServer(('127.0.0.1', 0), Redirect)
        thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
        try:
            with patch.dict(os.environ, {'HTTP_PROXY': 'http://127.0.0.1:1', 'http_proxy': 'http://127.0.0.1:1', 'NO_PROXY': '', 'no_proxy': ''}), self.assertRaisesRegex(RuntimeError, 'direct loopback probe failed'):
                module.blockly_status(server.server_port)
            self.assertEqual(redirected, [])
        finally:
            server.shutdown(); server.server_close(); thread.join(2)

    def test_blockly_probe_ignores_environment_proxy_and_returns_only_mode(self):
        class Local(BaseHTTPRequestHandler):
            def log_message(self, *args): pass
            def do_GET(self):
                body = b"window.STATION_TOKEN='local_test';" if self.path == '/' else b'{"physical":false,"private_status":"not returned"}'
                if self.path == '/api/status':
                    assert self.headers.get('X-Station-Token') == 'local_test'
                self.send_response(200); self.send_header('Content-Length', str(len(body)))
                self.end_headers(); self.wfile.write(body)
        server = ThreadingHTTPServer(('127.0.0.1', 0), Local)
        thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
        try:
            with patch.dict(os.environ, {'HTTP_PROXY': 'http://127.0.0.1:1', 'http_proxy': 'http://127.0.0.1:1', 'NO_PROXY': '', 'no_proxy': ''}):
                self.assertEqual(module.blockly_status(server.server_port), {'physical': False})
        finally:
            server.shutdown(); server.server_close(); thread.join(2)

    def test_blockly_simulation_command_has_no_ros_or_physical_flags(self):
        command = module.blockly_command(False)
        self.assertEqual(command[0], sys.executable)
        self.assertNotIn('--physical', command)
        self.assertNotIn('--instructor', command)
        self.assertNotIn('ros-vulkan', command)

    def test_new_server_has_loopback_and_default_auth(self):
        with patch.object(Path, 'is_file', return_value=True), patch.object(module.subprocess, 'check_output', return_value=''), patch.object(module.os, 'chdir'), patch.object(module.os, 'execve') as execute:
            module.launch('student')
        args = execute.call_args.args[1]
        self.assertIn('--ServerApp.ip=127.0.0.1', args)
        self.assertIn('--ServerApp.port_retries=0', args)
        self.assertFalse(any('token' in arg or 'password' in arg for arg in args))


if __name__ == '__main__':
    unittest.main()


def test_private_skills_argument_reaches_only_explicit_physical_command(tmp_path):
    import pytest
    private=tmp_path/'private-skills';private.mkdir()
    command=module.blockly_command(True,skills=private)
    assert command[command.index('--skills')+1]==str(private.resolve())
    assert '--skills' not in module.blockly_command(True)
    with pytest.raises(ValueError,match='physical'):
        module.blockly_command(False,skills=private)


def test_private_skills_configuration_cannot_silently_reuse_other_library(tmp_path):
    import pytest
    private=tmp_path/'private';private.mkdir()
    record={'process':{'pid':123},'root':str(module.ROOT),'physical':True,'skills':None}
    (tmp_path/'blockly-server.json').write_text(json.dumps(record))
    with patch.object(module,'STATE',tmp_path),patch('tools.station_process.same_process',return_value=True),patch.object(module,'blockly_status',return_value={'physical':True}),patch.object(module.subprocess,'Popen') as launch:
        with pytest.raises(RuntimeError,match='skill configuration'):
            module.blockly('blockly-physical',skills=private)
        launch.assert_not_called()
