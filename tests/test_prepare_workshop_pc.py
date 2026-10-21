"""The classroom launcher must isolate Python and preserve authenticated loopback access."""
import importlib.util
import os
import json
import hashlib
import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch

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

    def test_duplicate_station_button_cannot_start_another_graph(self):
        with tempfile.TemporaryDirectory() as temporary, patch.object(module, 'STATE', Path(temporary)), patch.object(module, 'station_command', return_value=['existing-wrapper']), patch.object(module.fcntl, 'flock', side_effect=BlockingIOError), patch.object(module.subprocess, 'run') as run, patch.object(module.sys.stdin, 'isatty', return_value=False), self.assertRaises(SystemExit):
            module.start_station()
        run.assert_not_called()

    def test_new_server_has_loopback_and_default_auth(self):
        with patch.object(Path, 'is_file', return_value=True), patch.object(module.subprocess, 'check_output', return_value=''), patch.object(module.os, 'chdir'), patch.object(module.os, 'execve') as execute:
            module.launch('student')
        args = execute.call_args.args[1]
        self.assertIn('--ServerApp.ip=127.0.0.1', args)
        self.assertIn('--ServerApp.port_retries=0', args)
        self.assertFalse(any('token' in arg or 'password' in arg for arg in args))


if __name__ == '__main__':
    unittest.main()
