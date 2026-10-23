"""Qualification collector status boundary; pure tests need no ROS or camera."""
import ast
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / 'tools/performance/collect_resident_live.py').read_text()


def method(name):
    tree = ast.parse(SOURCE)
    collector = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Collector')
    node = next(n for n in collector.body if isinstance(n, ast.FunctionDef) and n.name == name)
    return ast.get_source_segment(SOURCE, node)


class CollectorBoundary(unittest.TestCase):
    def test_startup_cannot_start_steady_clock_or_hide_unknown_measurements(self):
        # Removing the readiness branch would time startup as steady; moving the
        # unknown branch below begin would silently lose startup failures.
        status = method('on_status')
        self.assertIn('status_ready(', status)
        self.assertLess(status.index('self.status.append'), status.index('status_ready('))
        self.assertLess(status.index('status_ready('), status.index('self.begin=time.monotonic()'))
        self.assertLess(status.index('if not ready:return'), status.index('self.cpu0=cpu_ticks'))
        self.assertIn('except ValueError as error:self.errors.append(str(error));return', status)
        measurement = method('on_measurement')
        self.assertLess(measurement.index("msg.data.startswith('{')"), measurement.index('if self.begin is None:return'))
        self.assertLess(measurement.index("self.errors.append('unknown/failure measurement: '"), measurement.index('if self.begin is None:return'))


def ready(text, mode, already_ready=False):
    from tools.performance.resident_live_status import status_ready
    return status_ready(text, mode, already_ready)


SAFE = 'execution_eligible=false; geometry_admission=unqualified'
STARTING = 'starting; source_admitted=false; waiting_for_stable_source=true; ' + SAFE


class ResidentLiveStatus(unittest.TestCase):
    def test_explicit_starting_waits_for_every_mode(self):
        for mode in ('cpu', 'vulkan', 'auto'):
            with self.subTest(mode=mode):
                self.assertFalse(ready(STARTING, mode))

    def test_only_expected_ready_backend_can_start_the_window(self):
        for mode, backend, alignment in [('cpu', 'cpu', 'vendor_cpu'),
                                         ('vulkan', 'vulkan', 'raw_vulkan'),
                                         ('auto', 'vulkan', 'raw_vulkan')]:
            text = f'experimental; backend={backend}; alignment={alignment}; {SAFE}'
            with self.subTest(mode=mode):
                self.assertTrue(ready(text, mode))
                self.assertTrue(ready(text, mode, already_ready=True))

    def test_starting_after_readiness_is_a_failure(self):
        with self.assertRaises(ValueError):
            ready(STARTING, 'vulkan', already_ready=True)

    def test_unknown_or_unexpected_status_cannot_claim_readiness(self):
        for prefix in ('unknown', 'unexpected', '', 'ready'):
            text = f'{prefix}; backend=vulkan; alignment=raw_vulkan; {SAFE}'
            with self.subTest(prefix=prefix), self.assertRaises(ValueError):
                ready(text, 'vulkan')

    def test_starting_must_be_explicit_ineligible_and_without_backend_claims(self):
        for text in [STARTING.replace('source_admitted=false', 'source_admitted=true'),
                     STARTING.replace('source_admitted=false; ', ''),
                     STARTING.replace('waiting_for_stable_source=true', 'waiting_for_stable_source=false'),
                     STARTING.replace('waiting_for_stable_source=true; ', ''),
                     STARTING + '; backend=vulkan', STARTING + '; alignment=raw_vulkan',
                     STARTING + '; backend=', STARTING.replace('starting;', 'unrecognized;')]:
            with self.subTest(text=text), self.assertRaises(ValueError):
                ready(text, 'vulkan')

    def test_fatal_restart_or_unsafe_status_fails_even_during_startup(self):
        for text in [STARTING + '; fatal', STARTING + '; restart_required=true',
                     STARTING.replace('execution_eligible=false', 'execution_eligible=true'),
                     STARTING.replace('execution_eligible=false', 'execution_eligible=unknown'),
                     STARTING.replace('execution_eligible=false; ', ''),
                     STARTING.replace('geometry_admission=unqualified', 'geometry_admission=qualified'),
                     STARTING.replace('geometry_admission=unqualified', '')]:
            with self.subTest(text=text), self.assertRaises(ValueError):
                ready(text, 'cpu')

    def test_missing_or_wrong_ready_backend_and_auto_cpu_fallback_fail(self):
        for mode, text in [('cpu', f'experimental; backend=vulkan; alignment=raw_vulkan; {SAFE}'),
                           ('vulkan', f'experimental; backend=cpu; alignment=vendor_cpu; {SAFE}'),
                           ('auto', f'experimental; backend=cpu; alignment=vendor_cpu; fallback_reason=probe failed; {SAFE}'),
                           ('cpu', f'experimental; backend=cpu; {SAFE}'),
                           ('cpu', f'experimental; alignment=vendor_cpu; {SAFE}'),
                           ('cpu', f'experimental; backend=cpu; alignment=vendor_cpu; execution_eligible=false')]:
            with self.subTest(mode=mode, text=text), self.assertRaises(ValueError):
                ready(text, mode)


if __name__ == '__main__':
    unittest.main()
