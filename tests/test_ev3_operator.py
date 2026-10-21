"""No ROS/hardware: bounded panel projection and admission visibility."""
import json
import unittest
from workshop.station.ev3_operator import operator_status

class EV3OperatorStatusTests(unittest.TestCase):
    def state(self, **changes):
        native = dict(connected=True, busy=False, session='test', references={'sorter':'home','feeder':'eject'},
                      angles={'sorter':132,'feeder':0}, power_mode='awake', touch=False, color='RED', last_error=None)
        native.update(changes)
        return operator_status({'connected':True,'status':native}, {'state':'idle'}, {}, fresh=True)

    def test_existing_actions_and_no_invented_endpoints(self):
        caps = self.state()['capabilities']
        self.assertTrue(caps['home']['enabled'])
        self.assertTrue(caps['eject']['enabled'])
        self.assertFalse(caps['left_end']['enabled'])
        self.assertFalse(caps['right_end']['enabled'])

    def test_configured_endpoints_use_interior_values_only(self):
        native = dict(busy=False, references={'sorter':'home'}, power_mode='awake')
        for target, expected in ((10., True), (530., True), (0., False), (550., False), (float('nan'), False)):
            result = operator_status({'connected':True,'status':native}, {'state':'idle'}, {},
                                     fresh=True, endpoints={'left':target})
            self.assertEqual(result['capabilities']['left_end']['enabled'], expected)

    def test_missing_feeder_reference_blocks_eject(self):
        self.assertFalse(self.state(references={'sorter':'home','feeder':None})['capabilities']['eject']['enabled'])

    def test_busy_and_fault_block_motion(self):
        for changes in ({'busy':True}, {'last_error':{'code':'fault'}}):
            self.assertFalse(self.state(**changes)['capabilities']['home']['enabled'])

    def test_stale_and_disconnect_block_motion(self):
        for cache, fresh in (({'connected':False}, True), ({'connected':True}, False)):
            result = operator_status(cache, {'state':'idle'}, {}, fresh=fresh)
            self.assertFalse(result['capabilities']['home']['enabled'])

    def test_projection_excludes_large_events_and_native_diagnostics(self):
        result = operator_status({'connected':True,'status':{'session':'x'*10000, 'last_error':'x'*10000}},
                                 {'state':'idle', 'events':['x'*100000], 'result':'x'*100000}, {}, fresh=True)
        self.assertLess(len(json.dumps(result)), 4096)
        self.assertNotIn('events', result['workflow'])
        self.assertNotIn('result', result['workflow'])

if __name__ == '__main__': unittest.main()
