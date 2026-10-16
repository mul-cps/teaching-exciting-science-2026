import threading
import time
import pytest
from workshop.station.ros_workflow import StationWorkflow
from tools.ros_station_action import parse_args


def joined(workflow):
    workflow.thread.join(2)
    assert not workflow.thread.is_alive()
    return workflow.snapshot()


def test_complete_fault_latch_and_explicit_reset():
    workflow = StationWorkflow()
    workflow.start('one', lambda cancel: {'completed': 1})
    assert joined(workflow)['result'] == {'completed': 1}
    def failed(cancel):
        raise RuntimeError('pickup unverified')
    workflow.start('sort', failed)
    assert joined(workflow)['state'] == 'fault'
    with pytest.raises(RuntimeError, match='fault latched'):
        workflow.start('sort', lambda cancel: None)
    workflow.reset()
    workflow.start('one', lambda cancel: {'completed': 1})
    assert joined(workflow)['fault'] is None


def test_cancel_serializes_jobs_and_cannot_report_success():
    workflow = StationWorkflow()
    entered = threading.Event()
    def wait(cancel):
        entered.set()
        assert cancel.wait(2)
        return {'completed': 100}
    workflow.start('sort', wait)
    assert entered.wait(1)
    with pytest.raises(RuntimeError, match='already active'):
        workflow.start('sort', lambda cancel: None)
    with pytest.raises(RuntimeError, match='active'):
        workflow.reset()
    workflow.stop()
    result = joined(workflow)
    assert result['state'] == 'fault'
    assert result['result'] is None
    assert 'cancelled' in result['fault']


def test_quick_commands_require_physical_ack_and_bound_cycles():
    with pytest.raises(SystemExit):
        parse_args(['confirm-reference', 'feeder'])
    with pytest.raises(SystemExit):
        parse_args(['run', '101'])
    assert parse_args(['run']).cycles == 1
    assert parse_args(['run', '20']).cycles == 20
    assert parse_args(['confirm-reference', 'feeder', '--physically-verified']).physically_verified
