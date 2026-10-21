"""Pure cached-status regression; no ROS imports, runtime or hardware."""
import ast
import math
from pathlib import Path

import pytest

SOURCE=Path(__file__).resolve().parents[1]/'ros2/src/so101_station_control/so101_station_control/trajectory_server.py'
TREE=ast.parse(SOURCE.read_text())
FUNCTION=next(n for n in TREE.body if isinstance(n,ast.FunctionDef) and n.name=='_teleop_leader_readiness')
NAMESPACE={'math':math}
exec(compile(ast.Module(body=[FUNCTION],type_ignores=[]),str(SOURCE),'exec'),NAMESPACE)
readiness=NAMESPACE['_teleop_leader_readiness']


@pytest.mark.parametrize('stamp,expected,reason',[
    (None,False,'leader_pose_timestamp_invalid'),
    (True,False,'leader_pose_timestamp_invalid'),
    (float('nan'),False,'leader_pose_timestamp_invalid'),
    (float('inf'),False,'leader_pose_timestamp_invalid'),
    (9.8,False,'leader_pose_stale'),
    (9.95,True,'fresh_calibrated_leader_pose'),
    (10.,True,'fresh_calibrated_leader_pose'),
    (10.01,True,'fresh_calibrated_leader_pose'),
    (10.03,False,'leader_pose_future')])
def test_validated_cache_is_ready_only_within_existing_freshness_window(stamp,expected,reason):
    ready,actual,age=readiness({'stamp_s':stamp},10.,.15)
    assert ready is expected and actual==reason
    assert age is None or math.isfinite(age)


def test_absent_validated_cache_is_not_ready():
    assert readiness(None,10.,.15)==(False,'leader_pose_missing',None)


def test_configured_stale_window_and_invalid_configuration_fail_closed():
    assert readiness({'stamp_s':9.9},10.,.05)[0] is False
    assert readiness({'stamp_s':9.9},10.,.2)[0] is True
    for bound in (0.,-1.,float('nan'),float('inf')):
        assert readiness({'stamp_s':10.},10.,bound)[0] is False


def test_status_uses_existing_cache_and_preserves_packet_validation():
    bridge=next(n for n in TREE.body if isinstance(n,ast.ClassDef) and n.name=='TrajectoryBridge')
    status=next(n for n in bridge.body if isinstance(n,ast.FunctionDef) and n.name=='_status')
    cached=next(n for n in ast.walk(status) if isinstance(n,ast.With)
        and any(isinstance(i.context_expr,ast.Attribute) and i.context_expr.attr=='_teleop_latest_lock' for i in n.items))
    assert any(isinstance(n,ast.Attribute) and n.attr=='_teleop_latest' for n in ast.walk(cached))
    calls={n.func.id if isinstance(n.func,ast.Name) else n.func.attr for n in ast.walk(status)
        if isinstance(n,ast.Call) and isinstance(n.func,(ast.Name,ast.Attribute))}
    assert '_teleop_leader_readiness' in calls
    assert not calls & {'validate_pose_packet','create_subscription','read','read_text','open'}
    receiver=next(n for n in bridge.body if isinstance(n,ast.FunctionDef) and n.name=='_leader_pose_received')
    assert any(isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='validate_pose_packet' for n in ast.walk(receiver))
    strings={n.value for n in ast.walk(status) if isinstance(n,ast.Constant) and isinstance(n.value,str)}
    assert {'teleop_leader_ready','teleop_leader_reason','teleop_leader_age_s'}<=strings
