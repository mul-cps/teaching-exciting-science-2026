"""No hardware: exact native prewrite rejection receipts."""
import json
import os
import pytest
from test_arm_session import make_session, _collision_path_digest
from workshop.station.arm_session import ArmPlanPreflightError


def prepared(tmp_path):
    session, arm, _ = make_session(tmp_path)
    session.initialize()
    names = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
    raw = arm.bus.sync_read('Present_Position')
    start = session._arm_radians(raw)
    points = [{'time': 0., 'positions': [start[j] for j in names]},
              {'time': 1., 'positions': [start[j] + .005 for j in names]}]
    digest = _collision_path_digest(session, arm, names, points)
    return session, arm, names, raw, points, digest


def test_one_count_jitter_rejects_before_any_actuation(tmp_path):
    session, arm, names, raw, points, digest = prepared(tmp_path)
    arm.bus.data['Present_Position']['wrist_flex'] += 1
    with pytest.raises(ArmPlanPreflightError, match='differs from collision-checked path'):
        session.execute_trajectory('jitter', names, points, expected_path_sha256=digest)
    assert arm.bus.writes == [] and arm.bus.goal_history == []


def test_exact_mismatch_emits_correlated_negative_actuation_receipt(tmp_path):
    session, arm, names, raw, points, digest = prepared(tmp_path)
    arm.bus.data['Present_Position']['wrist_flex'] += 1
    read_fd, write_fd = os.pipe()
    command = dict(command='execute_trajectory', goal_id='jitter', request_id='a'*32,
                   checked_start_raw=raw, joint_names=names, points=points,
                   path_sha256=digest, ros_motion_profile='default')
    os.write(write_fd, (json.dumps(command)+'\n').encode())
    os.close(write_fd)
    replies=[]
    def emit(reply):
        assert arm.bus.writes == [] and arm.bus.goal_history == []
        replies.append(reply)
    try: session.serve(fd=read_fd, emit=emit)
    finally: os.close(read_fd)
    rejection = replies[0]
    proof = rejection['trajectory_start_rejection']
    assert rejection['error_type'] == 'ArmPlanPreflightError'
    assert proof['no_actuation'] is True
    assert proof['request_id'] == 'a'*32 and proof['goal_id'] == 'jitter'
    assert proof['owner_session_id'] == session._owner_session_id
    assert proof['owner_source_sha256'] == session._owner_source_sha256
    assert proof['checked_start_raw'] == raw
    assert proof['actual_start_raw']['wrist_flex'] == raw['wrist_flex'] + 1
    assert proof['checked_path_sha256'] == digest
    assert proof['actual_path_sha256'] != digest


def test_legacy_mismatch_has_no_retry_receipt(tmp_path):
    session, arm, names, raw, points, digest = prepared(tmp_path)
    arm.bus.data['Present_Position']['wrist_flex'] += 1
    with pytest.raises(ArmPlanPreflightError) as error:
        session.execute_trajectory('legacy', names, points, expected_path_sha256=digest)
    assert not hasattr(error.value, 'trajectory_start_rejection')
    assert not arm.bus.writes


def test_generic_error_after_profile_write_cannot_claim_no_actuation(tmp_path, monkeypatch):
    session, arm, names, raw, points, digest = prepared(tmp_path)
    def wrote_profile():
        arm.bus.write('Acceleration','wrist_flex',8,normalize=False,num_retry=0)
        raise ArmPlanPreflightError('generic late preflight error')
    monkeypatch.setattr(session, '_ensure_profile', wrote_profile)
    with pytest.raises(ArmPlanPreflightError) as error:
        session.execute_trajectory('late', names, points, expected_path_sha256=digest,
                                   request_id='a'*32, checked_start_raw=raw)
    assert arm.bus.writes
    assert not hasattr(error.value, 'trajectory_start_rejection')


@pytest.mark.parametrize('context', [dict(request_id='a'*32),dict(checked_start_raw={}),
                                    dict(request_id='bad',checked_start_raw={})])
def test_incomplete_receipt_context_rejects_without_writes_or_receipt(tmp_path,context):
    session, arm, names, raw, points, digest = prepared(tmp_path)
    with pytest.raises(ArmPlanPreflightError) as error:
        session.execute_trajectory('bad-context', names, points, expected_path_sha256=digest,**context)
    assert not arm.bus.writes and not hasattr(error.value,'trajectory_start_rejection')
