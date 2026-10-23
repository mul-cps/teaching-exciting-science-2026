import copy
import math
import pytest
from workshop.blockly_recording import (Recorder, replay_points, stationary_pause_points,
                                        nominal_curve_bounds, recording_timing)

IDENTITY={'arm_id':'a','calibration_sha256':'b'*64,'jaw_calibration_sha256':'c'*64,'joint_calibration_sha256':'d'*64}


def record(points):
    recorder=Recorder('stroke',IDENTITY)
    for time,positions in points: recorder.add(positions,0,time)
    return recorder.finish()


def moving_edges(points):
    return [(a['positions'],b['positions']) for a,b in zip(points,points[1:])
            if a['positions']!=b['positions']]


def test_skip_only_exact_stationary_samples_preserves_5d_corners_and_source():
    positions=[[0]*5]*8+[[.02,0,0,0,0]]*7+[[.02,.03,.01,0,0]]*9+[[.02,.03,.01,.04,.01]]*6
    source=record([(i*.1,p) for i,p in enumerate(positions)]); frozen=copy.deepcopy(source)
    reduced=stationary_pause_points(source['points'])
    assert len(reduced)==16
    assert moving_edges(reduced)==moving_edges(source['points'])
    assert reduced[0]['positions']==source['points'][0]['positions']
    assert reduced[-1]['positions']==source['points'][-1]['positions']
    assert source==frozen
    assert all(b['time']>a['time'] for a,b in zip(reduced,reduced[1:]))


def test_original_is_default_and_unknown_modes_reject():
    source=record([(0,[0]*5),(.1,[.01]*5)])
    assert replay_points(source,IDENTITY)==replay_points(source,IDENTITY,timing='original')
    with pytest.raises(ValueError): replay_points(source,IDENTITY,timing='fast')
    assert stationary_pause_points(source['points'])==source['points']


def test_constant_600_and_nonuniform_timestamps():
    source=record([(i*.099,[.2]*5) for i in range(600)])
    reduced=stationary_pause_points(source['points'])
    assert len(reduced)==4 and reduced[0]['time']==0
    output=replay_points(source,IDENTITY,timing='skip_stationary_pauses',start_radians=dict.fromkeys(source['joint_names'],.2))
    assert len(output)==4
    assert recording_timing(source)['original_duration_s']==source['points'][-1]['time']
    nonuniform=record([(0,[0]*5),(.07,[0]*5),(.19,[0]*5),(.29,[0]*5),(.44,[0]*5),(.57,[0]*5),(.7,[.01]*5)])
    reduced=stationary_pause_points(nonuniform['points'])
    assert moving_edges(reduced)==moving_edges(nonuniform['points'])
    assert all(b['time']>a['time'] for a,b in zip(reduced,reduced[1:]))


def test_nominal_guard_uses_same_current_start_and_rejects_increased_bound():
    source=record([(i*.1,[0]*5 if i<10 else [.01]*5) for i in range(20)])
    start=dict.fromkeys(source['joint_names'],0.);start['shoulder_pan']=.005
    baseline,candidate=nominal_curve_bounds(source['joint_names'],source['points'],stationary_pause_points(source['points']),start)
    assert math.isfinite(baseline) and math.isfinite(candidate)
    if candidate<=baseline:
        assert replay_points(source,IDENTITY,timing='skip_stationary_pauses',start_radians=start)
    else:
        with pytest.raises(ValueError,match='original timing'): replay_points(source,IDENTITY,timing='skip_stationary_pauses',start_radians=start)


def test_5d_corner_guard_rejects_larger_envelope_without_changing_original():
    positions=[[0]*5]*6+[[.01,0,0,0,0]]*6+[[.01,.02,0,0,0]]*6+[[.01,.02,.03,.01,.04]]*6
    source=record([(i*.1,p) for i,p in enumerate(positions)])
    original=copy.deepcopy(source)
    with pytest.raises(ValueError,match='original timing'):
        replay_points(source,IDENTITY,timing='skip_stationary_pauses',start_radians=dict.fromkeys(source['joint_names'],0.))
    assert replay_points(source,IDENTITY,timing='original')
    assert source==original


def test_program_timing_is_explicit_optional_and_strict():
    from workshop.blockly_program import validate_program
    base={'op':'replay','name':'stroke','speed':.5}
    assert validate_program([base])
    assert validate_program([dict(base,timing='original')])
    assert validate_program([dict(base,timing='skip_stationary_pauses')])
    for bad in (dict(base,timing='fast'),dict(base,timing=None),dict(base,timing='original',relative=True)):
        with pytest.raises(ValueError): validate_program([bad])


def test_nominal_guard_is_lazy_and_outside_feedback_callback():
    import ast
    from pathlib import Path
    tree=ast.parse((Path(__file__).resolve().parents[1]/'workshop/blockly_recording.py').read_text())
    for node in tree.body:
        assert not isinstance(node,ast.ImportFrom) or 'ros_trajectory' not in (node.module or '')
    tree=ast.parse((Path(__file__).resolve().parents[1]/'workshop/blockly_ros.py').read_text())
    callback=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='_joints')
    assert not any(isinstance(n,ast.Name) and n.id in {'nominal_curve_bounds','replay_points','_spline'} for n in ast.walk(callback))


def test_transformation_never_inserts_current_pose_or_offsets_taught_positions():
    source=record([(i*.1,[.02]*5) for i in range(20)])
    current=dict.fromkeys(source['joint_names'],.025)
    output=replay_points(source,IDENTITY,timing='skip_stationary_pauses',start_radians=current)
    assert len(output)==4
    assert all(row['positions']==[.02]*5 for row in output)
    assert output[0]['positions']==source['points'][0]['positions']
    baseline,candidate=nominal_curve_bounds(source['joint_names'],source['points'],stationary_pause_points(source['points']),current)
    assert candidate<=baseline


def test_loaded_nonzero_origin_metadata_matches_requested_elapsed_durations():
    source=record([(i*.1,[.2]*5) for i in range(10)])
    for row in source['points']: row['time']+=10.
    frozen=copy.deepcopy(source)
    summary=recording_timing(source)
    original=replay_points(source,IDENTITY,speed=1.)
    shortened=replay_points(source,IDENTITY,speed=1.,timing='skip_stationary_pauses',
                            start_radians=dict.fromkeys(source['joint_names'],.2))
    assert summary['original_duration_s']==pytest.approx(original[-1]['time'])
    assert summary['skip_stationary_pauses_duration_s']==pytest.approx(shortened[-1]['time'])
    assert source==frozen
