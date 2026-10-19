"""Offline stationary diagnostic contracts; no ROS or hardware required."""
import ast
import hashlib
import struct
from pathlib import Path

import numpy as np
import pytest

from workshop.digital_twin.registration_monitor import (
    BudgetExceeded, Deadline, MeshSamples, StationaryHistory, fit_registration,
    marker_residual, rigid,
)


def surface():
    rng = np.random.default_rng(17)
    points = rng.normal(size=(1500, 3))
    points /= np.linalg.norm(points, axis=1)[:, None]
    points *= np.array([.10, .07, .13])
    normals = points / np.array([.10, .07, .13]) ** 2
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    return points, normals


def test_bounded_fit_reports_proposal_and_never_qualifies():
    p, n = surface()
    observed = p + [.002, -.001, .001]
    result = fit_registration(p, n, np.repeat('body', len(p)), observed)
    assert result['qualified'] is False and result['applied'] is False
    assert result['after']['p95_surface_sample_m'] < result['before']['p95_surface_sample_m']
    assert result['candidate_delta'] is not None
    assert result['observed_population'] == len(observed)
    assert np.linalg.norm(np.array(result['candidate_delta'])[:3, 3]) <= .01
    assert result['observability']['rank'] == 6


def test_deadline_checked_inside_optimizer_and_outside():
    with pytest.raises(BudgetExceeded):
        Deadline(.2, clock=lambda: 1.).check(now=2.)
    p, n = surface()
    times = iter([0., 0., 0., 1.])
    budget = Deadline(.2, clock=lambda: next(times, 1.))
    with pytest.raises(BudgetExceeded):
        fit_registration(p, n, np.repeat('body', len(p)), p, deadline=budget)


def test_missing_coverage_and_planar_observability_do_not_invent_confidence():
    p = np.column_stack((np.linspace(-.1, .1, 100), np.zeros(100), np.zeros(100)))
    n = np.tile([0., 0., 1.], (100, 1))
    result = fit_registration(p, n, np.repeat('body', len(p)), p)
    assert result['candidate_delta'] is None
    assert result['state'] == 'insufficient_observability'
    assert result['observability']['rank'] < 6
    with pytest.raises(ValueError, match='coverage'):
        fit_registration(p, n, np.repeat('body', len(p)), p + 1.)


def test_stationary_window_rejects_future_outoforder_gaps_and_motion():
    h = StationaryHistory()
    for stamp in np.arange(1., 1.61, .05):
        h.add(float(stamp), {j: 0. for j in h.JOINTS}, now=2.)
    assert h.select(1.6, now=1.7)[0] == pytest.approx(1.6)
    assert not h.add(2., {j: 0. for j in h.JOINTS}, now=1.9)
    h.add(1.55, {j: .02 for j in h.JOINTS}, now=1.7)
    with pytest.raises(ValueError, match='moving'):
        h.select(1.6, now=1.7)
    with pytest.raises(ValueError):
        StationaryHistory().select(1.6, now=1.7)


def test_marker_mount_needs_explicit_identity_model_pin_and_full_pose():
    assert marker_residual(None, None, np.eye(4), 1., 'hash')['state'] == 'unavailable_missing_mount_or_observation'
    mount = {'schema': 'so101-link-marker-v1', 'qualified': False,
             'model_sha256': 'hash', 'dictionary': 'DICT_APRILTAG_25h9',
             'marker_id': 3, 'link': 'gripper_link', 'link_from_marker': np.eye(4).tolist()}
    observation = {'stamp': 1., 'association': 'unique_id', 'detection_count':1, 'dictionary': mount['dictionary'],
                   'marker_id': 3, 'camera_from_marker': np.eye(4).tolist()}
    result = marker_residual(mount, observation, np.eye(4), 1., 'hash')
    assert result['translation_m'] == 0. and result['qualified'] is False
    for update in ({'association': 'unassociated'}, {'stamp': 2.}, {'marker_id': 0}):
        assert marker_residual(mount, observation | update, np.eye(4), 1., 'hash')['state'] == 'unavailable_invalid_binding'
    assert marker_residual(mount, observation, np.eye(4), 1., 'newhash')['state'] == 'unavailable_invalid_binding'


def test_rigid_rejects_reflection_nan_and_shape():
    for value in (np.zeros((3, 3)), np.diag([-1., 1., 1., 1.]), np.full((4,4), np.nan)):
        with pytest.raises(ValueError): rigid(value)


def test_mesh_cache_loads_all_visuals_scales_origins_once(tmp_path):
    vertices = np.array([[0,0,0], [1,0,0], [0,1,0]], '<f4')
    raw = b' ' * 80 + struct.pack('<I',1) + struct.pack('<12fH',0,0,1,*vertices.reshape(-1),0)
    path = tmp_path / 'mesh.stl'; path.write_bytes(raw)
    xml = f'<robot name="test"><link name="body"><visual><origin xyz="0 0 1"/><geometry><mesh filename="{path}" scale=".1 .1 .1"/></geometry></visual></link><link name="gripper_frame_link"/></robot>'
    cache = MeshSamples(xml, lambda _: '', samples=100)
    before = path.stat().st_mtime_ns
    p,n,labels = cache.at({'body':np.eye(4)})
    assert len(p)==100 and np.allclose(p[:,2],1.) and np.max(p[:,:2])<=.1
    assert np.allclose(n[:,2],1.) and set(labels)=={'body'}
    path.unlink()
    assert cache.at({'body':np.eye(4)})[0] == pytest.approx(p)
    assert cache.model_sha256 == hashlib.sha256(xml.encode()).hexdigest()
    assert 'gripper_frame_link' in cache.all_links and 'gripper_frame_link' not in cache.links


def test_no_per_point_python_loop_or_tick_file_parse_in_fitter():
    import inspect
    tree=ast.parse(inspect.getsource(fit_registration))
    for node in ast.walk(tree):
        if isinstance(node,(ast.For,ast.While,ast.ListComp,ast.GeneratorExp,ast.DictComp)):
            # The sole comprehension enumerates the bounded link labels for reporting.
            assert isinstance(node,ast.DictComp) and ast.unparse(node.generators[0].iter)=='np.unique(labels[indices])'
    source=inspect.getsource(fit_registration)
    assert '.read_' not in source and 'glRead' not in source and 'workers=1' in source


def test_owner_bracket_real_clocks_identity_raw_and_pose_required():
    from datetime import datetime, timezone
    from workshop.digital_twin.registration_monitor import owner_bracket
    cohort={'owner_session_id':'session','arm_id':'rig','calibration_sha256':'cal','owner_source_sha256':{'arm':'source'}}
    joints={j:0. for j in StationaryHistory.JOINTS}
    def item(stamp,raw=2000):
        return {'state':dict(cohort,schema=1,event='sample',timestamp_utc=datetime.fromtimestamp(stamp,timezone.utc).isoformat(),
                             monotonic=stamp-100.,raw={j:raw for j in joints},pose=joints,
                             violations=[],torque={j:1 for j in joints},
                             arm_bounds={'classification':'normal','admitted_min':{j:1000 for j in joints},
                                         'admitted_max':{j:3000 for j in joints}}),
                'utc_before':stamp+.01,'utc_after':stamp+.011,'mono_before':stamp-99.99,'mono_after':stamp-99.989}
    reads=[item(101.),item(101.05)]
    assert owner_bracket(reads,101.02,joints,cohort,now=101.1)['raw']['gripper']==2000
    for field,value in [('phase','trajectory'),('trajectory_goal_id','goal'),('component_command',{}),('camera_recovery_phase','waiting'),('owner_session_id','other-session')]:
        active=[dict(r,state=r['state']|{field:value}) for r in reads]
        with pytest.raises(ValueError):owner_bracket(active,101.02,joints,cohort,now=101.1)
    for invalid in ([item(101.,2000),item(101.05,2001)], [item(100.8),item(101.2)],
                    [item(101.)], [item(101.)|{'mono_after':50.},item(101.05)],
                    [item(101.),dict(item(101.05),state=item(101.05)['state']|{'owner_session_id':'new'})]):
        with pytest.raises(ValueError):owner_bracket(invalid,101.02,joints,cohort,now=101.3)


def test_ros_node_static_readonly_bounded_and_exact_stamp_contract():
    source=(Path(__file__).resolve().parents[1]/'ros2/src/so101_station_perception/so101_station_perception/registration_monitor.py').read_text()
    tree=ast.parse(source)
    names={n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)}
    assert not {'sendTransform','set_parameters','create_action_client','send_goal_async','write_text','write_bytes'} & names
    assert 'Time(nanoseconds=stamp)' in source
    assert "len(job['reads'])<12" in source and "elapsed>.7" in source
    assert 'self.fit_budget=min(.2' in source and 'max(30.' in source
    assert 'owner_bracket' in source and "post['raw']!=binding['raw']" in source
    assert "configure_ros_domain(os.environ)" in source
    guard=(Path(__file__).resolve().parents[1]/'ros2/src/so101_station_perception/so101_station_perception/derived_lineage.py').read_text()
    assert "environ.get('ROS_DOMAIN_ID','76')" in guard


def test_malformed_model_xml_is_rejected_as_unavailable():
    with pytest.raises(ValueError,match='invalid model XML'):
        MeshSamples('<robot>',lambda name:'',samples=100)


def test_exact_base_seed_pin_and_candidate_ground_violation():
    from workshop.digital_twin.registration_monitor import validate_base_pin,ground_metrics
    seed=np.eye(4);seed[2,3]=.02
    assert validate_base_pin(seed,seed)==pytest.approx(seed)
    shifted=seed.copy();shifted[0,3]=1e-7
    with pytest.raises(ValueError):validate_base_pin(shifted,seed)
    candidate=seed.copy();candidate[2,3]+=.003
    result=ground_metrics(seed,candidate,cad_floor_z=-.02)
    assert result['current']['cad_floor_origin_height_m']==0.
    assert result['candidate_ground_violation'] is True
    assert result['qualified'] is False


def test_owner_eligibility_rejects_failed_hold_unknown_or_active_component_and_non_normal():
    from workshop.digital_twin.registration_monitor import owner_eligibility
    raw=dict.fromkeys(StationaryHistory.JOINTS,2000)
    state={'raw':raw,'monotonic':10.,'torque':dict.fromkeys(raw,1),'violations':[],
           'arm_bounds':{'classification':'normal','admitted_min':dict.fromkeys(raw,1000),
                         'admitted_max':dict.fromkeys(raw,3000)}}
    assert owner_eligibility(state) is None
    completed=state|{'component_command':{'status':'complete','completed_monotonic':9.}}
    assert owner_eligibility(completed) is None
    for changed,match in [({'failed_endpoint_hold':{'target_reached':False}},'failed endpoint hold'),
                          ({'component_command':{'status':'moving'}},'component'),
                          ({'component_command':{'status':'failed_held'}},'component'),
                          ({'component_command':{}},'component'),
                          ({'component_restore_pending':True},'component'),
                          ({'arm_bounds':{'classification':'recoverable'}},'normal'),
                          ({'violations':['out_of_range']},'violations'),
                          ({'torque':{}},'torque'),
                          ({'raw':raw|{'elbow_flex':3001}},'admitted')]:
        assert match in owner_eligibility(state|changed)


def test_marker_binding_requires_real_link_and_integer_identity_not_bool():
    mount={'schema':'so101-link-marker-v1','model_sha256':'model','link':'tool',
           'dictionary':'DICT_APRILTAG_25h9','marker_id':1,'link_from_marker':np.eye(4).tolist()}
    observed={'association':'unique_id','detection_count':1,'frame_id':'optical',
              'dictionary':mount['dictionary'],'marker_id':1,'stamp':100.,'camera_from_marker':np.eye(4).tolist()}
    for bad_mount,bad_observation in ((mount|{'link':''},observed),(mount|{'marker_id':True},observed),
                                      (mount,observed|{'detection_count':True}),
                                      (mount,observed|{'marker_id':True})):
        assert marker_residual(bad_mount,bad_observation,np.eye(4),100.,'model',camera_frame='optical')['state']=='unavailable_invalid_binding'
