"""Native reset metadata proofs: no ROS installation, pixels or device access."""
import copy
import json
from types import SimpleNamespace as S

import pytest
from test_station_emergency_reset import NS


def proof_type():
    assert 'NativeResetProof' in NS, 'explicit native proof implementation is missing'
    return NS['NativeResetProof']


def header(ns=10_100_000_000):
    return S(stamp=S(sec=ns//10**9, nanosec=ns%10**9), frame_id='optical')


def image(ns, depth=False):
    return S(header=header(ns), width=2, height=2, encoding='16UC1' if depth else 'rgb8',
             step=4 if depth else 6, data=b'0'*(8 if depth else 12))


def info(ns):
    return S(header=header(ns), width=2, height=2, k=[1.,0.,1.,0.,1.,1.,0.,0.,1.])


def gid(i):
    return bytes([i])*16


KEYS = ('rgb_image','depth_image','rgb','depth','tags','objects','objects_status','scene_status','joint_states','movegroup')


def make():
    return proof_type()({key: gid(i+1) for i,key in enumerate(KEYS)}, 10_000_000_000, 5., 'vulkan', .04)


def publish(p, ns, now=5.2):
    msgs = {'rgb_image':image(ns), 'depth_image':image(ns, True), 'rgb':info(ns), 'depth':info(ns),
            'tags':S(data=json.dumps(dict(schema='so101-tag-detections-v1', stamp=dict(sec=ns//10**9,nanosec=ns%10**9), observations=[]))),
            'objects':S(header=S(stamp=header(ns).stamp,frame_id='charuco_board'), detections=[]),
            'objects_status':S(data=json.dumps(dict(schema=1,state='tracking',stamp=ns/1e9,frame_id='charuco_board',color_mask_backend={'selected':'vulkan'}))),
            'scene_status':S(data=json.dumps(dict(state='empty',stamp=ns/1e9,accepted=0,rejected=0,instance_id='b'*32,scene_busy=False,pending_count=0,scene_fault=False,live_ids=[],applied_world_ids=[]))),
            'joint_states':S(header=header(ns),name=sorted(NS['JOINTS']),position=[0.]*6)}
    for key,msg in msgs.items():
        p.add(key,msg,p.gids[key],now,ns+100_000_000)
    return msgs


def ready(p, now=5.3, ns=10_300_000_000):
    return p.ready(now,ns)


def test_two_joined_advancing_cohorts_are_required():
    p=make(); publish(p,10_100_000_000)
    assert not ready(p)
    publish(p,10_200_000_000)
    assert ready(p)
    assert all(len(v)<=128 for v in p.samples.values())


@pytest.mark.parametrize('fault',['old_gid','old_source','future','repeat','short_data','stride','encoding','bad_info','unjoined','unavailable','backend','busy_scene','bad_joints'])
def test_invalid_cohort_does_not_establish_readiness(fault):
    p=make()
    for ns in (10_100_000_000,10_200_000_000):
        msgs=publish(p,ns)
        key='rgb_image'; msg=msgs[key]; identity=p.gids[key]
        # Remove the successful samples, then inject the faulty replacements.
        if fault=='old_gid': identity=gid(99)
        elif fault=='old_source': msg.header=header(9_999_999_999)
        elif fault=='future': msg.header=header(11_000_000_000)
        elif fault=='repeat': msg.header=header(10_100_000_000)
        elif fault=='short_data': msg.data=b'0'
        elif fault=='stride': msg.step=1
        elif fault=='encoding': msg.encoding='not-an-image'
        elif fault=='bad_info': key='rgb'; msg=msgs[key]; msg.k[0]=float('nan')
        elif fault=='unjoined': key='tags'; msg=msgs[key]; d=json.loads(msg.data); d['stamp']['nanosec']+=5_000_000; msg.data=json.dumps(d)
        elif fault in ('unavailable','backend'):
            key='objects_status'; msg=msgs[key]; d=json.loads(msg.data)
            if fault=='unavailable': d['state']='unavailable'
            else: d['color_mask_backend']={'selected':'cpu'}
            msg.data=json.dumps(d)
        elif fault=='busy_scene': key='scene_status'; msg=msgs[key]; d=json.loads(msg.data); d['scene_busy']=True; msg.data=json.dumps(d)
        elif fault=='bad_joints': key='joint_states'; msg=msgs[key]; msg.position=[float('nan')]*6
        p.samples[key]=[]
        p.add(key,msg,identity if key=='rgb_image' else p.gids[key],5.2,ns+100_000_000)
    assert not ready(p),fault


def test_receipt_and_source_leases_expire_without_renewal():
    p=make(); publish(p,10_100_000_000); publish(p,10_200_000_000)
    assert ready(p)
    assert not ready(p,now=6.3)
    assert not ready(p,ns=11_300_000_000)


def test_graph_exact_replacement_absent_before_and_unchanged_owner():
    cls=proof_type(); after={k:gid(i+1) for i,k in enumerate(KEYS)}
    before=dict(after); before['rgb_image']=None; before['tags']=gid(90)
    cls.check_graph(before,after,{'rgb_image','tags'})
    with pytest.raises(RuntimeError): cls.check_graph(after,after,{'tags'})
    changed=dict(after,joint_states=gid(88))
    with pytest.raises(RuntimeError): cls.check_graph(before,changed,{'rgb_image','tags'})


def test_metadata_only_does_not_iterate_or_copy_pixels():
    p=make()
    class Pixels:
        def __len__(self):return 12
        def __iter__(self):raise AssertionError('pixel scan')
        def __bytes__(self):raise AssertionError('pixel copy')
    msg=image(10_100_000_000);msg.data=Pixels()
    p.add('rgb_image',msg,p.gids['rgb_image'],5.2,10_200_000_000)
    assert len(p.samples['rgb_image'])==1


def test_native_graph_refuses_duplicate_wrong_node_and_old_invocation():
    from test_station_emergency_reset import API,harness
    api=harness(); api.recovery_profile='native_ros'
    api.native_topics={'rgb_image':'/rgb'};api.native_nodes={'rgb_image':'/camera'}
    api.get_publishers_info_by_topic=lambda topic:[S(node_namespace='/',node_name='camera',endpoint_gid=gid(1))]
    assert api.native_graph()=={'rgb_image':gid(1)}
    api.get_publishers_info_by_topic=lambda topic:[]
    assert api.native_graph(allow_missing=True)=={'rgb_image':None}
    with pytest.raises(RuntimeError):api.native_graph()
    for endpoints in ([S(node_namespace='/',node_name='other',endpoint_gid=gid(1))], [S(),S()]):
        api.get_publishers_info_by_topic=lambda topic:endpoints
        with pytest.raises(RuntimeError):api.native_graph()


def test_native_profile_capability_has_no_rocm_dependency():
    from test_station_emergency_reset import harness
    api=harness();api.recovery_profile='native_ros'
    api.service_clients={name:S(service_is_ready=lambda:True) for name in ('status','stop','reconnect','reset_owner','maintenance','rearm','home','teleop','teleop_mode','damping')}
    api.service_clients.update({name:S(service_is_ready=lambda:False) for name in ('recovery_start','recovery_status')})
    assert api.capabilities(api.state)['emergency_reset']['enabled']
    api.recovery_profile='derived_rocm'
    assert not api.capabilities(api.state)['emergency_reset']['enabled']


def test_launch_profile_is_explicit_and_rocm_defaults_remain_unchanged():
    from test_station_emergency_reset import ROOT
    launch=(ROOT/'ros2/src/so101_station_operator/launch/operator.launch.py').read_text()
    station=(ROOT/'ros2/src/so101_station_bringup/launch/station_live.launch.py').read_text()
    for key in ('recovery_profile','native_publisher_nodes_json','native_mask_backend','native_pair_delta_s',
                'rgb_image_topic','depth_image_topic','rgb_topic','depth_topic','objects_status_topic','movegroup_scene_topic'):
        assert key in launch
        assert 'operator_'+key in station
    assert '"recovery_profile": "derived_rocm"' in launch
    assert "'operator_recovery_profile': 'derived_rocm'" in station


def test_final_check_rechecks_freshness_and_identity_after_owner_rpc():
    from test_station_emergency_reset import harness
    api=harness();api.recovery_profile='native_ros';api.native_stop_generation=0
    api.native_owner_identity=api.reset_owner_proof(api.state)
    p=make();publish(p,10_100_000_000);publish(p,10_200_000_000);api.native_proof=p
    api.native_graph=lambda:dict(p.gids)
    api.source_admission_phase=lambda *args:None
    api.maintenance_state=lambda token:dict(api.state,active_operation='operator_maintenance',operator_maintenance_token=token)
    api.get_clock=lambda:S(now=lambda:S(nanoseconds=10_300_000_000))
    from unittest.mock import patch
    with patch.object(NS['time'],'monotonic',return_value=5.3):
        api.native_final_check('a'*32,6.)
        api.native_graph=lambda:dict(p.gids,tags=gid(88))
        with pytest.raises(RuntimeError,match='identity changed'):api.native_final_check('a'*32,6.)
    api.native_graph=lambda:dict(p.gids)
    with patch.object(NS['time'],'monotonic',return_value=6.3):
        with pytest.raises(RuntimeError,match='not currently fresh'):api.native_final_check('a'*32,7.)


@pytest.mark.parametrize('key,data',[('objects_status',dict(schema=1,state='unavailable',stamp=10.25)),('scene_status',dict(state='fault',stamp=10.25,scene_fault=True))])
def test_negative_status_revokes_previous_success_immediately(key,data):
    p=make();publish(p,10_100_000_000);publish(p,10_200_000_000);assert ready(p)
    p.add(key,S(data=json.dumps(data)),p.gids[key],5.3,10_300_000_000)
    assert not ready(p)
    publish(p,10_300_000_000,now=5.4)
    assert ready(p,now=5.5,ns=10_400_000_000)


def test_rejected_nonempty_objects_cannot_qualify_as_empty_scene():
    p=make()
    for ns in (10_100_000_000,10_200_000_000):
        msgs=publish(p,ns)
        p.samples['objects'].pop()
        msg=msgs['objects'];msg.detections=[S(id='red-0001')]
        p.add('objects',msg,p.gids['objects'],5.2,ns+100_000_000)
    assert not ready(p)


def test_final_check_does_not_accept_stop_during_owner_read():
    from test_station_emergency_reset import harness
    api=harness();api.recovery_profile='native_ros';api.native_stop_generation=0
    api.native_owner_identity=api.reset_owner_proof(api.state)
    p=make();publish(p,10_100_000_000);publish(p,10_200_000_000);api.native_proof=p
    api.native_graph=lambda:dict(p.gids)
    def owner_rpc(token):
        api.stop_generation+=1
        return dict(api.state,active_operation='operator_maintenance',operator_maintenance_token=token)
    api.maintenance_state=owner_rpc
    api.get_clock=lambda:S(now=lambda:S(nanoseconds=10_300_000_000))
    from unittest.mock import patch
    with patch.object(NS['time'],'monotonic',return_value=5.3):
        with pytest.raises(RuntimeError,match='STOP'):api.native_final_check('a'*32,6.)


def test_wrong_gid_never_updates_shared_measured_state():
    from test_station_emergency_reset import harness
    api=harness();api.native_proof=make()
    api.get_clock=lambda:S(now=lambda:S(nanoseconds=10_300_000_000))
    calls=[];api.reset_stream=lambda *args:calls.append(args)
    msg=S(header=header(),name=sorted(NS['JOINTS']),position=[0.]*6)
    api.native_callback('joint_states',msg,S(publisher_gid=gid(90)))
    assert calls==[]


def test_new_rejected_or_mismatched_scene_never_falls_back_to_old_success():
    for rejected in (0,1):
        p=make();publish(p,10_100_000_000);publish(p,10_200_000_000);assert ready(p)
        ns=10_300_000_000;msgs=publish(p,ns,now=5.4)
        p.samples['objects'].pop();msgs['objects'].detections=[S(id='red-0001')]
        p.add('objects',msgs['objects'],p.gids['objects'],5.4,10_400_000_000)
        p.samples['scene_status'].pop();data=json.loads(msgs['scene_status'].data);data['rejected']=rejected
        p.add('scene_status',S(data=json.dumps(data)),p.gids['scene_status'],5.4,10_400_000_000)
        assert not ready(p,now=5.5,ns=10_400_000_000)


def test_jazzy_dict_message_info_and_zero_padded_gid_are_supported():
    from test_station_emergency_reset import harness
    api=harness();api.native_proof=make();api.get_clock=lambda:S(now=lambda:S(nanoseconds=10_300_000_000))
    calls=[];api.reset_stream=lambda *args:calls.append(args)
    msg=S(header=header(),name=sorted(NS['JOINTS']),position=[0.]*6)
    api.native_callback('joint_states',msg,{'publisher_gid':api.native_proof.gids['joint_states']+bytes(8)})
    assert len(calls)==1
    api.native_callback('joint_states',msg,{})
    assert len(calls)==1


def test_native_missing_owner_identity_fails_before_reservation():
    from test_station_emergency_reset import API,harness
    api=harness();api.recovery_profile='native_ros';api.native_graph=lambda **kw:{}
    api.state.pop('calibration_sha256');api.reset_units['leader_readonly']=''
    calls=[];api.rpc=lambda *args,**kw:calls.append(args) or json.dumps({'token':'a'*32})
    with pytest.raises(RuntimeError,match='identity'):
        API.restart(api,'camera')
    assert calls==[]


def test_joint_state_frame_can_be_empty_like_resident_bridge():
    p=make();msg=S(header=header(),name=sorted(NS['JOINTS']),position=[0.]*6)
    msg.header.frame_id=''
    assert p.add('joint_states',msg,p.gids['joint_states'],5.2,10_300_000_000)
