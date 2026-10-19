"""Derived diagnostics are explicit local provenance, never direct camera proof."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'ros2/src/so101_station_perception/so101_station_perception/derived_lineage.py'


def api():
    assert SOURCE.exists(),'derived lineage validation is not implemented'
    spec=importlib.util.spec_from_file_location('derived_lineage',SOURCE)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


def fixture():
    names=('depth','color','depth_info','color_info');gids={name:('%032x'%(i+1)) for i,name in enumerate(names)}
    snapshot={'node':'/actual_camera','endpoints':{name:{'node':'/actual_camera','gid':gids[name]} for name in names}}
    generation=hashlib.sha256(json.dumps(snapshot,sort_keys=True).encode()).hexdigest()
    value=dict(schema='so101-derived-rocm-rgbd-cohort-v1',qualified=False,execution_enabled=False,direct_camera_attestation=False,
        producer_generation='a'*32,sequence=1,source_generation=generation,
        original=dict(depth_stamp_ns=10_000_000_000,color_stamp_ns=10_001_000_000,depth_info_stamp_ns=10_000_000_000,
                      color_info_stamp_ns=10_001_000_000,depth_frame='depth',color_frame='color',pair_delta_ns=1_000_000),
        source=dict(camera_node='/actual_camera',serial_number='actual_serial',depth_topic='/camera/depth/image_rect_raw',
            color_topic='/camera/color/image_raw',**{name+'_publisher_gid':gid for name,gid in gids.items()}),
        identity=dict(source_generation=generation,serial_number='actual_serial',request_started_monotonic=100.,
            confirmed_monotonic=100.01,confirmed_ros_ns=10_000_000_000,expires_monotonic=101.,device_info_request_source=snapshot),
        outputs=dict(cloud_topic='/so101_perception/rocm/points',status_topic='/so101_perception/rocm/lineage',
            cloud_publisher_gid='5'*32,status_publisher_gid='6'*32,aligned_publisher_gid='7'*32,aligned_topic='/so101_perception/rocm/aligned_depth',cloud_stamp_ns=10_000_000_000,cloud_frame='depth',
            aligned_stamp_ns=10_000_000_000,aligned_frame='color'),
        calibration=dict(sha256='b'*64,depth_size=[320,240],color_size=[640,480],depth_K=[200.,200.,160.,120.],
            color_K=[400.,400.,320.,240.],depth_D=[0.]*5,color_D=[0.]*5,rotation_column_major=[1.,0.,0.,0.,1.,0.,0.,0.,1.],
            translation_m=[.015,0.,0.],depth_scale_m=.001,depth_scale_source='existingROS16UC1millimetercontract; notindependentphysicalmeasurement'),
        implementation=dict(source_sha256={n:'c'*64 for n in ('rocm_transport.py','rocm_client.py','rocm_worker.py','rocm_projection.py','rocm_projection_backend.py','rocm_projection_kernels.py')},
            producer_node_sha256='f'*64,worker_python='/actual/python',worker_python_sha256='d'*64,compatibility_sha256='e'*64,hardware={'name':'actual GPU'}),
        health=dict(state='valid',source_receipts_monotonic=dict.fromkeys(names,100.1),source_expiry_monotonic=100.35,
            original_max_age_ns=500_000_000,observation_ros_ns=10_100_000_000),metrics=dict(points=2,cloud_bytes=40,backend_ms=8.,worker_ms=10.))
    return value


def validate(value=None,**changes):
    options=dict(now_ns=10_120_000_000,now_mono=100.12,receipt_mono=100.11);options.update(changes)
    return api().validate_statement(value or fixture(),**options)


def test_valid_statement_is_unqualified_with_integer_exact_stamps():
    value=fixture();pin=validate(value)
    assert pin['source_generation']==value['source_generation']
    assert 'sequence' not in pin and 'original' not in pin
    assert pin['calibration']['sha256']=='b'*64


@pytest.mark.parametrize('path,value',[(('sequence',),True),(('original','depth_stamp_ns'),10e9),
    (('outputs','cloud_stamp_ns'),10_000_000_001),(('health','state'),'ready'),(('qualified',),True),
    (('health','original_max_age_ns'),600_000_000),(('identity','expires_monotonic'),102.),
    (('source','depth_publisher_gid'),'0'*32),(('source_generation',),'0'*64)])
def test_invalid_metadata_is_rejected(path,value):
    item=fixture();target=item
    for key in path[:-1]:target=target[key]
    target[path[-1]]=value
    with pytest.raises(ValueError):validate(item)


@pytest.mark.parametrize('changes',[dict(now_ns=9_999_999_999),dict(now_ns=10_600_000_001),
    dict(now_mono=100.36),dict(now_mono=101.01),dict(receipt_mono=101.)])
def test_no_future_old_original_or_expired_source_identity(changes):
    with pytest.raises(ValueError):validate(**changes)


def test_buffer_bounded_exact_ns_pair_no_replay_renewal_and_generation_latch():
    m=api();value=fixture();buffer=m.LineageBuffer()
    buffer.add(json.dumps(value),bytes.fromhex('6'*32),now_ns=10_120_000_000,now_mono=100.12)
    assert buffer.match(10_000_000_000,now_ns=10_120_000_000,now_mono=100.12)[0]['sequence']==1
    with pytest.raises(ValueError):buffer.match(10_000_000_001,now_ns=10_120_000_000,now_mono=100.12)
    # Replayed status cannot renew its first receipt clock.
    buffer.add(json.dumps(value),bytes.fromhex('6'*32),now_ns=10_120_000_000,now_mono=100.2)
    assert buffer.entries[-1][2]==100.12
    value['producer_generation']='f'*32
    with pytest.raises(ValueError):buffer.add(json.dumps(value),bytes.fromhex('6'*32),now_ns=10_120_000_000,now_mono=100.12)
    assert buffer.invalid_reason and not buffer.entries
    with pytest.raises(ValueError):buffer.match(10_000_000_000,now_ns=10_120_000_000,now_mono=100.12)


def endpoints(value):
    m=api();records={}
    for stream,topic in m.TOPICS.items():
        records[topic]=[NS(node_namespace='/',node_name='actual_camera',endpoint_gid=bytes.fromhex(value['source'][stream+'_publisher_gid']))]
    for stream in ('cloud','status','aligned'):
        records[value['outputs'][stream+'_topic']]=[NS(node_namespace='/',node_name='derived',endpoint_gid=bytes.fromhex(value['outputs'][stream+'_publisher_gid']))]
    return records


@pytest.mark.parametrize('index',range(7))
def test_every_image_info_cloud_lineage_endpoint_generation_is_bound(index):
    value=fixture();graph=endpoints(value);m=api()
    assert m.validate_graph(value,graph.__getitem__) == value['identity']['device_info_request_source']
    topic=list(graph)[index];old=graph[topic][0]
    graph[topic]=[NS(node_namespace=old.node_namespace,node_name=old.node_name,endpoint_gid=b'\xff'*16)]
    with pytest.raises(ValueError):m.validate_graph(value,graph.__getitem__)


def test_cloud_and_actual_camera_metadata_exactness():
    m=api();value=fixture()
    def info(kind):
        c=value['calibration'];o=value['original'];fx,fy,cx,cy=c[kind+'_K'];ns=o[kind+'_info_stamp_ns']
        return NS(header=NS(stamp=NS(sec=ns//10**9,nanosec=ns%10**9),frame_id=kind),width=c[kind+'_size'][0],height=c[kind+'_size'][1],
            k=[fx,0.,cx,0.,fy,cy,0.,0.,1.],d=[0.]*5)
    cloud=NS(header=NS(stamp=NS(sec=10,nanosec=0),frame_id='depth'),width=2,height=1,point_step=20,row_step=40,data=b'\0'*40,is_bigendian=False,
        fields=[NS(name=n,offset=o,datatype=7,count=1) for n,o in [('x',0),('y',4),('z',8),('rgb',16)]])
    m.validate_cloud(value,cloud,bytes.fromhex('5'*32))
    pose=[[1.,0.,0.,.015],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]]
    depth,color=info('depth'),info('color');m.validate_camera(value,depth,color,pose)
    cloud.header.stamp.nanosec=1
    with pytest.raises(ValueError):m.validate_cloud(value,cloud,bytes.fromhex('5'*32))
    depth.k[0]+=.01
    with pytest.raises(ValueError):m.validate_camera(value,depth,color,pose)
    depth=info('depth');pose[0][3]+=.001
    with pytest.raises(ValueError):m.validate_camera(value,depth,color,pose)


def test_status_size_finite_json_duplicates_buffer_cap_and_missing_gid_latch():
    m=api()
    for raw in ('{"x":1,"x":2}','{"x":NaN}',' '*65537,'[]'):
        with pytest.raises(ValueError):m.parse_lineage(raw)
    b=m.LineageBuffer()
    for i in range(12):
        v=fixture();v['sequence']=i+1
        for key in ('depth_stamp_ns','color_stamp_ns','depth_info_stamp_ns','color_info_stamp_ns'):v['original'][key]+=i
        for key in ('cloud_stamp_ns','aligned_stamp_ns'):v['outputs'][key]+=i
        b.add(json.dumps(v),bytes.fromhex('6'*32),now_ns=10_120_000_000,now_mono=100.12)
    assert len(b.entries)==8 and b.entries[0][0]['sequence']==5
    with pytest.raises(ValueError):b.observe_gid('depth_info',b'')
    assert b.invalid_reason and not b.entries


def test_source_callback_hot_path_has_no_file_device_or_gpu_work():
    import ast
    tree=ast.parse(SOURCE.read_text())
    forbidden={'open','read_bytes','read_text','create_client','create_timer','send_goal_async','set_parameters','sendTransform','item','synchronize'}
    called={n.func.id if isinstance(n.func,ast.Name) else n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,(ast.Name,ast.Attribute))}
    assert not called&forbidden
    imports={n.names[0].name for n in ast.walk(tree) if isinstance(n,ast.Import)}
    assert not imports&{'torch','triton','rclpy','pyrealsense2','numpy'}


def test_opt_in_defaults_false_and_no_direct_camera_forgery_or_writes():
    import ast
    package=SOURCE.parent
    helper=(package/'registration_source.py').read_text();monitor=(package/'registration_monitor.py').read_text()
    launch=(package.parent/'launch/periodic_registration.launch.py').read_text()
    assert "'--allow-derived-source',action='store_true',default=False" in helper
    assert "'allow_derived_source':False" in monitor and "'allow_derived_source':('false',bool)" in launch
    assert "if not getattr(self,'allow_derived',False):raise ValueError" in monitor
    assert "candidates=tuple(self.clouds)[-1:]" in helper and "candidates=tuple(self.clouds)[-1:]" in monitor
    for text in (helper,monitor):
        tree=ast.parse(text);called={n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)}
        assert not called&{'sendTransform','send_goal_async','set_parameters','create_action_client'}
    tree=ast.parse(helper)
    clients=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='create_client']
    assert len(clients)==1 and isinstance(clients[0].args[0],ast.Name) and clients[0].args[0].id=='DeviceInfo'


def test_domain_normalization_rejects_all_zero_spellings_and_invalid_range():
    module=api();assert hasattr(module,'configure_ros_domain')
    guard=module.configure_ros_domain;env={};guard(env)
    assert env=={'ROS_DOMAIN_ID':'76','ROS_LOCALHOST_ONLY':'1'}
    for raw in ('0','00','+0','-0','-1','233','nan','76.0',''):
        with pytest.raises(ValueError):guard({'ROS_DOMAIN_ID':raw})
    env={'ROS_DOMAIN_ID':'+177'};guard(env);assert env['ROS_DOMAIN_ID']=='177'
    import ast
    for path in (SOURCE.parent/'registration_source.py',SOURCE.parent/'registration_monitor.py',SOURCE.parent.parent/'launch/periodic_registration.launch.py'):
        tree=ast.parse(path.read_text())
        assert any(isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='configure_ros_domain' for n in ast.walk(tree)),str(path)


@pytest.mark.parametrize('field,value', [('producer_node_sha256',None),('producer_node_sha256','bad')])
def test_producer_node_code_hash_is_required(field,value):
    item=fixture()
    if value is None:del item['implementation'][field]
    else:item['implementation'][field]=value
    with pytest.raises(ValueError):validate(item)


def test_aligned_output_identity_is_required_and_pinned():
    item=fixture();del item['outputs']['aligned_publisher_gid']
    with pytest.raises(ValueError):validate(item)


def test_actual_native_cpu_emitted_aligned_contract_fixture():
    import hashlib
    path=Path(__file__).parent/'fixtures/derived-lineage-native-cpu-v1.json'
    assert hashlib.sha256(path.read_bytes()).hexdigest()=='b727c55c808f1b3219c5027ac8fcafe8fba957b4455899d794be46e7abd0d2fa'
    envelope=json.loads(path.read_text());value=envelope['lineage']
    assert envelope['physical_camera_used'] is False and envelope['gpu_executed'] is False
    now_ns=value['health']['observation_ros_ns'];now_mono=max(value['health']['source_receipts_monotonic'].values())+.02
    pin=api().validate_statement(value,now_ns=now_ns,now_mono=now_mono,receipt_mono=now_mono)
    assert pin['outputs']['aligned_publisher_gid']==envelope['observed_aligned_publisher_gid']
