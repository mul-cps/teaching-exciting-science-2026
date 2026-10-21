"""Native CameraInfo and receiver path; no node, executor, ROS init or devices."""
import ast
from types import SimpleNamespace as NS
import pytest
pytest.importorskip('rclpy')
from sensor_msgs.msg import CameraInfo
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from test_early_camera_info import setup, message, snapshot
from test_derived_source_binding_health import SOURCE


@pytest.mark.parametrize('kind',['depth_info','color_info'])
@pytest.mark.parametrize('gid_type',['dict','message_info'])
def test_native_info_receiver_discards_only_early_valid_pinned_input(kind,gid_type):
    obj,value,clocks,scope,offers=setup();fake=message(value,clocks,kind)
    native=CameraInfo(header=Header(stamp=Time(sec=fake.header.stamp.sec,nanosec=fake.header.stamp.nanosec),
        frame_id=fake.header.frame_id),height=fake.height,width=fake.width,k=fake.k,d=fake.d)
    tree=ast.parse(SOURCE.read_text())
    functions=[n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name in ('_receiver','received_gid')]
    exec(compile(ast.Module(body=functions,type_ignores=[]),str(SOURCE),'exec'),scope)
    gid=bytes.fromhex(value['source'][kind+'_publisher_gid'])
    info={'publisher_gid':gid} if gid_type=='dict' else NS(publisher_gid=gid)
    receiver=scope['_receiver'](obj,kind);before=snapshot(obj)
    receiver(native,info)
    assert snapshot(obj)==before and not offers and obj._early_info_drops[kind]==1
    obj.drain.assert_not_called()
    native.k[0]+=1
    receiver(native,info)
    assert obj.fault and offers==[None] and obj._early_info_drops[kind]==1
