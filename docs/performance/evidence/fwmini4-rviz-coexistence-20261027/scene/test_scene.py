"""CPU-only contract checks; no RCL node, X connection, USB or GPU context."""
from pathlib import Path
import ast
import importlib.util
import json
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parent


def main():
    config = yaml.safe_load((ROOT / 'rviz-coexistence.rviz').read_text())
    manager = config['Visualization Manager']
    assert config['Window Geometry']['Width'] == 1600
    assert config['Window Geometry']['Height'] == 900
    assert manager['Global Options']['Frame Rate'] == 30
    assert manager['Global Options']['Fixed Frame'] == 'charuco_board'
    classes = {display['Class'] for display in manager['Displays']}
    assert classes == {'rviz_default_plugins/' + name for name in ('Grid', 'TF', 'RobotModel', 'PointCloud2', 'Image', 'MarkerArray')}
    assert all(panel['Class'].startswith('rviz_common/') for panel in config['Panels'])
    image = next(d for d in manager['Displays'] if d['Class'].endswith('/Image'))
    assert image['Topic']['Value'] == '/station/resident/frame'
    assert image['Topic']['Reliability Policy'] == 'Best Effort'
    assert image['Enabled']
    params = yaml.safe_load((ROOT / 'rsp.yaml').read_text())['/**']['ros__parameters']
    robot = ET.fromstring(params['robot_description'])
    assert robot.find('ros2_control') is None
    for mesh in robot.findall('.//mesh'):
        assert mesh.attrib['filename'].startswith('file://')
        assert Path(mesh.attrib['filename'][7:]).is_file()
    source = (ROOT / 'visual_scene.py').read_text()
    parsed = ast.parse(source)
    hot_names = {'update_joint', 'update_world', 'on_joint', 'on_world'}
    forbidden = {'array', 'asarray', 'zeros', 'ones', 'empty', 'linspace', 'meshgrid', 'tobytes', 'resize', 'append', 'extend', 'create_cloud', 'PointCloud2', 'Marker', 'MarkerArray', 'JointState', 'TransformStamped', 'Image'}
    for method in (n for n in ast.walk(parsed) if isinstance(n, ast.FunctionDef) and n.name in hot_names):
        assert not any(isinstance(n, (ast.ListComp, ast.SetComp, ast.DictComp, ast.GeneratorExp)) for n in ast.walk(method)), method.name
        for call in (n for n in ast.walk(method) if isinstance(n, ast.Call)):
            name = getattr(call.func, 'attr', getattr(call.func, 'id', ''))
            assert name not in forbidden, (method.name, name)
    assert 'create_subscription' not in source
    assert 'sensor_msgs.msg import Image' not in source
    spec = importlib.util.spec_from_file_location('scene', ROOT / 'visual_scene.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    scene = module.SceneMessages()
    assert scene.cloud.width == 20000 and scene.cloud.height == 1
    assert scene.cloud.point_step == 16 and len(scene.cloud.data) == 320000
    assert scene.cloud.header.frame_id == 'charuco_board'
    assert len(scene.joints.name) == len(scene.joints.position) == 6
    moving = {j.attrib['name']: j for j in robot.findall('joint') if j.attrib['type'] != 'fixed' and j.find('mimic') is None}
    assert set(scene.joints.name) == set(moving)
    for name, position in zip(scene.joints.name, scene.joints.position):
        limits = moving[name].find('limit')
        assert float(limits.attrib['lower']) <= position <= float(limits.attrib['upper'])
    assert scene.board_to_base.header.frame_id == 'charuco_board'
    assert scene.board_to_base.child_frame_id == 'base_link'
    assert all(m.header.frame_id == 'charuco_board' for m in scene.markers.markers)
    assert len(scene.markers.markers) == 4
    assert scene.markers.markers[-1].text.startswith('SYNTHETIC')
    ids = (id(scene.cloud), id(scene.cloud.data), id(scene.joints), id(scene.joints.position), id(scene.markers), tuple(id(m) for m in scene.markers.markers))
    payload = bytes(scene.cloud.data)
    for i in range(1000):
        scene.update_joint(1000000000 + i * 33333333, i)
        if i % 3 == 0:
            scene.update_world(1000000000 + i * 33333333)
    assert ids == (id(scene.cloud), id(scene.cloud.data), id(scene.joints), id(scene.joints.position), id(scene.markers), tuple(id(m) for m in scene.markers.markers))
    assert bytes(scene.cloud.data) == payload
    print(json.dumps({'passed': True, 'point_count': 20000, 'payload_bytes': 320000, 'hot_payload_identity_stable': True, 'camera_rgb_source': '/station/resident/frame', 'gpu_or_usb': False}))


if __name__ == '__main__':
    main()
