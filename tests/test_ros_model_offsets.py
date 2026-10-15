import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2/src/so101_station_control"))
from so101_station_control.model_offsets import apply_gripper_mount_roll


DESCRIPTION = '''<robot name="test"><joint name="wrist_roll" type="revolute">
<origin xyz=".1 .2 .3" rpy="1.5708 .0486795 3.14159"/>
<parent link="wrist_link"/><child link="gripper_link"/><axis xyz="0 0 1"/>
</joint><joint name="gripper_frame_joint" type="fixed"><origin xyz=".01 0 -.1"/></joint>
<link name="gripper_link"><visual name="body"/><collision name="body"/></link></robot>'''


def test_offset_rotates_full_mount_about_local_axis_without_changing_geometry():
    old = ET.fromstring(DESCRIPTION)
    new = ET.fromstring(apply_gripper_mount_roll(DESCRIPTION, 90))
    a = old.find("joint[@name='wrist_roll']/origin")
    b = new.find("joint[@name='wrist_roll']/origin")
    r0 = Rotation.from_euler('xyz', list(map(float, a.get('rpy').split()))).as_matrix()
    r1 = Rotation.from_euler('xyz', list(map(float, b.get('rpy').split()))).as_matrix()
    assert np.allclose(r1, r0 @ Rotation.from_euler('z', 90, degrees=True).as_matrix())
    assert np.allclose(r0[:, 2], r1[:, 2])
    assert a.get('xyz') == b.get('xyz')
    for path in ["joint[@name='gripper_frame_joint']", "link[@name='gripper_link']"]:
        assert ET.tostring(old.find(path)) == ET.tostring(new.find(path))


def test_zero_preserves_description_and_invalid_offset_is_rejected():
    assert apply_gripper_mount_roll(DESCRIPTION, 0) == DESCRIPTION
    with pytest.raises(ValueError):
        apply_gripper_mount_roll(DESCRIPTION, float('nan'))
