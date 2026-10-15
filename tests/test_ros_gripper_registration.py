import tempfile
import unittest
from pathlib import Path

import numpy as np

from tools.fit_ros_robot_registration import forward_kinematics, parse_urdf
from tools.fit_ros_gripper_registration import rotate_tool_description, validate_snapshot_diversity


URDF = '''<robot name="test">
<link name="base"/><link name="wrist"/><link name="gripper_link"/><link name="jaw"/>
<joint name="wrist_roll" type="revolute"><parent link="wrist"/><child link="gripper_link"/>
<origin xyz="0.1 0.2 0.3" rpy="0.2 -0.1 0.3"/><axis xyz="0 0 1"/></joint>
<joint name="fixed" type="fixed"><parent link="base"/><child link="wrist"/><origin xyz="0.4 0 0"/></joint>
<joint name="jaw_joint" type="revolute"><parent link="gripper_link"/><child link="jaw"/>
<origin xyz="0.02 0 0"/><axis xyz="0 1 0"/></joint></robot>'''


class ToolRollTests(unittest.TestCase):
    def _transforms(self, text):
        links, joints = parse_urdf(text)
        return forward_kinematics(links, joints, {"wrist_roll": .25, "jaw_joint": .1})

    def test_roll_only_changes_tool_subtree_and_matches_local_rotation(self):
        base = self._transforms(URDF)
        candidate_text = rotate_tool_description(Path(self._write_urdf()), 180)
        candidate = self._transforms(candidate_text)
        np.testing.assert_allclose(candidate["base"], base["base"], atol=1e-12)
        np.testing.assert_allclose(candidate["wrist"], base["wrist"], atol=1e-12)
        self.assertFalse(np.allclose(candidate["gripper_link"], base["gripper_link"]))
        self.assertFalse(np.allclose(candidate["jaw"], base["jaw"]))
        self.assertTrue(np.allclose(candidate["gripper_link"][:3, 3], base["gripper_link"][:3, 3], atol=1e-12))

    def _write_urdf(self):
        handle = tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False)
        handle.write(URDF)
        handle.close()
        self.addCleanup(Path(handle.name).unlink, missing_ok=True)
        return handle.name

    def test_rejects_wrong_tool_parent(self):
        bad = URDF.replace('child link="gripper_link"', 'child link="wrist"', 1)
        path = self._write_urdf().replace("test", "bad")
        Path(path).write_text(bad)
        with self.assertRaisesRegex(ValueError, "wrist_roll -> gripper_link"):
            rotate_tool_description(path, 90)

    def test_requires_diverse_train_poses_and_distinct_holdout(self):
        names = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
        frames = []
        for i, (pan, lift, elbow) in enumerate(((0, 0, 0), (.1, 0, 0), (0, .1, 0), (.05, .05, .05))):
            frames.append({"q": dict(zip(names, (pan, lift, elbow, .02*i, .03*i)))})
        self.assertTrue(validate_snapshot_diversity(frames, 3)["heldout_pose_distinct"])
        repeated = [{"q": dict(zip(names, [0]*5))} for _ in range(4)]
        with self.assertRaisesRegex(ValueError, "independent movement"):
            validate_snapshot_diversity(repeated, 3)


if __name__ == "__main__":
    unittest.main()
