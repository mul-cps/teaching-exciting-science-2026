"""Explicit official SO101 native-site / URDF-frame bridge, CPU-only.

Both frames share an origin to the official model's numerical precision, but
have different axes: native +X is URDF +Z; native +Y is URDF +Y; native +Z is
URDF -X. This fixed model convention does NOT register a real camera or robot.
"""
import numpy as np

# T_urdf_gripper_frame_link_from_native_gripper_frame_site.
NATIVE_SITE_TO_URDF_FRAME = np.array([
    [0., 0., -1., 0.],
    [0., 1., 0., 0.],
    [1., 0., 0., 0.],
    [0., 0., 0., 1.],
])
NATIVE_SITE_TO_URDF_FRAME.flags.writeable = False


def convert_native_pinch_offset(xyz):
    """Return URDF-link-from-pinch SE3 from a native-site local XYZ offset.

    The resulting pinch frame RETAINS native site axes, rotated into URDF link
    coordinates. Multiplication by URDF FK therefore reproduces native world
    pinch axes and position. This is not an arbitrary chosen grasp orientation.
    """
    xyz = np.asarray(xyz, dtype=float)
    if xyz.shape != (3,) or not np.isfinite(xyz).all():
        raise ValueError("Expected finite native-site XYZ in metres")
    transform = NATIVE_SITE_TO_URDF_FRAME.copy()
    transform[:3, 3] = transform[:3, :3] @ xyz
    return transform


def frame_provenance():
    return {
        "source_offset_frame": "native-MuJoCo-gripper_frame_site",
        "destination_offset_frame": "URDF-gripper_frame_link",
        "pinch_orientation_axes": "native-MuJoCo-gripper_frame_site",
        "native_site_to_urdf_frame": NATIVE_SITE_TO_URDF_FRAME.tolist(),
        "camera_registration": "not-provided",
        "grasp_eligible": False,
    }


class NativeCADPinchProvider:
    """Converted approximate CAD aperture midpoint for commanded 0..30% opening.

    The native gripper-angle percent mapping and mesh vertex contact estimates
    are approximate. Wider openings are intentionally rejected until checked;
    this provider certifies neither contact geometry nor grasp clearance.
    """

    def __init__(self):
        import mujoco
        from .scene import make_model
        self._mujoco = mujoco
        xml = make_model([0, 0, 0], [1, 0, 0], [0, 0, 1], [1, 0, 0, 0])
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "gripper")
        self._qpos = self.model.jnt_qposadr[joint]
        self._limits = self.model.jnt_range[joint].copy()

    def __call__(self, opening_percent):
        from .kinematics import export_pinch_geometry
        if not np.isfinite(opening_percent) or not 0 <= opening_percent <= 30:
            raise ValueError("Approximate native CAD provider qualified only for 0..30 percent opening")
        low, high = self._limits
        self.data.qpos[self._qpos] = low + opening_percent / 100 * (high - low)
        self._mujoco.mj_forward(self.model, self.data)
        geometry = export_pinch_geometry(self.model, self.data)
        return convert_native_pinch_offset(geometry["pinch_center_gripper_site_m"])

    @staticmethod
    def provenance():
        return {**frame_provenance(), "method": "approximate distal CAD face midpoint",
                "opening_percent_range": [0, 30], "qualification": "model frame bridge only; physical grasp unvalidated"}
