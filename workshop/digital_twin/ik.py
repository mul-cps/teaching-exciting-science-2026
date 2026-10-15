"""Offline SO101 IK using LeRobot/Placo; never sends hardware commands.

Coordinates are metres in the fixed URDF base frame. Calibration is LeRobot's
range-centred mapping, not a camera registration. Gripper angle is the explicitly
approximate native-model mapping (-10 + 1.1 * opening percent) degrees.
"""
from dataclasses import dataclass
from pathlib import Path
import json
import hashlib
import tempfile
import time
import xml.etree.ElementTree as ET

import numpy as np

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
JOINTS = ARM_JOINTS + ("gripper",)
ASSETS = Path(__file__).parent / "assets"


@dataclass(frozen=True)
class IKResult:
    success: bool
    raw: dict[str, int] | None
    pose: np.ndarray | None
    position_error_m: float
    orientation_error_rad: float
    iterations: int
    elapsed_s: float
    reason: str
    provenance: dict
    numerical_ik_only: bool = True
    grasp_eligible: bool = False


def _se3(value):
    value = np.asarray(value, dtype=float)
    if value.shape != (4, 4) or not np.isfinite(value).all():
        raise ValueError("Expected a finite 4x4 SE3 matrix")
    r = value[:3, :3]
    if not np.allclose(value[3], [0, 0, 0, 1], atol=1e-8) or not np.allclose(r.T @ r, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(r), 1, atol=1e-6):
        raise ValueError("Invalid SE3 rotation or homogeneous row")
    return value


def _errors(actual, target):
    pos = float(np.linalg.norm(actual[:3, 3] - target[:3, 3]))
    angle = float(np.arccos(np.clip((np.trace(actual[:3, :3].T @ target[:3, :3]) - 1) / 2, -1, 1)))
    return pos, angle


class SO101IK:
    """A fixed-opening pinch solver. offset_provider(percent) returns SE3 from
    gripper_frame_link to the aperture midpoint, using measured/CAD geometry.
    Dependencies load only when constructing a backend. Use as a context manager.
    """

    def __init__(self, calibration, offset_provider, urdf_path=None, backend_factory=None, joint_limits_deg=None):
        if isinstance(calibration, (str, Path)):
            calibration_bytes = Path(calibration).read_bytes()
            calibration = json.loads(calibration_bytes)
            self.calibration_sha_source = "file-bytes"
        else:
            calibration_bytes = json.dumps(calibration, sort_keys=True, separators=(",", ":")).encode()
            self.calibration_sha_source = "canonical-json"
        self.calibration_sha256 = hashlib.sha256(calibration_bytes).hexdigest()
        self.calibration = {}
        for name in JOINTS:
            entry = calibration[name]
            lo, hi = int(entry["range_min"]), int(entry["range_max"])
            if lo < 0 or hi > 4095 or lo >= hi:
                raise ValueError(f"Invalid calibration range for {name}")
            self.calibration[name] = (lo, hi)
        if not callable(offset_provider):
            raise ValueError("An opening-dependent pinch offset provider is required")
        self.offset_provider = offset_provider
        self.urdf_path = Path(urdf_path or ASSETS / "so101_new_calib.urdf").resolve()
        self.official_urdf_sha256 = hashlib.sha256(self.urdf_path.read_bytes()).hexdigest()
        self._source = ET.parse(self.urdf_path)
        if joint_limits_deg is None:
            self.admitted_joint_limits_deg = None
        else:
            if not hasattr(joint_limits_deg, "keys") or set(joint_limits_deg) != set(ARM_JOINTS):
                raise ValueError("Declared joint limits must specify every arm joint")
            self.admitted_joint_limits_deg = {}
            for name in ARM_JOINTS:
                bounds = np.asarray(joint_limits_deg[name], dtype=float)
                if bounds.shape != (2,) or not np.isfinite(bounds).all() or bounds[0] >= bounds[1]:
                    raise ValueError(f"Invalid declared joint limits for {name}")
                self.admitted_joint_limits_deg[name] = (float(bounds[0]), float(bounds[1]))
        self.limits = []
        for name in ARM_JOINTS:
            limit = self._source.getroot().find(f"joint[@name='{name}']/limit")
            urdf_lo, urdf_hi = np.rad2deg([float(limit.get("lower")), float(limit.get("upper"))])
            lo, hi = self.calibration[name]
            span = (hi - lo) * 180 / 4095
            lo, hi = max(-span, urdf_lo), min(span, urdf_hi)
            if self.admitted_joint_limits_deg is not None:
                admitted_lo, admitted_hi = self.admitted_joint_limits_deg[name]
                lo, hi = max(lo, admitted_lo), min(hi, admitted_hi)
            if lo >= hi:
                raise ValueError(f"Declared joint limits do not overlap URDF/calibration limits for {name}")
            self.limits.append((lo, hi))
        self.limits = np.asarray(self.limits)
        self._factory = backend_factory
        self._backend = None
        self._opening = None
        self._temp = None
        self._closed = False

    @staticmethod
    def available():
        try:
            import placo  # noqa: F401
            from lerobot.model.kinematics import RobotKinematics  # noqa: F401
            return True
        except (ImportError, OSError):
            return False

    def _raw_degrees(self, raw):
        vals = []
        for name in JOINTS:
            value = raw[name]
            lo, hi = self.calibration[name]
            if not np.isfinite(value) or value != int(value) or not lo <= value <= hi:
                raise ValueError(f"Raw {name} outside calibrated range")
            vals.append((value - (lo + hi) / 2) * 360 / 4095)
        if np.any(np.asarray(vals[:5]) < self.limits[:, 0] - 1e-8) or np.any(np.asarray(vals[:5]) > self.limits[:, 1] + 1e-8):
            raise ValueError("Raw pose violates admitted joint limits")
        lo, hi = self.calibration["gripper"]
        return np.asarray(vals[:5]), 100 * (raw["gripper"] - lo) / (hi - lo)

    def _prepare(self, opening):
        if self._closed:
            raise RuntimeError("SO101IK is closed")
        if self._backend is not None and opening == self._opening:
            return self._backend
        offset = _se3(self.offset_provider(opening))
        root = ET.fromstring(ET.tostring(self._source.getroot()))
        for mesh in root.findall(".//mesh"):
            original = Path(mesh.get("filename"))
            candidate = self.urdf_path.parent / original
            if not candidate.exists():
                candidate = self.urdf_path.parent / "cad" / original.name
            if not candidate.exists():
                raise FileNotFoundError(candidate)
            mesh.set("filename", str(candidate.resolve()))
        # RPY convention used by URDF: Rz(yaw) Ry(pitch) Rx(roll).
        r = offset[:3, :3]
        pitch = np.arctan2(-r[2, 0], np.hypot(r[0, 0], r[1, 0]))
        if abs(np.cos(pitch)) > 1e-8:
            roll, yaw = np.arctan2(r[2, 1], r[2, 2]), np.arctan2(r[1, 0], r[0, 0])
        else:
            roll, yaw = 0., np.arctan2(-r[0, 1], r[1, 1])
        ET.SubElement(root, "link", name="cps_pinch")
        joint = ET.SubElement(root, "joint", name="cps_pinch_fixed", type="fixed")
        ET.SubElement(joint, "parent", link="gripper_frame_link")
        ET.SubElement(joint, "child", link="cps_pinch")
        ET.SubElement(joint, "origin", xyz=" ".join(map(str, offset[:3, 3])), rpy=f"{roll} {pitch} {yaw}")
        cache = Path.home() / ".cache" / "exciting-science"
        cache.mkdir(parents=True, exist_ok=True)
        temp = tempfile.TemporaryDirectory(prefix="so101-ik-", dir=cache)
        target = Path(temp.name) / "pinch.urdf"
        ET.ElementTree(root).write(target)
        try:
            factory = self._factory
            if factory is None:
                from lerobot.model.kinematics import RobotKinematics
                factory = RobotKinematics
            backend = factory(str(target), target_frame_name="cps_pinch", joint_names=list(ARM_JOINTS))
            backend.solver.mask_dof("gripper")
            backend.solver.enable_joint_limits(True)
            for name, (lo, hi) in zip(ARM_JOINTS, self.limits):
                backend.robot.set_joint_limits(name, np.deg2rad(lo), np.deg2rad(hi))
            backend.robot.set_joint("gripper", np.deg2rad(-10 + 1.1 * opening))
            backend.robot.update_kinematics()
        except BaseException:
            temp.cleanup()
            raise
        self._backend = backend
        self._opening = opening
        if self._temp is not None:
            self._temp.cleanup()
        self._temp = temp
        return backend

    def _opening_values(self, raw, opening_raw):
        degrees, actual = self._raw_degrees(raw)
        model = actual
        if opening_raw is not None:
            lo, hi = self.calibration["gripper"]
            if not np.isfinite(opening_raw) or opening_raw != int(opening_raw) or not lo <= opening_raw <= hi:
                raise ValueError("Commanded opening outside calibrated range")
            model = 100 * (opening_raw - lo) / (hi - lo)
        return degrees, actual, model

    def forward_provenance(self, raw, opening_raw=None):
        """Measured opening and model opening differ only with an explicit override.
        A commanded opening stabilizes planning against +/- one-count telemetry,
        but does not describe measured finger contact or certify a grasp.
        """
        _, actual, model = self._opening_values(raw, opening_raw)
        return {
            "actual_opening_percent": actual,
            "model_opening_percent": model,
            "opening_source": "measured" if opening_raw is None else "commanded-fixed",
            "calibration_sha256": self.calibration_sha256,
            "calibration_sha_source": self.calibration_sha_source,
            "official_urdf_sha256": self.official_urdf_sha256,
            "declared_admitted_joint_limits_deg": self.admitted_joint_limits_deg,
            "effective_joint_limits_deg": {name: [float(lo), float(hi)] for name, (lo, hi) in zip(ARM_JOINTS, self.limits)},
            "gripper_angle_mapping": "approximate-native-minus10-plus1.1percent",
            "frame": "urdf-base",
            "numerical_ik_only": True,
            "grasp_eligible": False,
        }

    def forward_raw(self, raw, opening_raw=None):
        """FK matrix; override only model opening, never measured body joints.
        Use forward_provenance with the same arguments for opening/hash metadata.
        """
        degrees, _, opening = self._opening_values(raw, opening_raw)
        return np.asarray(self._prepare(opening).forward_kinematics(degrees)).copy()

    def solve_pinch(self, target_pose, seed_raw, *, position_tolerance_m=.002, orientation_tolerance_rad=.05, orientation_weight=0., max_iterations=100):
        """Iterate LeRobot's single-step IK, then verify the rounded motor pose.
        Invalid inputs raise ValueError; unreachable valid targets return failure.
        Orientation is unconstrained by default for the five-DOF arm.
        """
        target = _se3(target_pose)
        q, opening = self._raw_degrees(seed_raw)
        if not np.isfinite([position_tolerance_m, orientation_tolerance_rad, orientation_weight]).all() or position_tolerance_m <= 0 or orientation_tolerance_rad <= 0 or orientation_weight < 0 or not isinstance(max_iterations, int) or max_iterations < 1:
            raise ValueError("Invalid solver tolerances or iteration count")
        backend = self._prepare(opening)
        provenance = self.forward_provenance(seed_raw, opening_raw=seed_raw["gripper"])
        start = time.perf_counter()
        error, angle = float("inf"), float("inf")
        for iteration in range(1, max_iterations + 1):
            try:
                q = np.asarray(backend.inverse_kinematics(q, target, orientation_weight=orientation_weight))[:5]
            except RuntimeError:
                break
            if not np.isfinite(q).all():
                break
            q = np.clip(q, self.limits[:, 0], self.limits[:, 1])
            pose = np.asarray(backend.forward_kinematics(q))
            error, angle = _errors(pose, target)
            if error <= position_tolerance_m and (orientation_weight == 0 or angle <= orientation_tolerance_rad):
                raw = {name: int(round(deg * 4095 / 360 + sum(self.calibration[name]) / 2)) for name, deg in zip(ARM_JOINTS, q)}
                raw["gripper"] = int(seed_raw["gripper"])
                # Quantization near a URDF limit can cross it; never emit that pose.
                try:
                    pose = self.forward_raw(raw, opening_raw=seed_raw["gripper"])
                except ValueError:
                    continue
                error, angle = _errors(pose, target)
                if error <= position_tolerance_m and (orientation_weight == 0 or angle <= orientation_tolerance_rad):
                    return IKResult(True, raw, pose, error, angle, iteration, time.perf_counter() - start, "converged", provenance)
        return IKResult(False, None, None, error, angle, iteration, time.perf_counter() - start, "unreachable or unconverged", provenance)

    def benchmark(self, raw, samples=20):
        """Separate cold build, warm FK, and displaced reachable IK latency.
        No hardware reads or writes. Repeated solves start at the same seed.
        """
        if not isinstance(samples, int) or samples < 1:
            raise ValueError("samples must be a positive integer")
        q, opening = self._raw_degrees(raw)
        cold = self._backend is None or self._opening != opening
        start = time.perf_counter()
        backend = self._prepare(opening)
        prepare_ms = (time.perf_counter() - start) * 1000
        target_q = q.copy()
        target_q[0] += 2. if q[0] + 2 < self.limits[0, 1] else -2.
        if not self.limits[0, 0] <= target_q[0] <= self.limits[0, 1]:
            raise ValueError("Seed pan has no benchmark displacement room")
        target = np.asarray(backend.forward_kinematics(target_q)).copy()
        durations, solve_times, iterations = [], [], []
        successes = []
        for _ in range(samples):
            start = time.perf_counter()
            self.forward_raw(raw, opening_raw=raw["gripper"])
            durations.append(time.perf_counter() - start)
            result = self.solve_pinch(target, raw)
            solve_times.append(result.elapsed_s)
            iterations.append(result.iterations)
            successes.append(result.success)
        return {
            "cold_model_ms": prepare_ms if cold else None,
            "warm_prepare_ms": None if cold else prepare_ms,
            "samples": samples,
            "target_pan_displacement_degrees": float(target_q[0] - q[0]),
            "fk_median_ms": float(np.median(durations) * 1000),
            "fk_p95_ms": float(np.percentile(durations, 95) * 1000),
            "ik_median_ms": float(np.median(solve_times) * 1000),
            "ik_p95_ms": float(np.percentile(solve_times, 95) * 1000),
            "ik_iterations_median": float(np.median(iterations)),
            "ik_iterations_p95": float(np.percentile(iterations, 95)),
            "ik_success": all(successes),
        }

    def close(self):
        self._backend = None
        if self._temp is not None:
            self._temp.cleanup()
            self._temp = None
        self._closed = True

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
