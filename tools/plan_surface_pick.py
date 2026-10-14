#!/usr/bin/env python3
"""Create an offline, non-executable surface-pick joint plan.

Inputs are one camera-frame candidate (center_cam, closing_axis_cam and optional
footprint_axes_cam), a camera-frame support plane, an explicitly tagged
base_from_camera transform, and an URDF gripper_frame_link_from_pinch offset.
The planner searches both signs of each supplied footprint axis and records
numerical IK, joint interpolation, and pinch-midpoint clearance evidence. It
never imports a hardware transport or sends commands.
"""
import argparse
from dataclasses import asdict, is_dataclass
import hashlib
import json
from pathlib import Path
import numpy as np


WAYPOINTS = (('pregrasp', .060), ('approach', .020), ('grasp', 0.), ('lift', .080))
ARM_JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')


def _se3(value, name):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1., atol=1e-6)):
        raise ValueError(f'{name} must be a proper finite SE3')
    return matrix


def _unit(value, name):
    vector = np.asarray(value, dtype=float)
    norm = np.linalg.norm(vector)
    if vector.shape != (3,) or not np.isfinite(vector).all() or norm < 1e-9:
        raise ValueError(f'{name} must be a finite nonzero 3-vector')
    return vector / norm


def _pinch_rotation(closing_axis, up):
    """Build native-site axes: X points down, Z follows horizontal closing."""
    closing_axis = np.asarray(closing_axis, dtype=float)
    up = _unit(up, 'support-plane up')
    z = closing_axis - up * np.dot(closing_axis, up)
    norm = np.linalg.norm(z)
    if norm < 1e-8:
        raise ValueError('Closing axis is parallel to support-plane normal')
    z /= norm
    x = -up
    y = np.cross(z, x)
    y /= np.linalg.norm(y)
    rotation = np.column_stack((x, y, z))
    if np.linalg.det(rotation) < 0.999999:
        raise ValueError('Constructed pinch rotation is not right-handed')
    return rotation


def _raw_checked(raw, calibration):
    result = {}
    for name, limits in calibration.items():
        value = raw[name]
        lo, hi = int(limits['range_min']), int(limits['range_max'])
        if (isinstance(value, bool) or not np.isfinite(value) or value != int(value)
                or not lo <= int(value) <= hi):
            raise ValueError(f'Raw {name} violates calibrated joint bounds')
        result[name] = int(value)
    return result


def _joint_interpolation(start, end, calibration, max_body_counts=120, max_gripper_counts=650):
    """Linearly sample rounded raw joint commands within the per-leg limits."""
    a, b = _raw_checked(start, calibration), _raw_checked(end, calibration)
    body_delta = max(abs(b[name] - a[name]) for name in ARM_JOINTS)
    grip_delta = abs(b['gripper'] - a['gripper'])
    count = max(1, int(np.ceil(body_delta / max_body_counts)),
                int(np.ceil(grip_delta / max_gripper_counts)))
    samples = []
    for index in range(1, count + 1):
        fraction = index / count
        sample = {name: int(round(a[name] + fraction * (b[name] - a[name]))) for name in a}
        samples.append(_raw_checked(sample, calibration))
    return samples


def _pose_record(result):
    pose = None if result.pose is None else np.asarray(result.pose, dtype=float)
    if pose is not None and (pose.shape != (4, 4) or not np.isfinite(pose).all()):
        pose = None
    return {
        'success': bool(result.success),
        'raw': None if result.raw is None else dict(result.raw),
        'estimated_fk': None if pose is None else pose.tolist(),
        'position_error_m': float(result.position_error_m),
        'orientation_error_rad': float(result.orientation_error_rad),
        'iterations': int(result.iterations),
        'elapsed_s': float(result.elapsed_s),
        'reason': str(result.reason),
        'provenance': result.provenance,
    }


def plan_surface_pick(candidate, support_plane, extrinsic, pinch_offset, calibration, pose_raw,
                      opening_raw, *, ik_factory=None, position_tolerance_m=.001,
                      orientation_tolerance_rad=np.deg2rad(8), orientation_weight=.01,
                      max_body_counts=120, max_gripper_counts=650,
                      position_seed_pregrasp=True, pregrasp_offset_m=.060, input_sha256=None):
    """Return all axis/sign attempts and a best fully solved numerical path.

    `candidate.center_cam`, `candidate.closing_axis_cam`, `support_plane.normal`,
    and `support_plane.d` are in camera coordinates. Optional
    `candidate.footprint_axes_cam` is a list of additional unoriented axes.
    No resulting candidate is executable or physically qualified.
    """
    if extrinsic.get('base_frame') != 'urdf:base_link' or extrinsic.get('direction') != 'base_from_camera':
        raise ValueError("Extrinsic must identify urdf:base_link with direction base_from_camera")
    if not isinstance(extrinsic.get('qualified'), bool):
        raise ValueError('Extrinsic qualified must be an explicit boolean')
    if support_plane.get('frame') != 'camera':
        raise ValueError("Support plane frame must be exactly 'camera'")
    if pinch_offset.get('parent_frame') != 'urdf:gripper_frame_link':
        raise ValueError("Pinch parent_frame must be exactly 'urdf:gripper_frame_link'")
    if not isinstance(pinch_offset.get('qualified'), bool):
        raise ValueError('Pinch offset qualified must be an explicit boolean')
    if pinch_offset.get('opening_raw') != opening_raw:
        raise ValueError('Pinch offset opening_raw must exactly equal requested opening_raw')
    if ('gripper' not in calibration or isinstance(opening_raw, bool)
            or not np.isfinite(opening_raw) or opening_raw != int(opening_raw)):
        raise ValueError('A calibrated integer gripper opening is required')
    if any(name not in calibration for name in ARM_JOINTS):
        raise ValueError('Calibration must include all five arm joints and gripper')
    if not np.isfinite([position_tolerance_m, orientation_tolerance_rad,
                        orientation_weight, max_body_counts, max_gripper_counts]).all():
        raise ValueError('Solver tolerances and interpolation limits must be finite')
    if position_tolerance_m <= 0 or orientation_tolerance_rad <= 0 or orientation_weight <= 0:
        raise ValueError('Position/orientation tolerances and orientation weight must be positive')
    if max_body_counts < 1 or max_gripper_counts < 1:
        raise ValueError('Interpolation limits must be positive')
    if not isinstance(position_seed_pregrasp, bool):
        raise ValueError('position_seed_pregrasp must be a boolean')
    if not np.isfinite(pregrasp_offset_m) or pregrasp_offset_m <= .020:
        raise ValueError('pregrasp_offset_m must be finite and greater than the 20 mm approach offset')
    waypoints = (('pregrasp', float(pregrasp_offset_m)), ('approach', .020),
                 ('grasp', 0.), ('lift', .080))

    base_from_camera = _se3(extrinsic['base_from_camera'], 'base_from_camera')
    gripper_from_pinch = _se3(pinch_offset['gripper_frame_from_pinch'], 'gripper_frame_from_pinch')
    plane_normal_cam = _unit(support_plane['normal'], 'support plane normal')
    plane_d_cam = float(support_plane['d']) / np.linalg.norm(support_plane['normal'])
    if not np.isfinite(plane_d_cam):
        raise ValueError('Support plane d must be finite')
    center_cam = np.asarray(candidate['center_cam'], dtype=float)
    if center_cam.shape != (3,) or not np.isfinite(center_cam).all():
        raise ValueError('candidate.center_cam must be finite XYZ in camera coordinates')
    height_m = float(candidate['height_prior_m'])
    if not np.isfinite(height_m) or height_m <= 0:
        raise ValueError('Candidate height_prior_m must be positive')
    axes = [('closing_axis', _unit(candidate['closing_axis_cam'], 'closing_axis_cam'))]
    for index, axis in enumerate(candidate.get('footprint_axes_cam', [])):
        axes.append((f'footprint_axis_{index}', _unit(axis, f'footprint_axes_cam[{index}]')))

    plane_up_base = base_from_camera[:3, :3] @ plane_normal_cam
    if np.dot(plane_up_base, [0., 0., 1.]) < 0:
        plane_normal_cam = -plane_normal_cam
        plane_d_cam = -plane_d_cam
        plane_up_base = -plane_up_base
    plane_up_base = _unit(plane_up_base, 'base support-plane up')
    base_plane_d = plane_d_cam - np.dot(plane_up_base, base_from_camera[:3, 3])
    center_height = float(np.dot(plane_normal_cam, center_cam) + plane_d_cam)
    contact_cam = center_cam + plane_normal_cam * (height_m / 2 - center_height)
    contact_base = base_from_camera[:3, :3] @ contact_cam + base_from_camera[:3, 3]

    gripper_limits = calibration['gripper']
    if not int(gripper_limits['range_min']) <= opening_raw <= int(gripper_limits['range_max']):
        raise ValueError('Requested gripper opening is outside calibration')
    expected_percent = 100 * (opening_raw - int(gripper_limits['range_min'])) / (
        int(gripper_limits['range_max']) - int(gripper_limits['range_min']))
    def offset_provider(percent):
        if not np.isclose(percent, expected_percent, atol=1e-9, rtol=0):
            raise ValueError('Fixed pinch offset requested at a different gripper opening')
        return gripper_from_pinch.copy()

    if ik_factory is None:
        from workshop.digital_twin.ik import SO101IK
        ik_factory = SO101IK
    solver = ik_factory(calibration, offset_provider)
    attempts = []
    try:
        initial_raw = _raw_checked(dict(pose_raw, gripper=opening_raw), calibration)
        initial_fk = np.asarray(solver.forward_raw(initial_raw, opening_raw=opening_raw), dtype=float)
        _se3(initial_fk, 'initial FK')
        for axis_name, axis_cam in axes:
            axis_base = base_from_camera[:3, :3] @ axis_cam
            for sign in (1, -1):
                attempt = {'axis': axis_name, 'sign': sign, 'waypoints': [],
                           'usable_numerically': False, 'executable': False,
                           'physical_qualified': False, 'finger_clearance_verified': False,
                           'pinch_midpoint_clearance_passed': False}
                try:
                    rotation = _pinch_rotation(sign * axis_base, plane_up_base)
                except ValueError as error:
                    attempt['failure'] = str(error)
                    attempt['orientation_status'] = 'invalid projected closing axis'
                    attempts.append(attempt)
                    continue
                attempt['target_rotation_base'] = rotation.tolist()
                attempt['orientation_status'] = (
                    'native site X=-support-plane-up (finger direction); '
                    'Z=projected closing axis')
                attempt['initial_seed_axis_residuals'] = {
                    'tool_x_down_error_rad': float(np.arccos(np.clip(
                        np.dot(initial_fk[:3, 0], -plane_up_base), -1., 1.))),
                    'tool_z_closing_error_rad': float(np.arccos(np.clip(
                        np.dot(initial_fk[:3, 2], rotation[:, 2]), -1., 1.))),
                }
                seed = dict(initial_raw)
                min_plane_height = float(np.dot(plane_up_base, initial_fk[:3, 3]) + base_plane_d)
                if min_plane_height < -0.001:
                    attempt['failure'] = 'Initial pinch midpoint is over 1 mm below support plane'
                    attempt['blocked_waypoints'] = [name for name, _ in waypoints]
                    attempt['min_sampled_pinch_midpoint_height_above_support_m'] = min_plane_height
                    attempts.append(attempt)
                    continue
                path_ok = True
                for waypoint_name, normal_offset in waypoints:
                    target = np.eye(4)
                    target[:3, :3] = rotation
                    target[:3, 3] = contact_base + plane_up_base * normal_offset
                    position_seed_info = None
                    solve_seed = seed
                    if waypoint_name == 'pregrasp' and position_seed_pregrasp:
                        try:
                            position_target = target.copy()
                            seed_fk = np.asarray(solver.forward_raw(seed, opening_raw=opening_raw), dtype=float)
                            _se3(seed_fk, 'pregrasp position-seed FK')
                            position_target[:3, :3] = seed_fk[:3, :3]
                            position_result = solver.solve_pinch(
                                position_target, seed, position_tolerance_m=position_tolerance_m,
                                orientation_tolerance_rad=orientation_tolerance_rad,
                                orientation_weight=0.)
                            position_seed_info = _pose_record(position_result)
                            position_seed_info.update({'target_base': position_target.tolist(),
                                                       'motion_command': False,
                                                       'purpose': 'IK seed only; never a motion command'})
                            if (not position_result.success or position_result.raw is None
                                    or position_seed_info['estimated_fk'] is None
                                    or position_seed_info['position_error_m'] > position_tolerance_m):
                                attempt['waypoints'].append({'name': waypoint_name,
                                    'success': False, 'target_base': target.tolist(),
                                    'position_only_seed_solve': position_seed_info,
                                    'failure': 'Position-only pregrasp seed solve failed'})
                                path_ok = False
                                break
                            solve_seed = _raw_checked(position_result.raw, calibration)
                            _se3(position_result.pose, 'pregrasp position-seed estimated FK')
                        except Exception as error:
                            attempt['waypoints'].append({'name': waypoint_name,
                                'success': False, 'target_base': target.tolist(),
                                'position_only_seed_solve': position_seed_info,
                                'failure': f'{type(error).__name__}: {error}'})
                            path_ok = False
                            break
                    try:
                        result = solver.solve_pinch(
                            target, solve_seed, position_tolerance_m=position_tolerance_m,
                            orientation_tolerance_rad=orientation_tolerance_rad,
                            orientation_weight=orientation_weight)
                    except Exception as error:
                        attempt['waypoints'].append({'name': waypoint_name, 'success': False,
                            'failure': f'{type(error).__name__}: {error}', 'target_base': target.tolist(),
                            'position_only_seed_solve': position_seed_info})
                        path_ok = False
                        break
                    record = _pose_record(result)
                    record.update({'name': waypoint_name, 'target_base': target.tolist()})
                    if position_seed_info is not None:
                        record['position_only_seed_solve'] = position_seed_info
                    if record['estimated_fk'] is not None:
                        estimated_rotation = np.asarray(record['estimated_fk'])[:3, :3]
                        record['axis_residuals'] = {
                            'tool_x_down_error_rad': float(np.arccos(np.clip(
                                np.dot(estimated_rotation[:, 0], -plane_up_base), -1., 1.))),
                            'tool_z_closing_error_rad': float(np.arccos(np.clip(
                                np.dot(estimated_rotation[:, 2], rotation[:, 2]), -1., 1.))),
                        }
                    else:
                        record['axis_residuals'] = {'tool_x_down_error_rad': None,
                                                    'tool_z_closing_error_rad': None}
                    attempt['waypoints'].append(record)
                    if not result.success or result.raw is None or record['estimated_fk'] is None:
                        path_ok = False
                        break
                    try:
                        next_raw = _raw_checked(result.raw, calibration)
                        fk = _se3(result.pose, f'{waypoint_name} estimated FK')
                        if (record['position_error_m'] > position_tolerance_m
                                or record['orientation_error_rad'] > orientation_tolerance_rad):
                            raise ValueError('Post-solve residual exceeds configured tolerance')
                        samples = _joint_interpolation(seed, next_raw, calibration,
                            max_body_counts=max_body_counts, max_gripper_counts=max_gripper_counts)
                        stage_min = float('inf')
                        for sample in samples:
                            sample_fk = np.asarray(solver.forward_raw(sample, opening_raw=opening_raw), dtype=float)
                            _se3(sample_fk, 'interpolated FK')
                            signed_height = float(np.dot(plane_up_base, sample_fk[:3, 3]) + base_plane_d)
                            stage_min = min(stage_min, signed_height)
                    except (TypeError, ValueError, KeyError) as error:
                        record['success'] = False
                        record['failure'] = str(error)
                        path_ok = False
                        break
                    min_plane_height = min(min_plane_height, stage_min)
                    record['interpolation_samples'] = len(samples)
                    record['interpolation_raw'] = samples
                    record['min_pinch_midpoint_height_above_support_m'] = stage_min
                    if stage_min < -0.001:
                        record['success'] = False
                        record['failure'] = 'Interpolated pinch midpoint crosses support plane by over 1 mm'
                        path_ok = False
                        break
                    record['command_raw'] = next_raw
                    seed = next_raw
                attempt['min_sampled_pinch_midpoint_height_above_support_m'] = (
                    None if not np.isfinite(min_plane_height) else min_plane_height)
                attempt['usable_numerically'] = bool(path_ok and len(attempt['waypoints']) == len(waypoints))
                attempt['pinch_midpoint_clearance_passed'] = attempt['usable_numerically']
                if not path_ok and len(attempt['waypoints']) < len(waypoints):
                    attempt['blocked_waypoints'] = [name for name, _ in waypoints[len(attempt['waypoints']):]]
                attempts.append(attempt)
    finally:
        solver.close()

    successful = [i for i, attempt in enumerate(attempts) if attempt['usable_numerically']]
    best = min(successful, key=lambda i: (
        sum(w['orientation_error_rad'] for w in attempts[i]['waypoints']),
        sum(w['position_error_m'] for w in attempts[i]['waypoints']))) if successful else None
    planned_commands = []
    if best is not None:
        for waypoint_index, waypoint in enumerate(attempts[best]['waypoints']):
            samples = waypoint['interpolation_raw']
            for sample_index, raw in enumerate(samples, start=1):
                planned_commands.append({'waypoint': waypoint['name'],
                    'sample_index': sample_index, 'sample_count': len(samples),
                    'final_waypoint': sample_index == len(samples), 'raw': raw})
    return {
        'schema': 'offline_surface_pick_plan_v1',
        'frames': {'camera': 'camera', 'base': 'urdf:base_link',
                   'pinch_offset_parent': 'urdf:gripper_frame_link',
                   'pinch_axes': 'native-MuJoCo-gripper_frame_site; X=finger-down, Z=closing'},
        'parameters': {'opening_raw': opening_raw, 'height_prior_m': height_m,
                       'position_tolerance_m': position_tolerance_m,
                       'orientation_tolerance_rad': orientation_tolerance_rad,
                       'orientation_weight': orientation_weight,
                       'position_seed_pregrasp': position_seed_pregrasp,
                       'pregrasp_offset_m': pregrasp_offset_m,
                       'max_body_counts_per_leg': max_body_counts,
                       'max_gripper_counts_per_leg': max_gripper_counts,
                       'waypoint_offsets_from_grasp_m': dict(waypoints)},
        'input_sha256': input_sha256 or {},
        'initial_seed_raw': initial_raw,
        'initial_estimated_fk': initial_fk.tolist(),
        'planned_commands': planned_commands,
        'execution_performed': False,
        'attempts': attempts,
        'best_attempt_index': best,
        'usable_numerically': best is not None,
        'executable': False,
        'grasp_eligible': False,
        'physical_qualified': False,
        'finger_clearance_verified': False,
        'provenance': {'extrinsic_qualified': extrinsic['qualified'],
                       'pinch_offset_qualified': pinch_offset['qualified'],
                       'opening_offset_raw': opening_raw, 'numerical_ik_only': True,
                       'physical_evaluation_required': True},
    }


def _json_safe(value):
    if is_dataclass(value):
        return _json_safe(asdict(value))
    if isinstance(value, np.ndarray):
        return _json_safe(value.tolist())
    if isinstance(value, np.generic):
        return _json_safe(value.item())
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('candidate', 'support-plane', 'extrinsic', 'pinch-offset',
                 'calibration', 'pose-raw', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--opening-raw', type=int, required=True)
    parser.add_argument('--position-tolerance-mm', type=float, default=1.)
    parser.add_argument('--orientation-tolerance-deg', type=float, default=8.)
    parser.add_argument('--orientation-weight', type=float, default=.01)
    parser.add_argument('--pregrasp-mm', type=float, default=60.)
    args = parser.parse_args()
    paths = [getattr(args, name.replace('-', '_')) for name in
             ('candidate', 'support_plane', 'extrinsic', 'pinch_offset', 'calibration', 'pose_raw')]
    docs = [json.loads(path.read_text()) for path in paths]
    hashes = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}
    plan = plan_surface_pick(*docs, args.opening_raw,
        position_tolerance_m=args.position_tolerance_mm / 1000,
        orientation_tolerance_rad=np.deg2rad(args.orientation_tolerance_deg),
        orientation_weight=args.orientation_weight,
        pregrasp_offset_m=args.pregrasp_mm / 1000, input_sha256=hashes)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(_json_safe(plan), indent=2, allow_nan=False) + '\n')


if __name__ == '__main__':
    main()
