#!/usr/bin/env python3
"""Offline episode -> support plane -> silhouette contacts -> SO101 numerical IK.

Never connects to an arm. All candidates remain unqualified and non-executable.
Extrinsic JSON must declare {"base_frame": "urdf:base_link",
"direction": "base_from_camera", "base_from_camera": [[...]],
"qualified": false}. Pinch JSON must declare {"parent_frame":
"urdf:gripper_frame_link", "gripper_frame_from_pinch": [[...]],
"opening_raw": 2170, "qualified": false}. The pinch offset is valid only
at exactly that opening. Outputs are offline numerical proposals and never
qualify a grasp for physical execution.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np

from workshop.digital_twin.grasp_geometry import fit_support_plane, footprint_grasps, fuse_depth


def se3(value):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1])
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-6)):
        raise ValueError('Expected proper finite SE3')
    return matrix


def color_masks(rgb):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    hue, saturation, value = np.moveaxis(hsv, -1, 0)
    vivid = (saturation >= 90) & (value >= 50)
    return {'red': vivid & ((hue <= 10) | (hue >= 170)),
            'green': vivid & (hue >= 41) & (hue <= 89),
            'blue': vivid & (hue >= 90) & (hue <= 140),
            'yellow': vivid & (hue >= 15) & (hue <= 40)}


def validate_metadata(metadata, shape):
    intr = metadata['intrinsics']
    scale = float(metadata['depth_scale_m'])
    if metadata.get('aligned_to') != 'color' or not np.isfinite(scale) or scale <= 0:
        raise ValueError('Need color-aligned metric depth metadata')
    if (int(intr['height']), int(intr['width'])) != tuple(shape):
        raise ValueError('Intrinsics dimensions differ from depth')
    if not np.isfinite([intr[k] for k in ('fx', 'fy', 'ppx', 'ppy')]).all() or min(intr['fx'], intr['fy']) <= 0:
        raise ValueError('Invalid pinhole intrinsics')
    if np.any(np.asarray(intr.get('coeffs', [0] * 5), dtype=float) != 0):
        raise ValueError('Nonzero distortion requires explicit undistortion first')
    return intr, scale


def propose_geometry(rgb, depth_stack, metadata, support_roi, color, height_m, maxopening_m=.055):
    """Fit local support excluding colored silhouettes; reject high wrist colors.

    Silhouette projection uses the support plane, explicitly an approximation.
    The height prior supplies the contact midpoint, not measured object topology.
    """
    start = time.perf_counter()
    stack = np.asarray(depth_stack)
    if stack.ndim != 3 or stack.dtype != np.uint16 or not len(stack):
        raise ValueError('Need nonempty uint16 depth stack')
    if rgb.shape != (*stack.shape[1:], 3) or rgb.dtype != np.uint8:
        raise ValueError('Need matching uint8 RGB')
    if (color not in ('red', 'green', 'blue', 'yellow')
            or not np.isfinite([height_m, maxopening_m]).all()
            or min(height_m, maxopening_m) <= 0):
        raise ValueError('Invalid color or positive height prior')
    intr, scale = validate_metadata(metadata, stack.shape[1:])
    height, width = stack.shape[1:]
    fusion_start = time.perf_counter()
    fused = fuse_depth(stack, (0, 0, width, height))
    fusion_ms = 1000 * (time.perf_counter() - fusion_start)
    z = fused['median'] * scale
    projection_start = time.perf_counter()
    yy, xx = np.indices(z.shape)
    points = np.stack(((xx - intr['ppx']) * z / intr['fx'],
                       (yy - intr['ppy']) * z / intr['fy'], z), axis=-1)
    projection_ms = 1000 * (time.perf_counter() - projection_start)
    mask_start = time.perf_counter()
    masks = color_masks(rgb)
    object_union = np.logical_or.reduce(list(masks.values()))
    excluded = cv2.dilate(object_union.astype(np.uint8), np.ones((7, 7), np.uint8)).astype(bool)
    color_mask_ms = 1000 * (time.perf_counter() - mask_start)
    x0, y0, x1, y1 = support_roi
    if not (0 <= x0 < x1 <= width and 0 <= y0 < y1 <= height):
        raise ValueError('Invalid support ROI')
    support_start = time.perf_counter()
    support = np.zeros(z.shape, bool)
    support[y0:y1, x0:x1] = True
    support &= fused['stable_mask'] & ~excluded & (z > 0)
    if support.sum() < 30:
        raise ValueError('Insufficient observed support points')
    plane = fit_support_plane(points[support], threshold_m=.002)
    if plane['inlier_mask'].sum() < 30 or plane['inlier_mask'].mean() < .5:
        raise ValueError('No dominant local support plane')
    support_ms = 1000 * (time.perf_counter() - support_start)
    object_start = time.perf_counter()
    above = points @ plane['normal'] + plane['d']
    selected = (masks[color] & fused['stable_mask'] & (z > 0)
                & (above >= -.002) & (above <= height_m + .003))
    contours, _ = cv2.findContours(selected.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        if cv2.contourArea(contour) < 12 or len(contour) < 3:
            continue
        pixels = contour[:, 0, :]
        for candidate in footprint_grasps(pixels, plane, intr, height_m, maxopening_m):
            candidate['contour_pixels'] = pixels
            candidate['height_prior_m'] = height_m
            candidate['color'] = color
            candidate['executable'] = False
            candidate['grasp_eligible'] = False
            candidate['physical_qualified'] = False
            candidates.append(candidate)
    object_ms = 1000 * (time.perf_counter() - object_start)
    total_ms = 1000 * (time.perf_counter() - start)
    return {'plane_camera': {key: plane[key] for key in ('normal', 'd', 'median_m', 'p95_m', 'all_p95_m')},
            'support_points': int(support.sum()), 'support_inliers': int(plane['inlier_mask'].sum()),
            'selected_color_pixels': int(selected.sum()),
            'rejected_color_pixels': int((masks[color] & ~selected).sum()),
            'candidates': candidates, 'elapsed_s': total_ms / 1000,
            'latency_ms': {'depth_fusion': fusion_ms, 'depth_projection': projection_ms,
                           'support_plane': support_ms,
                           'color_mask': color_mask_ms,
                           'object_candidates': object_ms, 'geometry_total': total_ms},
            'height_source': 'explicit object height prior; silhouette support-plane approximation',
            'opening_limit_source': 'caller nominal geometric limit; no measured opening calibration',
            'executable': False}


def transform_and_solve(report, extrinsic, pinch_offset, calibration, pose_raw, opening_raw,
                        *, no_ik=False, ik_factory=None, desired_rotation=None):
    """Map contacts into the base and optionally solve fixed-opening pinch IK.

    Default rotation is current FK rotation. That deliberately does not enforce
    the extracted closing direction; no IK result qualifies finger orientation.
    """
    if not isinstance(extrinsic.get('qualified'), bool) or not isinstance(pinch_offset.get('qualified'), bool):
        raise ValueError('Extrinsic and pinch metadata require qualified booleans')
    if extrinsic.get('base_frame') != 'urdf:base_link':
        raise ValueError("Extrinsic base_frame must be exactly 'urdf:base_link'")
    if extrinsic.get('direction') != 'base_from_camera':
        raise ValueError("Extrinsic direction must be exactly 'base_from_camera'")
    if pinch_offset.get('parent_frame') != 'urdf:gripper_frame_link':
        raise ValueError("Pinch parent_frame must be exactly 'urdf:gripper_frame_link'")
    transform = se3(extrinsic['base_from_camera'])
    offset = se3(pinch_offset['gripper_frame_from_pinch'])
    if 'opening_raw' not in pinch_offset:
        raise ValueError('Pinch offset must declare its explicit opening_raw')
    declared_opening = pinch_offset['opening_raw']
    if (isinstance(declared_opening, bool) or not isinstance(declared_opening, (int, float))
            or not np.isfinite(declared_opening) or declared_opening != int(declared_opening)
            or int(declared_opening) != opening_raw):
        raise ValueError('Pinch offset is valid only at its explicit opening_raw')
    limits = calibration['gripper']
    if opening_raw != int(opening_raw) or not limits['range_min'] <= opening_raw <= limits['range_max']:
        raise ValueError('Commanded opening outside calibration')
    expected_percent = 100 * (opening_raw - limits['range_min']) / (limits['range_max'] - limits['range_min'])
    def provider(percent):
        if not np.isclose(percent, expected_percent, atol=1e-9, rtol=0):
            raise ValueError('Constant offset requested at another opening')
        return offset.copy()
    ik = None
    latency = report.setdefault('latency_ms', {})
    try:
        if not no_ik:
            ik_init_start = time.perf_counter()
            if ik_factory is None:
                from workshop.digital_twin.ik import SO101IK
                ik_factory = SO101IK
            ik = ik_factory(calibration, provider)
            latency['ik_backend_init'] = 1000 * (time.perf_counter() - ik_init_start)
            fk_start = time.perf_counter()
            current = ik.forward_raw(pose_raw, opening_raw=opening_raw)
            latency['ik_forward_kinematics'] = 1000 * (time.perf_counter() - fk_start)
            rotation = current[:3, :3] if desired_rotation is None else se3(np.block([
                [np.asarray(desired_rotation), np.zeros((3, 1))], [np.zeros((1, 3)), np.ones((1, 1))]]))[:3, :3]
            seed = dict(pose_raw, gripper=opening_raw)
        candidate_stage_start = time.perf_counter()
        ik_solve_ms = 0.
        for candidate in report['candidates']:
            candidate['center_base_m'] = transform[:3, :3] @ candidate['center'] + transform[:3, 3]
            candidate['contacts_base_m'] = np.asarray(candidate['contacts']) @ transform[:3, :3].T + transform[:3, 3]
            candidate['closing_axis_base'] = transform[:3, :3] @ candidate['closing_axis']
            candidate['approach_direction_base'] = transform[:3, :3] @ candidate['approach_direction']
            candidate['ik'] = None
            candidate['orientation_status'] = 'not enforced; pinch axes and contact orientation require verification'
            if ik is not None:
                target = np.eye(4)
                target[:3, :3], target[:3, 3] = rotation, candidate['center_base_m']
                solve_start = time.perf_counter()
                result = ik.solve_pinch(target, seed, orientation_weight=0. if desired_rotation is None else 1.)
                ik_solve_ms += 1000 * (time.perf_counter() - solve_start)
                candidate['ik'] = asdict(result)
                candidate['target_pinch_base'] = target
            candidate['executable'] = False
            candidate['grasp_eligible'] = False
            candidate['physical_qualified'] = False
        latency['candidate_transform_total'] = 1000 * (time.perf_counter() - candidate_stage_start)
        latency['ik_candidate_solve_total'] = ik_solve_ms
        report['provenance'] = {'extrinsic_qualified': extrinsic['qualified'],
                                'pinch_offset_qualified': pinch_offset['qualified'],
                                'pinch_offset_opening_raw': opening_raw,
                                'opening_model_percent': expected_percent,
                                'opening_calibrated': False,
                                'extrinsic_base_frame': extrinsic['base_frame'],
                                'extrinsic_direction': extrinsic['direction'],
                                'pinch_parent_frame': pinch_offset['parent_frame'],
                                'orientation_mode': 'position-only-current-FK' if desired_rotation is None else 'caller-explicit-rotation',
                                'numerical_ik_only': True, 'real_contact_verified': False,
                                'physical_qualification_verified': False}
        return report
    finally:
        if ik is not None:
            ik.close()


def finite_json(value):
    """Unconverged IK can have infinite residuals; encode them as unknown."""
    if isinstance(value, np.ndarray):
        return finite_json(value.tolist())
    if isinstance(value, np.generic):
        return finite_json(value.item())
    if isinstance(value, dict):
        return {key: finite_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json(item) for item in value]
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def load_episode(episode, frame_count=20):
    if not isinstance(frame_count, int) or frame_count < 1:
        raise ValueError('frame_count must be positive')
    episode = Path(episode)
    files = sorted((episode / 'frames').glob('*.npy'))[-frame_count:]
    if not files:
        raise ValueError('Episode has no depth frames')
    rgb_file = files[-1].with_suffix('.png')
    bgr = cv2.imread(str(rgb_file))
    if bgr is None:
        raise ValueError('Matching latest RGB frame missing')
    metadata = json.loads((episode / 'metadata.json').read_text())
    stack = np.stack([np.load(path, allow_pickle=False) for path in files])
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), stack, metadata, files


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episode', type=Path, required=True)
    parser.add_argument('--support-roi', nargs=4, type=int, required=True)
    parser.add_argument('--color', choices=('red', 'green', 'blue', 'yellow'), required=True)
    parser.add_argument('--height-mm', type=float, required=True)
    parser.add_argument('--maxopening-mm', type=float, default=55)
    for name in ('calibration', 'pose-raw', 'extrinsic', 'pinch-offset', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--opening-raw', type=int, required=True)
    parser.add_argument('--no-ik', action='store_true')
    parser.add_argument('--desired-rotation', type=Path,
                        help='Optional JSON 3x3 pinch rotation in base coordinates; enables orientation-weighted IK')
    parser.description += (' Frame JSON requires extrinsic base_frame=urdf:base_link and '
                           'direction=base_from_camera, plus pinch parent_frame='
                           'urdf:gripper_frame_link and opening_raw equal to --opening-raw.')
    args = parser.parse_args()
    rgb, depth, metadata, files = load_episode(args.episode)
    documents = {name: json.loads(getattr(args, name).read_text())
                 for name in ('calibration', 'pose_raw', 'extrinsic', 'pinch_offset')}
    report = propose_geometry(rgb, depth, metadata, args.support_roi, args.color,
                              args.height_mm / 1000, args.maxopening_mm / 1000)
    transform_and_solve(report, documents['extrinsic'], documents['pinch_offset'],
                        documents['calibration'], documents['pose_raw'], args.opening_raw, no_ik=args.no_ik,
                        desired_rotation=None if args.desired_rotation is None else json.loads(args.desired_rotation.read_text()))
    report['episode'] = str(args.episode.resolve())
    report['input_sha256'] = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                              for path in [args.episode / 'metadata.json', *files,
                                           files[-1].with_suffix('.png'), args.calibration,
                                           args.pose_raw, args.extrinsic, args.pinch_offset]}
    if args.desired_rotation is not None:
        report['input_sha256'][str(args.desired_rotation)] = hashlib.sha256(args.desired_rotation.read_bytes()).hexdigest()
    report['depth_frame_count'] = len(files)
    report['nonfinite_residual_encoding'] = 'null means unknown or unconverged; never zero error'
    report['depth_fusion_assumption'] = 'last 20 saved frames static scene; caller must verify no arm/object motion'
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(finite_json(report), indent=2, allow_nan=False) + '\n')


if __name__ == '__main__':
    main()
