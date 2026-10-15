"""Saved RGB AprilTag localization helpers for the unqualified twin overlay."""
import math

import numpy as np

TAG_ID = 0
TAG_SIZE_M = .0194
BOOTSTRAP_CENTER_PX = (377., 257.)
BOOTSTRAP_ROI_PX = (330., 200., 440., 290.)
TRACK_MAX_SHIFT_PX = 40.
TRACK_AMBIGUITY_MARGIN_PX = 8.
DETECTION_INTERVAL_S = .2
POSE_PAIR_LIMIT_S = .16
IPPE_BRANCH_SEPARATION_DEG = 15.
DEPTH_NORMAL_MARGIN_DEG = 10.
DEPTH_NORMAL_MIN_MODULES = 10
DEPTH_NORMAL_MAX_RMS_M = .001


def detect_apriltag_25h9(rgb):
    """Detect 25h9 IDs from an already-loaded RGB frame; never opens a device/file."""
    import cv2
    rgb = np.asarray(rgb)
    if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
        raise ValueError('expected an RGB uint8 image')
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_25h9)
    parameters = cv2.aruco.DetectorParameters()
    corners, identifiers, _ = cv2.aruco.ArucoDetector(dictionary, parameters).detectMarkers(
        cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))
    if identifiers is None:
        return []
    return [(int(marker_id), np.asarray(points, dtype=np.float32).reshape(1, 4, 2))
            for marker_id, points in zip(identifiers.ravel(), corners)]


def track_tag_candidates(detections, previous_center=None,
                         bootstrap_center=BOOTSTRAP_CENTER_PX,
                         bootstrap_roi=BOOTSTRAP_ROI_PX,
                         max_shift_px=TRACK_MAX_SHIFT_PX,
                         ambiguity_margin_px=TRACK_AMBIGUITY_MARGIN_PX):
    """Bootstrap in a known wrist ROI, then associate only nearby ID-0 detections."""
    candidates = [(corners.reshape(4, 2).mean(axis=0), corners)
                  for marker_id, corners in detections if marker_id == TAG_ID]
    anchor = np.asarray(bootstrap_center if previous_center is None else previous_center,
                        dtype=float)
    limit = max_shift_px
    nearby = [(float(np.linalg.norm(center - anchor)), center, corners)
              for center, corners in candidates
              if ((bootstrap_roi[0] <= center[0] <= bootstrap_roi[2]
                   and bootstrap_roi[1] <= center[1] <= bootstrap_roi[3])
                  if previous_center is None else float(np.linalg.norm(center - anchor)) <= limit)]
    nearby.sort(key=lambda item: item[0])
    if not nearby:
        return {'status': 'missing', 'center_px': None, 'corners': None}
    if previous_center is None and len(nearby) != 1:
        return {'status': 'ambiguous', 'center_px': None, 'corners': None}
    if len(nearby) > 1 and nearby[1][0] - nearby[0][0] < ambiguity_margin_px:
        return {'status': 'ambiguous', 'center_px': None, 'corners': None}
    return {'status': 'tracked', 'center_px': nearby[0][1].copy(),
            'corners': nearby[0][2].copy()}


def paired_pose_raw(pair):
    """Return only an actual raw pose whose camera pairing is qualified and <=160 ms."""
    if not isinstance(pair, dict) or pair.get('freshness', 'fresh') not in ('fresh', 'fresh-live'):
        return None
    raw = pair.get('pose_raw')
    if not isinstance(raw, dict) or not raw:
        return None
    if pair.get('pose_qualified_sync') is True:
        delta = pair.get('pose_sample_delta_seconds')
    elif pair.get('pose_synchronized') is True:
        delta = pair.get('phase_event_delta_s')
    else:
        return None
    try:
        delta = float(delta)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(delta) or abs(delta) > POSE_PAIR_LIMIT_S:
        return None
    return raw


def expected_tag_transform(base_from_native_site, gripper_from_tag,
                           base_from_camera):
    """Predict camera_from_tag from actual base FK and the URDF gripper frame."""
    from .frame_conventions import NATIVE_SITE_TO_URDF_FRAME
    base_from_native_site = _rigid(base_from_native_site, 'base_from_native_site')
    gripper_from_tag = _rigid(gripper_from_tag, 'gripper_from_tag')
    base_from_camera = _rigid(base_from_camera, 'base_from_camera')
    base_from_gripper = base_from_native_site @ np.linalg.inv(NATIVE_SITE_TO_URDF_FRAME)
    return np.linalg.inv(base_from_camera) @ base_from_gripper @ gripper_from_tag


def base_from_native_site(model, data):
    """Read MuJoCo's actual gripper site as arm-base-relative native FK."""
    import mujoco
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'arm_base')
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'gripper_frame_site')
    if base_id < 0 or site_id < 0:
        raise ValueError('model lacks arm_base or gripper_frame_site')
    base_rotation = data.xmat[base_id].reshape(3, 3)
    site_rotation = data.site_xmat[site_id].reshape(3, 3)
    result = np.eye(4)
    result[:3, :3] = base_rotation.T @ site_rotation
    result[:3, 3] = base_rotation.T @ (data.site_xpos[site_id] - data.xpos[base_id])
    return result


def tag_pose_residual(expected_camera_from_tag, observed_camera_from_tag,
                      intrinsics, marker_size_m=TAG_SIZE_M):
    """Return metric, projected-center and rotational residuals for two camera poses."""
    expected = _rigid(expected_camera_from_tag, 'expected_camera_from_tag')
    observed = _rigid(observed_camera_from_tag, 'observed_camera_from_tag')
    expected_px = _project(expected[:3, 3][None], intrinsics)[0]
    observed_px = _project(observed[:3, 3][None], intrinsics)[0]
    relative_rotation = expected[:3, :3].T @ observed[:3, :3]
    angle = math.degrees(math.acos(float(np.clip((np.trace(relative_rotation) - 1) / 2, -1, 1))))
    return {'translation_mm': float(np.linalg.norm(expected[:3, 3] - observed[:3, 3]) * 1000),
            'pixel': float(np.linalg.norm(expected_px - observed_px)),
            'rotation_deg': angle,
            'expected_center_px': expected_px.tolist(), 'observed_center_px': observed_px.tolist(),
            'expected_corners_px': project_tag_corners(expected, intrinsics, marker_size_m).tolist(),
            'observed_corners_px': project_tag_corners(observed, intrinsics, marker_size_m).tolist()}


def project_tag_corners(camera_from_tag, intrinsics, marker_size_m=TAG_SIZE_M):
    half = float(marker_size_m) / 2
    object_points = np.array([[-half, half, 0], [half, half, 0],
                              [half, -half, 0], [-half, -half, 0]], dtype=float)
    transform = _rigid(camera_from_tag, 'camera_from_tag')
    return _project(object_points @ transform[:3, :3].T + transform[:3, 3], intrinsics)


def select_ippe_pose_candidate(candidates, previous_camera_from_tag=None,
                              depth_normal_camera=None, depth_valid_module_count=0,
                              depth_plane_rms_m=None):
    """Select an IPPE pose, marking separated branches uncertain without depth.

    Depth normal selection is enabled only for a well-supported, low-residual
    plane. A previous pose may help when candidates describe the same normal,
    but cannot resolve a separated mirror branch.
    """
    candidates = list(candidates)
    if not candidates:
        return {'camera_from_tag': None, 'orientation_status': 'invalid',
                'branch_separation_deg': None, 'rotation_deg': None}
    candidates.sort(key=lambda item: item['reprojection_error_px'])
    normals = [np.asarray(item['camera_from_tag'], dtype=float)[:3, 2]
               for item in candidates]
    separation = max((math.degrees(math.acos(float(np.clip(a @ b, -1., 1.))))
                      for i, a in enumerate(normals) for b in normals[i + 1:]),
                     default=0.)
    separated = separation >= IPPE_BRANCH_SEPARATION_DEG
    depth_qualified = (depth_normal_camera is not None
        and type(depth_valid_module_count) is int
        and depth_valid_module_count >= DEPTH_NORMAL_MIN_MODULES
        and depth_plane_rms_m is not None and math.isfinite(float(depth_plane_rms_m))
        and float(depth_plane_rms_m) < DEPTH_NORMAL_MAX_RMS_M)
    if separated and depth_qualified:
        normal = np.asarray(depth_normal_camera, dtype=float).reshape(3)
        length = float(np.linalg.norm(normal))
        if np.isfinite(normal).all() and length > 1e-9:
            normal /= length
            # SVD plane normals have arbitrary sign. The visible tag surface
            # faces the camera, so orient the normal toward the camera origin.
            representative_position = np.mean(
                [np.asarray(item['camera_from_tag'])[:3, 3] for item in candidates], axis=0)
            if float(normal @ representative_position) > 0.:
                normal = -normal
            errors = [math.degrees(math.acos(float(np.clip(item @ normal, -1., 1.))))
                      for item in normals]
            order = np.argsort(errors)
            margin = (errors[order[1]] - errors[order[0]]) if len(order) > 1 else 180.
            if margin >= DEPTH_NORMAL_MARGIN_DEG:
                return {'camera_from_tag': candidates[int(order[0])]['camera_from_tag'],
                        'orientation_status': 'depth_disambiguated',
                        'branch_separation_deg': separation,
                        'rotation_deg': None}
    if separated:
        return {'camera_from_tag': candidates[0]['camera_from_tag'],
                'orientation_status': 'orientation_uncertain',
                'branch_separation_deg': separation, 'rotation_deg': None}
    if previous_camera_from_tag is not None:
        previous = _rigid(previous_camera_from_tag, 'previous_camera_from_tag')
        def continuity(item):
            relative = previous[:3, :3].T @ item['camera_from_tag'][:3, :3]
            angle = math.acos(float(np.clip((np.trace(relative) - 1) / 2, -1, 1)))
            return angle, item['reprojection_error_px']
        chosen = min(candidates, key=continuity)
    else:
        chosen = candidates[0]
    return {'camera_from_tag': chosen['camera_from_tag'],
            'orientation_status': 'resolved_pnp',
            'branch_separation_deg': separation, 'rotation_deg': None}


def estimate_camera_from_tag(corners, intrinsics, marker_size_m=TAG_SIZE_M,
                             previous_camera_from_tag=None, *, depth_normal_camera=None,
                             depth_valid_module_count=0, depth_plane_rms_m=None,
                             return_diagnostics=False):
    """Estimate a selected tag pose from its four detected corners, without image I/O."""
    import cv2
    half = float(marker_size_m) / 2
    object_points = np.array([[-half, half, 0], [half, half, 0],
                              [half, -half, 0], [-half, -half, 0]], dtype=np.float32)
    points = np.asarray(corners, dtype=np.float32).reshape(4, 2)
    camera = _camera_matrix(intrinsics)
    distortion = np.asarray(intrinsics.get('distortion', intrinsics.get('coeffs', np.zeros(5))),
                            dtype=np.float64)
    result = cv2.solvePnPGeneric(object_points, points, camera, distortion,
                                 flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not result[0]:
        diagnostic = {'camera_from_tag': None, 'orientation_status': 'invalid',
                      'branch_separation_deg': None, 'rotation_deg': None}
        return diagnostic if return_diagnostics else None
    errors = result[3]
    candidates = []
    for index, (rotation_vector, translation) in enumerate(zip(result[1], result[2])):
        if not np.isfinite(rotation_vector).all() or not np.isfinite(translation).all() or translation[2, 0] <= 0:
            continue
        rotation, _ = cv2.Rodrigues(rotation_vector)
        transform = np.eye(4)
        transform[:3, :3], transform[:3, 3] = rotation, translation[:, 0]
        reprojection_error = (float(np.asarray(errors).reshape(-1)[index])
                              if errors is not None else float(index))
        candidates.append({'camera_from_tag': transform,
                           'reprojection_error_px': reprojection_error})
    diagnostic = select_ippe_pose_candidate(candidates,
        previous_camera_from_tag=previous_camera_from_tag,
        depth_normal_camera=depth_normal_camera,
        depth_valid_module_count=depth_valid_module_count,
        depth_plane_rms_m=depth_plane_rms_m)
    return diagnostic if return_diagnostics else diagnostic['camera_from_tag']


def draw_tag_comparison(rgb, observed_corners, expected_corners=None):
    """Return display-only RGB copy with optional expected and observed outlines."""
    from PIL import Image, ImageDraw
    image = Image.fromarray(np.asarray(rgb, dtype=np.uint8).copy())
    draw = ImageDraw.Draw(image)
    overlays = [(observed_corners, (255, 215, 20))]
    if expected_corners is not None:
        overlays.insert(0, (expected_corners, (20, 230, 255)))
    for corners, color in overlays:
        points = [tuple(np.rint(point).astype(int)) for point in np.asarray(corners).reshape(4, 2)]
        draw.line(points + [points[0]], fill=color, width=2)
        for x, y in points:
            draw.ellipse((x - 2, y - 2, x + 2, y + 2), fill=color)
    return np.asarray(image)


def detection_due(now, last_detection, min_interval_s=DETECTION_INTERVAL_S):
    return (last_detection is None
            or float(now) - float(last_detection) >= min_interval_s - 1e-9)


def evaluate_tag_overlay(rgb, pair, expected_camera_from_tag, intrinsics,
                         previous_center=None, previous_camera_from_tag=None,
                         display_image=None, depth=None, depth_scale_m=None):
    """Compare one saved image and its paired actual pose; stale pairs are hidden."""
    rgb = np.asarray(rgb, dtype=np.uint8)
    display = rgb if display_image is None else np.asarray(display_image, dtype=np.uint8)
    if paired_pose_raw(pair) is None:
        return {'status': 'STALE / UNPAIRED · marker hidden', 'visible': False,
                'center_px': None, 'expected_camera_from_tag': None,
                'observed_camera_from_tag': None, 'image': display.copy(), 'residual': None}
    detections = detect_apriltag_25h9(rgb)
    selection = track_tag_candidates(detections, previous_center=previous_center)
    if selection['status'] != 'tracked':
        return {'status': selection['status'].upper() + ' ID0 · comparison hidden',
                'visible': False, 'center_px': None, 'expected_camera_from_tag': None,
                'observed_camera_from_tag': None, 'image': display.copy(), 'residual': None}
    depth_estimate = None
    depth_pose = None
    depth_anchor = {'status': 'UNAVAILABLE', 'reason': 'paired depth not supplied'}
    if depth is not None and depth_scale_m is not None:
        try:
            from workshop.perception.tag_depth_pose import estimate_tag_pose
            import cv2
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_25h9)
            inner = np.asarray(dictionary.getBitsFromByteList(dictionary.bytesList[0:1], 5),
                               dtype=np.uint8)
            bits = np.zeros((7, 7), dtype=np.uint8)
            bits[1:6, 1:6] = inner
            depth_intrinsics = dict(intrinsics)
            coeffs = np.asarray(depth_intrinsics.get('coeffs', ()), dtype=float)
            if coeffs.size and np.all(np.abs(coeffs) <= 1e-12):
                depth_intrinsics['model'] = 'none'
            depth_estimate = estimate_tag_pose(selection['corners'].reshape(4, 2), rgb, depth,
                depth_intrinsics, depth_scale_m, bits, white_rgb_min=100)
            depth_pose = np.asarray(depth_estimate['camera_from_tag'], dtype=float)
            depth_anchor = {'status': 'PROVISIONAL_DEPTH_TRANSLATION',
                'qualified': False,
                'black_edge_side_lengths_m': depth_estimate['black_edge_side_lengths_m'],
                'nominal_size_m': TAG_SIZE_M, 'plane': depth_estimate['plane'],
                'valid_module_count': depth_estimate['valid_module_count'],
                'module_count': depth_estimate['module_count']}
        except Exception as exc:
            depth_anchor = {'status': 'UNAVAILABLE · PnP ONLY UNQUALIFIED',
                            'reason': str(exc)}
    pose_diagnostic = estimate_camera_from_tag(selection['corners'], intrinsics,
        previous_camera_from_tag=previous_camera_from_tag,
        depth_normal_camera=(depth_estimate['plane']['normal_camera']
                             if depth_estimate is not None else None),
        depth_valid_module_count=(depth_estimate['valid_module_count']
                                  if depth_estimate is not None else 0),
        depth_plane_rms_m=(depth_estimate['plane']['rms_m']
                           if depth_estimate is not None else None),
        return_diagnostics=True)
    observed = pose_diagnostic['camera_from_tag']
    if observed is None:
        return {'status': 'TAG POSE INVALID · comparison hidden', 'visible': False,
                'center_px': None, 'expected_camera_from_tag': None,
                'observed_camera_from_tag': None, 'image': display.copy(), 'residual': None}
    orientation_uncertain = pose_diagnostic['orientation_status'] == 'orientation_uncertain'
    expected = _rigid(expected_camera_from_tag, 'expected_camera_from_tag')
    nominal_residual = tag_pose_residual(expected, observed, intrinsics)
    residual = nominal_residual
    reported_pose = observed.copy()
    if depth_pose is not None:
        reported_pose[:3, 3] = depth_pose[:3, 3]
        residual = tag_pose_residual(expected, reported_pose, intrinsics)
        depth_anchor.update(
            translation_delta_z_m=float(depth_pose[2, 3] - observed[2, 3]),
            translation_delta_xyz_m=(depth_pose[:3, 3] - observed[:3, 3]).tolist(),
            observed_camera_from_tag=reported_pose.tolist())
    if orientation_uncertain:
        residual['rotation_deg'] = None
        nominal_residual['rotation_deg'] = None
    image = draw_tag_comparison(display, residual['observed_corners_px'],
        None if orientation_uncertain else residual['expected_corners_px'])
    if orientation_uncertain:
        status = ('SOURCE UNQUALIFIED · ORIENTATION UNCERTAIN · observed center retained'
                  + (' · depth-anchored position provisional' if depth_pose is not None
                     else ' · depth position unavailable'))
    elif depth_pose is not None:
        status = ('SOURCE UNQUALIFIED · depth anchored Δ %.1f mm / %.1f px / %.1f° '
                  '(nominal PnP Δ %.1f mm)' % (residual['translation_mm'], residual['pixel'],
                  residual['rotation_deg'], nominal_residual['translation_mm']))
    else:
        status = ('SOURCE UNQUALIFIED · PnP only Δ %.1f mm / %.1f px / %.1f° '
                  '· depth unavailable' % (residual['translation_mm'], residual['pixel'],
                  residual['rotation_deg']))
    return {'status': status,
            'visible': True, 'center_px': selection['center_px'],
            'expected_camera_from_tag': expected,
            'observed_camera_from_tag': reported_pose, 'image': image,
            'residual': residual, 'nominal_pnp_residual': nominal_residual,
            'depth_anchor': depth_anchor,
            'orientation_status': pose_diagnostic['orientation_status'],
            'branch_separation_deg': pose_diagnostic['branch_separation_deg'],
            'depth_anchored_camera_from_tag': (reported_pose if depth_pose is not None else None)}


def validate_fiducial_calibration(payload):
    """Extract the held fit transforms and preserve its unqualified provenance."""
    fit = payload.get('fit') if isinstance(payload, dict) else None
    if not isinstance(fit, dict):
        raise ValueError('fiducial calibration must contain fit transforms')
    return {'base_from_camera': _rigid(fit['base_from_camera'], 'base_from_camera'),
            'gripper_from_tag': _rigid(fit['gripper_from_tag'], 'gripper_from_tag'),
            'source_status': 'SOURCE UNQUALIFIED', 'tag_id': TAG_ID,
            'dictionary': 'DICT_APRILTAG_25h9', 'tag_size_m': TAG_SIZE_M}


def _camera_matrix(intrinsics):
    cx = intrinsics.get('cx', intrinsics.get('ppx'))
    cy = intrinsics.get('cy', intrinsics.get('ppy'))
    return np.array([[float(intrinsics['fx']), 0., float(cx)],
                     [0., float(intrinsics['fy']), float(cy)],
                     [0., 0., 1.]], dtype=np.float64)


def _project(points_camera, intrinsics):
    points = np.asarray(points_camera, dtype=float).reshape(-1, 3)
    z = points[:, 2]
    if np.any(z <= 0):
        return np.full((len(points), 2), np.nan)
    camera = _camera_matrix(intrinsics)
    return np.column_stack((camera[0, 0] * points[:, 0] / z + camera[0, 2],
                            camera[1, 1] * points[:, 1] / z + camera[1, 2]))


def _rigid(value, name):
    transform = np.asarray(value, dtype=float)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError(f'{name} must be a finite 4x4 transform')
    if not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-8):
        raise ValueError(f'{name} must have a rigid homogeneous last row')
    rotation = transform[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=2e-5)
            or not np.isclose(np.linalg.det(rotation), 1., atol=2e-5)):
        raise ValueError(f'{name} rotation must be rigid')
    return transform
