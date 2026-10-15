"""Bounded, display-only RGB-D registration diagnostics."""
import math

import numpy as np

BOARD_SQUARES = (7, 5)
BOARD_SQUARE_M = .025
BOARD_MARKER_M = .018
MAX_CHARUCO_CORNERS = 24
ARM_ROI = (0, 375, 300, 565)  # y0, y1, x0, x1 in the saved RGB-D image.


def diagnostic_pair_is_fresh(pair, status):
    return bool(pair is not None and status.get('freshness') == 'fresh'
                and status.get('pairing_status', 'paired') == 'paired')


def diagnostic_source_is_fresh(pair, now_wall, max_age_s=1.25):
    age = diagnostic_source_age_s(pair, now_wall)
    return age is not None and 0. <= age <= max_age_s


def diagnostic_source_age_s(pair, now_wall):
    try:
        age = float(now_wall) - float(pair['timestamp_wall'])
    except (KeyError, TypeError, ValueError):
        return None
    return age if math.isfinite(age) else None


def arm_roi_cloud(world_points, rgb, valid, stride=2):
    """Return a bounded upper-arm ROI cloud, independent of the CAD mesh mask."""
    y0, y1, x0, x1 = ARM_ROI
    y1, x1 = min(y1, valid.shape[0]), min(x1, valid.shape[1])
    roi = valid[y0:y1:stride, x0:x1:stride]
    points = world_points[y0:y1:stride, x0:x1:stride]
    colors = rgb[y0:y1:stride, x0:x1:stride]
    return np.asarray(points[roi], dtype=np.float32), np.asarray(colors[roi], dtype=np.uint8)


def evaluate_charuco_depth(object_points, pixels, depth, intrinsics, depth_scale_m,
                           world_from_camera, registered_camera_from_target=None):
    """Compare RGB PnP board pose with aligned depth; outputs display-only evidence."""
    import cv2

    obj = np.asarray(object_points, dtype=np.float64).reshape(-1, 3)[:MAX_CHARUCO_CORNERS]
    uv = np.asarray(pixels, dtype=np.float64).reshape(-1, 2)[:len(obj)]
    z = np.asarray(depth)
    if len(obj) < 6 or uv.shape != (len(obj), 2) or z.ndim != 2:
        raise ValueError('at least six paired ChArUco corners are required')
    fx, fy = float(intrinsics['fx']), float(intrinsics['fy'])
    cx = float(intrinsics.get('ppx', intrinsics.get('cx')))
    cy = float(intrinsics.get('ppy', intrinsics.get('cy')))
    camera_matrix = np.array([[fx, 0., cx], [0., fy, cy], [0., 0., 1.]])
    distortion = np.zeros(5)
    ok, rotation_vector, translation = cv2.solvePnP(
        obj, uv, camera_matrix, distortion, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise ValueError('ChArUco RGB PnP failed')
    rotation, _ = cv2.Rodrigues(rotation_vector)
    camera_from_target = np.eye(4)
    camera_from_target[:3, :3] = rotation
    camera_from_target[:3, 3] = translation.reshape(3)
    px = np.rint(uv).astype(int)
    good = (px[:, 0] >= 0) & (px[:, 0] < z.shape[1]) & (px[:, 1] >= 0) & (px[:, 1] < z.shape[0])
    indices = np.flatnonzero(good)
    samples = np.full(len(obj), np.nan, dtype=float)
    for index in indices:
        x, y = px[index]
        patch = z[max(0, y-1):y+2, max(0, x-1):x+2].astype(float) * depth_scale_m
        patch = patch[np.isfinite(patch) & (patch > 0)]
        if patch.size:
            samples[index] = np.median(patch)
    good = np.isfinite(samples)
    if int(good.sum()) < 6:
        raise ValueError('fewer than six ChArUco corners have valid depth')
    depth_camera = np.column_stack(((uv[good, 0] - cx) * samples[good] / fx,
                                    (uv[good, 1] - cy) * samples[good] / fy,
                                    samples[good]))
    pnp_camera = obj[good] @ rotation.T + translation.reshape(1, 3)
    errors = np.linalg.norm(depth_camera - pnp_camera, axis=1)
    center = depth_camera.mean(axis=0)
    _, _, vt = np.linalg.svd(depth_camera - center, full_matrices=False)
    normal = vt[-1]
    residuals = (depth_camera - center) @ normal
    rms = float(np.sqrt(np.mean(residuals ** 2)))
    # Make a small depth-fitted plane patch in the RGB-PnP board axes.
    x_axis = rotation[:, 0]
    y_axis = rotation[:, 1]
    relative = depth_camera - center
    xy = np.column_stack((relative @ x_axis, relative @ y_axis))
    lo, hi = xy.min(axis=0), xy.max(axis=0)
    plane_camera = np.array([center + x_axis*x + y_axis*y - normal*(normal @ (center + x_axis*x + y_axis*y - center))
        for x, y in ((lo[0],lo[1]), (hi[0],lo[1]), (hi[0],hi[1]), (lo[0],hi[1]))])
    world = np.asarray(world_from_camera, dtype=float).reshape(4, 4)
    world_points = depth_camera @ world[:3, :3].T + world[:3, 3]
    world_plane = plane_camera @ world[:3, :3].T + world[:3, 3]
    flip = np.diag([1., -1., -1., 1.])
    world_from_detected_board = world @ camera_from_target @ flip
    result = {'status': 'RGBD_DIAGNOSTIC_UNQUALIFIED',
              'corner_count': int(len(obj)), 'depth_corner_count': int(good.sum()),
              'rgb_pnp_depth_residual_rms_mm': float(np.sqrt(np.mean(errors ** 2))*1000),
              'rgb_pnp_depth_residual_median_mm': float(np.median(errors)*1000),
              'depth_plane_rms_mm': rms * 1000,
              'camera_from_target': camera_from_target.tolist(),
              'world_from_detected_board': world_from_detected_board.tolist(),
              'observed_depth_points_camera': depth_camera.tolist(),
              'observed_depth_points_world': world_points.tolist(),
              'observed_depth_plane_world': world_plane.tolist(),
              'source': 'saved paired RGB-D; ChArUco RGB PnP versus aligned depth corners',
              'qualified': False}
    if registered_camera_from_target is not None:
        reference = np.asarray(registered_camera_from_target, dtype=float).reshape(4, 4)
        relative_rotation = reference[:3, :3].T @ rotation
        angle = math.degrees(math.acos(float(np.clip((np.trace(relative_rotation)-1)*.5, -1, 1))))
        result['registered_board_pnp_residual'] = {
            'translation_mm': float(np.linalg.norm(reference[:3, 3] - translation.reshape(3))*1000),
            'rotation_deg': angle,
        }
    return result


def detect_charuco_depth(rgb, depth, intrinsics, depth_scale_m, world_from_camera,
                         registered_camera_from_target=None):
    """Detect at most 24 7x5/25mm ChArUco corners in an already-loaded frame."""
    import cv2
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    board = cv2.aruco.CharucoBoard(BOARD_SQUARES, BOARD_SQUARE_M, BOARD_MARKER_M, dictionary)
    corners, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(
        cv2.cvtColor(np.asarray(rgb), cv2.COLOR_RGB2GRAY))
    if ids is None:
        raise ValueError('no ChArUco markers detected')
    gray = cv2.cvtColor(np.asarray(rgb), cv2.COLOR_RGB2GRAY)
    if hasattr(cv2.aruco, 'CharucoDetector'):
        charuco_corners, charuco_ids, _, _ = cv2.aruco.CharucoDetector(board).detectBoard(
            gray, markerCorners=corners, markerIds=ids)
        count = 0 if charuco_ids is None else len(charuco_ids)
    else:  # OpenCV < 4.7
        count, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, gray, board)
    if charuco_ids is None or int(count) < 6:
        raise ValueError('fewer than six ChArUco corners detected')
    charuco_ids = np.asarray(charuco_ids).reshape(-1)[:MAX_CHARUCO_CORNERS]
    pixels = np.asarray(charuco_corners).reshape(-1, 2)[:MAX_CHARUCO_CORNERS]
    board_points = np.asarray(board.getChessboardCorners(), dtype=float).reshape(-1, 3)
    return evaluate_charuco_depth(board_points[charuco_ids], pixels, depth, intrinsics,
        depth_scale_m, world_from_camera, registered_camera_from_target)
