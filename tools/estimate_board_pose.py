"""Estimate camera_from_target from an image of a measured ChArUco board."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np


def camera_parameters(metadata):
    values = metadata.get('intrinsics', metadata)
    matrix = np.array([[values['fx'], 0, values['ppx']], [0, values['fy'], values['ppy']], [0, 0, 1]], float)
    distortion = np.asarray(values.get('coeffs', values.get('distortion', [0]*5)), float)
    model = values.get('model', 'none')
    if str(model).lower() not in ('none', 'brown_conrady', 'distortion.none', 'distortion.brown_conrady') and np.any(distortion):
        raise ValueError('Nonzero distortion requires a supported OpenCV Brown model')
    if not np.isfinite(matrix).all() or matrix[0,0] <= 0 or matrix[1,1] <= 0 or not np.isfinite(distortion).all():
        raise ValueError('Invalid camera intrinsics')
    return matrix, distortion


def solve_corners(board, corners, ids, matrix, distortion):
    if corners is None or ids is None or len(ids) < 6:
        raise ValueError('At least six well-spread ChArUco corners required')
    ids = np.asarray(ids, dtype=np.int32).reshape(-1,1)
    corners = np.asarray(corners, dtype=np.float32).reshape(-1,1,2)
    if len(np.unique(ids)) != len(ids) or not np.isfinite(corners).all():
        raise ValueError('Invalid or duplicate corners')
    objects, pixels = board.matchImagePoints(corners, ids)
    centered = objects.reshape(-1,3)[:, :2]
    centered -= centered.mean(axis=0)
    singular = np.linalg.svd(centered, compute_uv=False)
    if len(singular) < 2 or singular[1] < board.getSquareLength() or singular[1]/singular[0] < .1:
        raise ValueError('Corners must be well spread and noncollinear')
    # matchImagePoints returns an owned array; use pristine object coordinates.
    objects, pixels = board.matchImagePoints(corners, ids)
    ok, rotation, translation = cv2.solvePnP(objects, pixels, matrix, distortion, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or translation[2,0] <= 0:
        raise ValueError('Board pose estimation failed')
    projected = cv2.projectPoints(objects, rotation, translation, matrix, distortion)[0]
    errors = np.linalg.norm(projected.reshape(-1,2)-pixels.reshape(-1,2), axis=1)
    transform = np.eye(4)
    transform[:3,:3] = cv2.Rodrigues(rotation)[0]
    transform[:3,3] = translation.ravel()
    return {'schema': 'camera-target-pose-v1', 'transform_direction': 'camera_from_target',
            'units': 'metres', 'camera_from_target': transform.tolist(), 'corner_ids': ids.ravel().tolist(),
            'reprojection_mean_px': float(errors.mean()), 'reprojection_p95_px': float(np.percentile(errors,95)),
            'qualified': False, 'qualification_note': 'Board frame is not the arm base; requires measured extrinsic calibration'}


def estimate(image, target, intrinsics):
    if target['kind'] != 'charuco':
        raise ValueError('ChArUco target metadata required')
    matrix, distortion = camera_parameters(intrinsics)
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, target['dictionary']))
    board = cv2.aruco.CharucoBoard(tuple(target['squares']), target['square_length_m'], target['marker_length_m'], dictionary)
    parameters = cv2.aruco.CharucoParameters()
    parameters.cameraMatrix = matrix
    parameters.distCoeffs = distortion
    parameters.tryRefineMarkers = True
    detector = cv2.aruco.CharucoDetector(board, parameters)
    corners, ids, _, _ = detector.detectBoard(image)
    return solve_corners(board, corners, ids, matrix, distortion)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('image', 'metadata', 'intrinsics', 'output'):
        parser.add_argument('--'+name, required=True, type=Path)
    args = parser.parse_args()
    image = cv2.imread(str(args.image))
    if image is None:
        raise ValueError('Cannot read image')
    result = estimate(image, json.loads(args.metadata.read_text()), json.loads(args.intrinsics.read_text()))
    result['provenance'] = {'image': str(args.image), 'target_metadata': str(args.metadata), 'intrinsics': str(args.intrinsics)}
    args.output.write_text(json.dumps(result, indent=2)+'\n')
