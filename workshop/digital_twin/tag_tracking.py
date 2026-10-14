"""Repeated-ID AprilTag observations; optical camera axes x right/y down/z forward.

Track numbers are temporal image associations, not certified physical/link IDs.
Every pose remains unqualified until a separate measured calibration validates it.
"""
import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment


def black_edge_from_total(total_size_m):
    """tag25h9 generator full tile is 9 modules; black border is 7 modules."""
    square_points(total_size_m)
    return float(total_size_m) * 7 / 9


def camera_matrix(intrinsics):
    matrix = np.array([[intrinsics['fx'], 0, intrinsics['ppx']],
                       [0, intrinsics['fy'], intrinsics['ppy']], [0, 0, 1.]], dtype=float)
    if not np.isfinite(matrix).all() or min(matrix[0, 0], matrix[1, 1]) <= 0:
        raise ValueError('Invalid camera intrinsics')
    return matrix


def square_points(size_m):
    """OpenCV IPPE_SQUARE order matches detected TL, TR, BR, BL corners.

    Tag +x points right, +y points toward its top, +z out of the printed face.
    Size is the outer black border edge, excluding any white margin.
    """
    h = float(size_m) / 2
    if not np.isfinite(h) or h <= 0:
        raise ValueError('Tag size must be positive finite metres')
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=float)


def depth_size(corners, depth, intrinsics, depth_scale_m, candidates=(.0075, .0194)):
    """Infer provisional edge size from robust 3x3 aligned-depth corner patches."""
    corners = np.asarray(corners, dtype=float).reshape(4, 2)
    if not np.isfinite(corners).all() or not np.isfinite(depth_scale_m) or depth_scale_m <= 0:
        raise ValueError('Invalid corners or depth scale')
    k = camera_matrix(intrinsics)
    points, depths = [], []
    for u, v in corners:
        x, y = np.rint([u, v]).astype(int)
        if not (1 <= x < depth.shape[1]-1 and 1 <= y < depth.shape[0]-1):
            return {'size_m': None, 'reason': 'corner_depth_unavailable'}
        values = np.asarray(depth[y-1:y+2, x-1:x+2], dtype=float).ravel() * depth_scale_m
        values = values[np.isfinite(values) & (values > 0)]
        if len(values) < 5:
            return {'size_m': None, 'reason': 'corner_depth_unavailable'}
        z = float(np.median(values))
        consistent = values[np.abs(values-z) <= max(.003, z*.01)]
        if len(consistent) < 5:
            return {'size_m': None, 'reason': 'corner_depth_spread'}
        depths.append(z)
        points.append([(u-k[0, 2])*z/k[0, 0], (v-k[1, 2])*z/k[1, 1], z])
    points = np.asarray(points)
    edges = np.linalg.norm(np.roll(points, -1, axis=0)-points, axis=1)
    measured = float(np.median(edges))
    candidates = np.asarray(candidates, dtype=float)
    if not len(candidates) or not np.isfinite(candidates).all() or (candidates <= 0).any():
        raise ValueError('Invalid size candidates')
    errors = np.abs(candidates-measured)/candidates
    best = int(np.argmin(errors))
    # A single edge on background depth must not qualify an otherwise plausible median.
    edge_error = float(np.max(np.abs(edges-candidates[best])/candidates[best]))
    return {'size_m': float(candidates[best]) if edge_error <= .20 else None,
            'reason': 'provisional_depth_size' if edge_error <= .20 else 'size_mismatch',
            'measured_edge_m': measured, 'edge_lengths_m': edges.tolist(),
            'edge_mad_m': float(np.median(np.abs(edges-measured))),
            'relative_error': float(errors[best]), 'max_edge_relative_error': edge_error,
            'corner_depth_m': depths, 'source': 'aligned_depth_provisional'}


def pose_candidates(corners, intrinsics, size_m, distortion=None, corner_depth_m=None):
    """Expose both planar PnP hypotheses instead of certifying a front-facing pose."""
    object_points = square_points(size_m)
    corners = np.asarray(corners, dtype=float).reshape(4, 2)
    k = camera_matrix(intrinsics)
    distortion = np.zeros(5) if distortion is None else np.asarray(distortion, dtype=float)
    result = cv2.solvePnPGeneric(object_points, corners, k, distortion,
                                 flags=cv2.SOLVEPNP_IPPE_SQUARE)
    poses = []
    if not result[0]:
        return {'candidates': [], 'ambiguous': True, 'selected_index': None}
    for rvec, tvec in zip(result[1], result[2]):
        rotation = cv2.Rodrigues(rvec)[0]
        translation = tvec.ravel()
        camera_points = object_points @ rotation.T + translation
        if not np.isfinite(camera_points).all() or (camera_points[:, 2] <= 0).any():
            continue
        pixels = cv2.projectPoints(object_points, rvec, tvec, k, distortion)[0].reshape(4, 2)
        transform = np.eye(4)
        transform[:3, :3], transform[:3, 3] = rotation, translation
        poses.append({'camera_from_tag': transform.tolist(),
                      'reprojection_rms_px': float(np.sqrt(np.mean(np.sum((pixels-corners)**2, axis=1)))),
                      'corner_depth_rms_m': None if corner_depth_m is None else float(np.sqrt(
                          np.mean((camera_points[:, 2]-corner_depth_m)**2)))})
    poses.sort(key=lambda p: p['reprojection_rms_px'])
    selected = None
    if len(poses) == 1:
        selected = 0
    elif len(poses) == 2:
        gap = poses[1]['reprojection_rms_px']-poses[0]['reprojection_rms_px']
        # Depth can support a clearly separated hypothesis; near-front poses retain ambiguity.
        if corner_depth_m is not None and gap > .5:
            depths = [p['corner_depth_rms_m'] for p in poses]
            if depths[0] < .005 and depths[1]-depths[0] > .002:
                selected = 0
    return {'candidates': poses, 'ambiguous': selected is None, 'selected_index': selected}


class TagTracker:
    """Instance-local nearest/Hungarian tracks, including multiple code ID 0s.

    Defaults are operator-declared outer black edges: 19.4 mm and 7.5 mm.
    These are not the full white tile dimensions or a print-scale certificate.
    max_distance_px is measured in source image coordinates, even with detector
    upsampling. Reassociation after occlusion is provisional and reported.
    """
    def __init__(self, max_distance_px=60., max_missed=3, detector_scale=2, size_candidates=(.0075, .0194)):
        if max_distance_px <= 0 or max_missed < 0 or detector_scale < 1:
            raise ValueError('Invalid tracker settings')
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_25h9), parameters)
        self.max_distance_px, self.max_missed = max_distance_px, max_missed
        self.detector_scale = detector_scale
        self.tracks, self.next_track = {}, 0
        self.size_overrides = {}
        self.size_candidates = tuple(size_candidates)
        if not self.size_candidates:
            raise ValueError('At least one size candidate is required')
        for size in self.size_candidates:
            square_points(size)

    def set_size_override(self, track_id, size_m, source):
        square_points(size_m)
        if not source:
            raise ValueError('Manual size requires a provenance source')
        self.size_overrides[int(track_id)] = (float(size_m), str(source))

    def detect(self, rgb):
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY) if rgb.ndim == 3 else rgb
        scaled = cv2.resize(gray, None, fx=self.detector_scale, fy=self.detector_scale,
                            interpolation=cv2.INTER_CUBIC)
        corners, identifiers, _ = self.detector.detectMarkers(scaled)
        return [] if identifiers is None else [
            {'code_id': int(identifier), 'corners_px': (corner.reshape(4, 2)/self.detector_scale).tolist()}
            for corner, identifier in zip(corners, identifiers.ravel())]

    def update(self, detections, intrinsics, depth=None, depth_scale_m=.001, distortion=None):
        centers = np.asarray([np.mean(d['corners_px'], axis=0) for d in detections]).reshape(-1, 2)
        previous = list(self.tracks)
        assigned = {}
        if previous and len(centers):
            costs = np.linalg.norm(np.asarray([self.tracks[t]['center'] for t in previous])[:, None]-centers, axis=2)
            for row, track_id in enumerate(previous):
                for col, detection in enumerate(detections):
                    if self.tracks[track_id]['code_id'] != detection['code_id']:
                        costs[row, col] = 1e9
            rows, cols = linear_sum_assignment(np.where(costs <= self.max_distance_px, costs, 1e9))
            assigned = {int(col): previous[row] for row, col in zip(rows, cols)
                        if costs[row, col] <= self.max_distance_px}
        for track in self.tracks.values():
            track['missed'] += 1
        observations = []
        for index, detection in enumerate(detections):
            track_id = assigned.get(index)
            if track_id is None:
                track_id = self.next_track
                self.next_track += 1
                self.tracks[track_id] = {'center': centers[index], 'code_id': detection['code_id'],
                                         'missed': 1, 'size_m': None}
            track = self.tracks[track_id]
            missed = track['missed']-1
            inferred = {'size_m': None, 'reason': 'depth_unavailable'} if depth is None else depth_size(
                detection['corners_px'], depth, intrinsics, depth_scale_m, self.size_candidates)
            override = self.size_overrides.get(track_id)
            size = override[0] if override else inferred['size_m']
            conflict = bool(not override and size is not None and track['size_m'] is not None
                            and size != track['size_m'])
            if override and inferred.get('edge_lengths_m') is not None:
                conflict = bool(np.max(np.abs(np.asarray(inferred['edge_lengths_m'])-size)/size) > .20)
            if conflict and not override:
                size = None
            if size is not None:
                track['size_m'] = size
            track.update(center=centers[index], missed=0)
            observation = {**detection, 'track_id': track_id, 'center_px': centers[index].tolist(),
                           'reassociated_after_missed_frames': missed, 'association_qualified': False,
                           'size_m': size, 'size_source': override[1] if override else inferred.get('source'),
                           'size_conflict': conflict, 'depth_size': inferred, 'qualified': False,
                           'pose': None}
            if size is not None:
                observation['pose'] = pose_candidates(detection['corners_px'], intrinsics, size,
                    distortion, inferred.get('corner_depth_m'))
            observations.append(observation)
        self.tracks = {t: s for t, s in self.tracks.items() if s['missed'] <= self.max_missed}
        return observations

    def process(self, rgb, intrinsics, depth=None, depth_scale_m=.001, distortion=None):
        if depth is not None and depth.shape != rgb.shape[:2]:
            raise ValueError('Depth must be aligned to the RGB image')
        return self.update(self.detect(rgb), intrinsics, depth, depth_scale_m, distortion)
