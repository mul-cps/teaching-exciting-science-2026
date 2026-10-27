"""Freeze the retained full geometry report and dense evidence. Offline only.

This tool deliberately uses retained NumPy/OpenCV geometry as a verification
oracle. It is never imported by the resident production component. Arrays are
stored losslessly, ordered contours retain OpenCV order, and every file is
hashed. No camera, ROS, GPU or private calibration is required.
"""
import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'ros2/src/so101_station_objects')]
from so101_station_objects import objects
from tools.propose_rgbd_grasp import color_masks

SOURCES = [
    'ros2/src/so101_station_objects/so101_station_objects/objects.py',
    'ros2/src/so101_station_objects/so101_station_objects/colored_object_validation.py',
    'ros2/src/so101_station_objects/so101_station_objects/percentile.py',
    'ros2/src/so101_station_objects/so101_station_objects/color_mask_backend.py',
    'workshop/digital_twin/grasp_geometry.py',
    'workshop/digital_twin/principal_axes.py',
    'tools/propose_rgbd_grasp.py',
    'tools/native_geometry_oracle.py',
]


ORACLE_SCHEMA = 'so101-native-full-geometry-oracle-v2'
LEGACY_ORACLE_SCHEMA = 'so101-native-full-geometry-oracle-v1'
REFERENCE_SCHEMA = 'so101-native-geometry-procedural-reference-v2'


def principal_axes_policy():
    """Explicit direction/ranking policy; not a universal LAPACK error bound."""
    return dict(
        name='canonical-principal-axes-v1',
        spectrum_cluster=dict(eps64_multiplier=8,
            scale='max(sample_count, dimension) * max(abs(spectrum))',
            spread='complete cluster spread; no adjacent-gap chaining',
            units=dict(svd='singular values', covariance_eigh='eigenvalues')),
        projected_anchor_tie=dict(eps64_multiplier=64,
            units='absolute squared norm', coordinate_order=['X', 'Y', 'Z']),
        grasp_width_ranking_tie=dict(eps64_multiplier=32,
            scale='max(abs(widths))', units='opening width',
            application='after exact opening <= maxopening_m filter; retain first candidate'),
        universal_cross_platform_bit_identity=False)


def analyze_reference(arguments):
    """Force the offline CPU oracle regardless of future station defaults."""
    options = dict(arguments)
    options['color_mask_backend'] = objects.ColorMaskBackend('cpu')
    return objects.analyze_rgbd(**options)


def procedural_scene(width=128, height=96):
    """Public deterministic fixture, with observed support and one red brick."""
    rgb = np.full((height, width, 3), 128, dtype=np.uint8)
    depth = np.full((height, width), 800, dtype=np.uint16)
    y, x = height//2, width//2
    rgb[y-6:y+6, x-6:x+6] = [255, 0, 0]
    depth[y-6:y+6, x-6:x+6] = 780
    transform = np.diag([1., -1., -1., 1.]); transform[2, 3] = .8
    return dict(rgb=rgb, depth=depth,
                intrinsics=dict(fx=300., fy=300., ppx=float(x), ppy=float(y)),
                distortion=np.zeros(5), depth_scale_m=.001,
                board_from_camera=transform)


def trace_intermediates(arguments):
    """Retain original memberships, exact ordered contours and both ray paths."""
    a = arguments
    rgb = a['rgb']; depth = a['depth'].astype(np.float32)*float(a['depth_scale_m'])
    limit = a.get('max_observation_depth_m', objects.DEFAULT_MAX_OBSERVATION_DEPTH_M)
    valid = np.isfinite(depth) & (depth >= objects.MIN_OBSERVATION_DEPTH_M) & (depth <= limit)
    masks = color_masks(rgb)
    vivid = np.logical_or.reduce(tuple(masks.values()))
    excluded = cv2.dilate(vivid.astype(np.uint8), np.ones((7, 7), np.uint8)) > 0
    yy, xx = np.mgrid[0:depth.shape[0]:8, 0:depth.shape[1]:8]
    selected = valid[yy, xx] & ~excluded[yy, xx]
    pixels = np.column_stack((xx[selected], yy[selected]))
    camera = objects.camera_points(pixels, depth[yy[selected], xx[selected]], a['intrinsics'], a['distortion'])
    board = objects.transform_points(camera, a['board_from_camera'])
    roi = a.get('support_roi', a.get('world_roi', objects.DEFAULT_TABLE_ROI))
    inside = objects._inside_roi(board, roi)
    board_inside = board[inside]
    indices = (np.random.default_rng(17).choice(len(board_inside), 2000, replace=False)
               if len(board_inside) > 2000 else np.arange(len(board_inside)))
    support = dict(sample_pixels=pixels, sample_camera=camera, sample_board=board,
                   inside=inside, inside_board=board_inside, selected_indices=indices,
                   selected_board=board_inside[indices])
    result = dict(depth_m=depth, valid_depth=valid, masks=masks, vivid=vivid,
                  support_excluded=excluded, support=support, parents=[], pieces=[])
    for color in objects.COLOR_NAMES:
        count, labels, stats, _ = cv2.connectedComponentsWithStatsWithAlgorithm(
            masks[color].astype(np.uint8), 8, cv2.CV_32S, cv2.CCL_GRANA)
        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < a.get('min_component_pixels', 30):
                continue
            x, y, w, h = (int(stats[label, k]) for k in (cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP,
                                                       cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT))
            crop = np.s_[y:y+h, x:x+w]
            parent = labels[crop] == label
            result['parents'].append(dict(color=color, label=label, origin=[x, y], mask=parent,
                                          valid=valid[crop] & parent))
            pieces = objects.split_depth_coherent_components(
                parent, depth[crop], min_pixels=max(24, a.get('min_component_pixels', 30)),
                max_observation_depth_m=limit, local_output=True, pixel_origin=(x, y))
            for piece in pieces:
                ox, oy = piece.origin_xy
                sy, sx = np.nonzero(piece.mask)
                global_pixels = np.column_stack((sx+ox, sy+oy))
                z = depth[sy+oy, sx+ox]
                local_k = dict(a['intrinsics'], ppx=a['intrinsics']['ppx']-ox,
                               ppy=a['intrinsics']['ppy']-oy)
                global_camera = objects.camera_points(global_pixels, z, a['intrinsics'], a['distortion'])
                local_camera = objects.camera_points(np.column_stack((sx, sy)), z, local_k, a['distortion'])
                piece_crop = np.s_[oy:oy+piece.mask.shape[0], ox:ox+piece.mask.shape[1]]
                validation = objects.validate_colored_component(
                    rgb[piece_crop], depth[piece_crop], piece.mask, local_k, a['distortion'],
                    support_mask=piece.support_mask, detailed_rejection_diagnostics=True,
                    max_observation_depth_m=limit)
                closed = cv2.morphologyEx(piece.mask.astype(np.uint8), cv2.MORPH_CLOSE,
                                          np.ones((3, 3), np.uint8))
                contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                contours = objects._split_elongated_contours(contours)
                children = []
                for contour in contours:
                    filled = np.zeros_like(closed)
                    cv2.drawContours(filled, [contour], -1, 1, thickness=cv2.FILLED)
                    silhouette = objects.validate_brick_silhouette(
                        contour, observed_mask=piece.mask & (filled != 0), min_rect_fill=.45)
                    children.append(dict(silhouette=dict(accepted=silhouette.accepted, reason=silhouette.reason, metrics=silhouette.metrics),
                                         vertices=contour.copy(), fill=filled,
                                         original_membership=(filled != 0) & piece.mask,
                                         selected=(filled[sy, sx] != 0)))
                result['pieces'].append(dict(color=color, origin=[ox, oy], mask=piece.mask,
                    support_mask=piece.support_mask,
                    depth_validation=dict(accepted=validation.accepted, reason=validation.reason, metrics=validation.metrics),
                    parent_pixels=int(piece.parent_pixels),
                    depth_cut_pixels=int(piece.depth_cut_pixels), closed=closed,
                    global_pixels=global_pixels, camera_global=global_camera, camera_local=local_camera,
                    board=objects.transform_points(global_camera, a['board_from_camera']), contours=children))
    return result


def assert_report_equal(expected, actual, path='report', tolerance=1e-9):
    """Exact schema/order/reasons/discrete values; existing numeric bound only."""
    if isinstance(expected, np.ndarray):
        actual = np.asarray(actual)
        assert actual.shape == expected.shape, path
        if expected.dtype.kind in 'biu':
            assert actual.dtype.kind == expected.dtype.kind, path
            np.testing.assert_array_equal(actual, expected, err_msg=path)
        else:
            np.testing.assert_allclose(actual, expected, rtol=0, atol=tolerance, err_msg=path)
    elif isinstance(expected, dict):
        assert isinstance(actual, dict) and expected.keys() == actual.keys(), path
        for key in expected:
            assert_report_equal(expected[key], actual[key], path+'.'+str(key), tolerance)
    elif isinstance(expected, (list, tuple)):
        assert isinstance(actual, (list, tuple)) and len(expected) == len(actual), path
        for i, (left, right) in enumerate(zip(expected, actual)):
            assert_report_equal(left, right, path+f'[{i}]', tolerance)
    elif isinstance(expected, (str, bool, int, type(None))):
        assert type(expected) is type(actual) and expected == actual, (path, expected, actual)
    else:
        np.testing.assert_allclose(actual, expected, rtol=0, atol=tolerance, err_msg=path)


def freeze(arguments, directory):
    directory = Path(directory); directory.mkdir(parents=True, exist_ok=False)
    arrays = {}
    def encode(value):
        if isinstance(value, np.ndarray):
            name = f'array-{len(arrays):05d}.npy'
            np.save(directory/name, value, allow_pickle=False)
            arrays[name] = dict(sha256=hashlib.sha256((directory/name).read_bytes()).hexdigest(),
                                shape=list(value.shape), dtype=str(value.dtype))
            return name
        if isinstance(value, dict):
            return {str(k): encode(v) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [encode(v) for v in value]
        if isinstance(value, np.generic):
            return value.item()
        return value
    report = analyze_reference(arguments)
    # Reports contain arrays too; their references remain lossless in manifest.
    manifest = dict(schema=ORACLE_SCHEMA,
        provenance=dict(numeric_absolute_tolerance=1e-9, local_ray_absolute_tolerance=1e-12,
            principal_axes_policy=principal_axes_policy(),
            python=platform.python_version(), numpy=np.__version__, opencv=cv2.__version__,
            scipy=importlib.metadata.version('scipy'),
            sources={p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in SOURCES},
            scope='offline retained oracle; no native or physical qualification'),
        inputs=encode(arguments), report=encode(report), trace=encode(trace_intermediates(arguments)),
        preparation=encode(preparation_reference(arguments)),
        arrays=arrays)
    (directory/'manifest.json').write_text(json.dumps(manifest, indent=2, allow_nan=False)+'\n')
    return manifest


def load_verified(directory):
    directory = Path(directory)
    manifest = json.loads((directory/'manifest.json').read_text())
    if manifest['schema'] not in (LEGACY_ORACLE_SCHEMA, ORACLE_SCHEMA):
        raise ValueError('unsupported geometry oracle schema')
    if manifest['schema'] == ORACLE_SCHEMA:
        if json.dumps(manifest['provenance'].get('principal_axes_policy'), sort_keys=True) != json.dumps(principal_axes_policy(), sort_keys=True):
            raise ValueError('unsupported principal axes policy')
        if 'workshop/digital_twin/principal_axes.py' not in manifest['provenance']['sources']:
            raise ValueError('missing principal axes source binding')
    arrays = {}
    for name, meta in manifest['arrays'].items():
        if Path(name).name != name or not name.endswith('.npy'):
            raise ValueError('invalid array path')
        data = (directory/name).read_bytes()
        if hashlib.sha256(data).hexdigest() != meta['sha256']:
            raise ValueError('array digest mismatch: '+name)
        value = np.load(directory/name, allow_pickle=False)
        if list(value.shape) != meta['shape'] or str(value.dtype) != meta['dtype']:
            raise ValueError('array metadata mismatch: '+name)
        arrays[name] = dict(value=value, **meta)
    return manifest, arrays


def plain_values(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {k: plain_values(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain_values(v) for v in value]
    return value


def reference_case(arguments):
    """Small versioned full-report fixture plus exact original child evidence."""
    trace = trace_intermediates(arguments)
    pieces = []
    def digest(array):
        return dict(shape=list(array.shape), dtype=str(array.dtype),
                    sha256=hashlib.sha256(array.tobytes(order='C')).hexdigest())
    for piece in trace['pieces']:
        pieces.append(dict(color=piece['color'], origin=piece['origin'], mask=digest(piece['mask']),
            support_mask=digest(piece['support_mask']),
            camera_global=plain_values(piece['camera_global']), camera_local=plain_values(piece['camera_local']),
            contours=[dict(vertices=plain_values(c['vertices']), original_membership=digest(c['original_membership']),
                           selected=digest(c['selected'])) for c in piece['contours']]))
    return dict(schema=REFERENCE_SCHEMA,
                principal_axes_policy=principal_axes_policy(),
                sources={p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in SOURCES},
                numeric_absolute_tolerance=1e-9, local_ray_absolute_tolerance=1e-12,
                report=plain_values(analyze_reference(arguments)), pieces=pieces)


def load_saved_capture(directory):
    """Read an already recorded frame. Never open a camera or rewrite evidence."""
    from scipy.spatial.transform import Rotation
    directory = Path(directory)
    metadata = json.loads((directory/'capture.json').read_text())
    camera = metadata.get('camera', metadata.get('camera_info'))
    if camera is None:
        camera = metadata.get('messages', {}).get('aligned_info', {}).get('message')
    if camera is None:
        raise ValueError('saved capture lacks aligned camera intrinsics')
    matrix = np.asarray(camera['k']).reshape(3, 3)
    depth_path = next((directory/name for name in ('depth.npy', 'aligned_depth-raw.npy',
                      'aligned-depth-raw.npy') if (directory/name).is_file()), None)
    if depth_path is None:
        raise ValueError('saved aligned depth unavailable')
    if 'world_from_camera' in metadata:
        transform = np.asarray(metadata['world_from_camera'], dtype=np.float64)
    else:
        pose = metadata.get('transforms', {}).get('charuco_board', {}).get('transform')
        if pose is None:
            raise ValueError('saved capture lacks board/world transform; full geometry freeze unavailable')
        camera_from_board = np.eye(4)
        camera_from_board[:3, :3] = Rotation.from_quat([pose['rotation'][k] for k in 'xyzw']).as_matrix()
        camera_from_board[:3, 3] = [pose['translation'][k] for k in 'xyz']
        transform = np.linalg.inv(camera_from_board)
    return dict(rgb=cv2.cvtColor(cv2.imread(str(directory/'color.png')), cv2.COLOR_BGR2RGB),
                depth=np.load(depth_path, allow_pickle=False),
                intrinsics=dict(fx=matrix[0, 0], fy=matrix[1, 1], ppx=matrix[0, 2], ppy=matrix[1, 2]),
                distortion=np.asarray(camera['d']), depth_scale_m=metadata.get('depth_scale_m', .001),
                board_from_camera=transform)


def preparation_reference(arguments):
    """Offline dense FP64 firstgate oracle and calibration-generation ray LUT."""
    a = arguments; h, w = a['depth'].shape
    yy, xx = np.mgrid[:h, :w]
    k = a['intrinsics']
    matrix = np.array([[k['fx'], 0, k['ppx']], [0, k['fy'], k['ppy']], [0, 0, 1]], dtype=np.float64)
    rays = cv2.undistortPoints(np.column_stack((xx.ravel(), yy.ravel())).astype(np.float64).reshape(-1, 1, 2),
                               matrix, np.asarray(a['distortion'])).reshape(h, w, 2)
    depth = a['depth'].astype(np.float32)*float(a['depth_scale_m'])
    camera = np.concatenate((rays*depth[:, :, None], depth[:, :, None]), axis=2)
    board = camera@a['board_from_camera'][:3, :3].T+a['board_from_camera'][:3, 3]
    valid = np.isfinite(depth)&(depth >= objects.MIN_OBSERVATION_DEPTH_M)&(depth <= a.get('max_observation_depth_m', 1.2))
    vivid = np.logical_or.reduce(tuple(color_masks(a['rgb']).values()))
    excluded = cv2.dilate(vivid.astype(np.uint8), np.ones((7, 7), np.uint8)) != 0
    roi = a.get('support_roi', a.get('world_roi', objects.DEFAULT_TABLE_ROI))
    inside = objects._inside_roi(board.reshape(-1, 3), roi).reshape(h, w)
    flags = valid.astype(np.uint32)|(excluded.astype(np.uint32)<<1)|(inside.astype(np.uint32)<<2)
    sy, sx = np.mgrid[0:h:8, 0:w:8]
    accepted = flags[sy, sx] == 5
    return dict(rays=rays, board=board, flags=flags, valid_count=int(valid.sum()),
                selected_board=board[sy[accepted], sx[accepted]],
                selected_pixels=(sy[accepted]*w+sx[accepted]).astype(np.uint32))


def restore_arrays(value, arrays):
    """Resolve manifest references before complete published-report comparison."""
    if isinstance(value, str) and value in arrays:
        return arrays[value]['value']
    if isinstance(value, dict):
        return {k: restore_arrays(v, arrays) for k, v in value.items()}
    if isinstance(value, list):
        return [restore_arrays(v, arrays) for v in value]
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--capture', type=Path, help='Optional existing saved aligned RGB-D capture; no device access')
    args = parser.parse_args()
    arguments = procedural_scene()
    if args.capture:
        arguments = load_saved_capture(args.capture)
    result = freeze(arguments, args.output)
    print(json.dumps(dict(output=str(args.output), arrays=len(result['arrays']),
                         objects=len(result['report']['objects']), schema=result['schema'])))


if __name__ == '__main__':
    main()
