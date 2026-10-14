"""Offline AprilTag pose from aligned RGB-D samples on its white modules.

The caller supplies the canonical 7x7 tag25h9 bitmap (black border included).
The estimated pose is always provisional: it does not certify a printed tag
scale, camera calibration, or an external frame registration.
"""
import numpy as np


def _intrinsics(intrinsics):
    try:
        fx, fy = float(intrinsics['fx']), float(intrinsics['fy'])
        cx, cy = float(intrinsics['ppx']), float(intrinsics['ppy'])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError('Invalid camera intrinsics') from exc
    values = np.array([fx, fy, cx, cy])
    if not np.isfinite(values).all() or fx <= 0 or fy <= 0:
        raise ValueError('Invalid camera intrinsics')
    _validate_zero_distortion(intrinsics)
    return fx, fy, cx, cy


def _validate_zero_distortion(intrinsics):
    """Reject declared distortion this pinhole-only estimator cannot model."""
    if not hasattr(intrinsics, 'get'):
        raise ValueError('Invalid camera intrinsics')
    model = intrinsics.get('model', intrinsics.get('distortion_model'))
    raw = intrinsics.get('distortion', intrinsics.get('coeffs', intrinsics.get(
        'coefficients', intrinsics.get('distortion_coeffs', intrinsics.get('distortion_coefficients')))))
    if isinstance(raw, dict):
        if model is None:
            model = raw.get('model', raw.get('distortion_model'))
        raw = raw.get('coeffs', raw.get('coefficients', raw.get(
            'distortion_coeffs', raw.get('distortion', raw.get(
                'distortion_coefficients', raw.get('values', raw.get('data')))))))
    if model is not None:
        normalized = str(model).strip().lower().replace('-', '_').replace(' ', '_')
        accepted = {'none', 'no_distortion', 'brown_conrady', 'brownconrady',
                    'plumb_bob', 'opencv', 'rational_polynomial'}
        if normalized not in accepted:
            raise ValueError(f'Unsupported camera distortion model: {model}')
    if raw is None:
        return
    try:
        coeffs = np.asarray(raw, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError('Invalid camera distortion coefficients') from exc
    if coeffs.size and (not np.isfinite(coeffs).all() or np.any(np.abs(coeffs) > 1e-12)):
        raise ValueError('Camera distortion coefficients must all be zero')


def _quad_area_and_validate(corners):
    edges = np.roll(corners, -1, axis=0) - corners
    cross = edges[:, 0] * np.roll(edges, -1, axis=0)[:, 1] - edges[:, 1] * np.roll(edges, -1, axis=0)[:, 0]
    if np.any(np.abs(cross) < 1e-6) or not (np.all(cross > 0) or np.all(cross < 0)):
        raise ValueError('Degenerate, concave, or self-crossing corner quad')
    area = abs(float(np.sum(corners[:, 0] * np.roll(corners[:, 1], -1)
                            - corners[:, 1] * np.roll(corners[:, 0], -1)))) * .5
    if area < 1.:
        raise ValueError('Degenerate corner quad')


def _homography(corners, uv):
    # Homogeneous DLT from unit-square module coordinates to image pixels.
    source = np.asarray(uv, dtype=float)
    dest = np.asarray(corners, dtype=float)
    rows = []
    for (x, y), (u, v) in zip(source, dest):
        rows.extend(([-x, -y, -1, 0, 0, 0, u*x, u*y, u],
                     [0, 0, 0, -x, -y, -1, v*x, v*y, v]))
    _, _, vt = np.linalg.svd(np.asarray(rows))
    h = vt[-1].reshape(3, 3)
    if abs(h[2, 2]) < 1e-12:
        raise ValueError('Degenerate corner homography')
    return h / h[2, 2]


def _project(h, points):
    homogeneous = np.column_stack((points, np.ones(len(points)))) @ h.T
    if np.any(np.abs(homogeneous[:, 2]) < 1e-9):
        raise ValueError('Invalid projected module centers')
    return homogeneous[:, :2] / homogeneous[:, 2, None]


def _backproject(pixels, depths, intrinsics):
    fx, fy, cx, cy = intrinsics
    return np.column_stack(((pixels[:, 0] - cx) * depths / fx,
                            (pixels[:, 1] - cy) * depths / fy, depths))


def _fit_plane(points, threshold_m, max_iterations, seed):
    # Bounded RANSAC on at most 25 module samples; fit/refine uses vectorized SVD.
    rng = np.random.default_rng(seed)
    best = None
    for _ in range(max_iterations):
        sample = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = np.linalg.norm(normal)
        if norm < 1e-10:
            continue
        normal /= norm
        residual = np.abs((points - sample[0]) @ normal)
        mask = residual <= threshold_m
        score = int(mask.sum())
        if best is None or score > int(best.sum()):
            best = mask
    if best is None or int(best.sum()) < 3:
        raise ValueError('Unable to fit a depth plane')
    inliers = points[best]
    for _ in range(3):
        center = inliers.mean(axis=0)
        _, singular, vt = np.linalg.svd(inliers - center, full_matrices=False)
        if len(singular) < 3 or singular[1] < 1e-9:
            raise ValueError('Degenerate depth plane samples')
        normal = vt[-1]
        residual = np.abs((points - center) @ normal)
        new_mask = residual <= threshold_m
        if int(new_mask.sum()) < 3 or np.array_equal(new_mask, best):
            break
        best = new_mask
        inliers = points[best]
    center = inliers.mean(axis=0)
    _, singular, vt = np.linalg.svd(inliers - center, full_matrices=False)
    if len(singular) < 3 or singular[1] < 1e-9:
        raise ValueError('Degenerate depth plane samples')
    normal = vt[-1]
    d = -float(normal @ center)
    residual = np.abs(points @ normal + d)
    mask = residual <= threshold_m
    rms = float(np.sqrt(np.mean(residual[mask] ** 2))) if np.any(mask) else float('inf')
    return normal, d, mask, rms


def estimate_tag_pose(corners_px, rgb, depth, intrinsics, depth_scale_m, tag_bits,
                      *, patch_radius=1, min_white_cells=8, plane_threshold_m=.002,
                      max_plane_rms_m=.001, min_plane_inlier_fraction=.75,
                      max_ransac_iterations=128, seed=17,
                      white_rgb_min=160):
    """Estimate provisional camera-from-tag pose from known white module centers.

    ``corners_px`` are OpenCV TL, TR, BR, BL outer black-edge corners. ``tag_bits``
    is the caller's canonical binary 7x7 pattern, with its black border included.
    Depth is aligned to RGB; scale converts each depth unit to metres. Invalid
    patches, dark RGB patches, too few modules, poor planes, and grazing rays
    raise ``ValueError``. Returned metric edge lengths describe the observed
    black border and do not overwrite caller-declared tag dimensions.
    """
    corners = np.asarray(corners_px, dtype=float)
    bits = np.asarray(tag_bits)
    image = None if rgb is None else np.asarray(rgb)
    z_image = np.asarray(depth)
    if corners.shape != (4, 2) or not np.isfinite(corners).all():
        raise ValueError('Expected four finite ordered marker corners')
    _quad_area_and_validate(corners)
    if bits.shape != (7, 7) or not np.isin(bits, [0, 1]).all():
        raise ValueError('Expected a binary 7x7 tag pattern including black border')
    if np.any(bits[0]) or np.any(bits[-1]) or np.any(bits[:, 0]) or np.any(bits[:, -1]):
        raise ValueError('Expected a black border around the 7x7 tag pattern')
    if z_image.ndim != 2 or (image is not None and
                             (image.ndim != 3 or image.shape[2] < 3 or z_image.shape != image.shape[:2])):
        raise ValueError('Expected depth and optionally aligned RGB arrays')
    if not np.isfinite(depth_scale_m) or depth_scale_m <= 0:
        raise ValueError('Depth scale must be positive and finite')
    if patch_radius < 0 or min_white_cells < 8 or plane_threshold_m <= 0 or max_plane_rms_m <= 0:
        raise ValueError('Invalid sampling or plane-fit settings')
    if not 0 < min_plane_inlier_fraction <= 1 or max_ransac_iterations < 1 or not 0 <= white_rgb_min <= 255:
        raise ValueError('Invalid RANSAC or RGB threshold setting')
    k = _intrinsics(intrinsics)

    cells = np.argwhere(bits[1:6, 1:6] == 1) + 1
    if len(cells) < min_white_cells:
        raise ValueError('Tag pattern has too few white internal modules')
    module_uv = (cells[:, ::-1] + .5) / 7.
    h = _homography(corners, np.array([[0., 0.], [1., 0.], [1., 1.], [0., 1.]]))
    pixels = _project(h, module_uv)
    centers = np.rint(pixels).astype(int)
    unique_centers = np.unique(centers, axis=0)
    if len(unique_centers) < min_white_cells:
        raise ValueError('Too few distinct rounded white-module pixels')
    # Give every image pixel one vote: duplicate module projections share one
    # measured depth and cannot create artificial subpixel plane support.
    pixels = unique_centers.astype(float)
    centers = unique_centers
    radius = int(patch_radius)
    offsets = np.arange(-radius, radius + 1)
    xs = centers[:, 0, None, None] + offsets[None, None, :]
    ys = centers[:, 1, None, None] + offsets[None, :, None]
    valid_bounds = ((xs >= 0) & (xs < z_image.shape[1]) & (ys >= 0) & (ys < z_image.shape[0]))
    xs_safe = np.clip(xs, 0, z_image.shape[1] - 1)
    ys_safe = np.clip(ys, 0, z_image.shape[0] - 1)
    z_samples = z_image[ys_safe, xs_safe].astype(float) * float(depth_scale_m)
    valid = valid_bounds & np.isfinite(z_samples) & (z_samples > 0)
    if image is not None:
        rgb_samples = image[ys_safe, xs_safe, :3].astype(float)
        valid &= np.min(rgb_samples, axis=-1) >= white_rgb_min
    # Each module is one independent sample; within-patch pixels are reduced together.
    sample_count = len(unique_centers)
    masked = np.where(valid, z_samples, np.nan).reshape(sample_count, -1)
    cell_depths = np.full(sample_count, np.nan)
    has_valid = np.any(np.isfinite(masked), axis=1)
    cell_depths[has_valid] = np.nanmedian(masked[has_valid], axis=1)
    good = np.isfinite(cell_depths)
    if int(good.sum()) < min_white_cells:
        raise ValueError('Too few distinct white modules have valid depth samples')
    points = _backproject(pixels[good], cell_depths[good], k)
    normal, d, inlier_mask, rms = _fit_plane(points, plane_threshold_m,
                                              max_ransac_iterations, seed)
    if int(inlier_mask.sum()) < min_white_cells:
        raise ValueError('Too few white module samples agree with one plane')
    if float(inlier_mask.mean()) < min_plane_inlier_fraction:
        raise ValueError('Depth samples support competing planes')
    if rms > max_plane_rms_m:
        raise ValueError('Depth plane RMS exceeds configured limit')

    # Marker axes follow the printed convention: +X right, +Y up, +Z toward camera.
    def intersect(pixel):
        ray = np.array([(pixel[0] - k[2]) / k[0], (pixel[1] - k[3]) / k[1], 1.])
        denominator = float(normal @ ray)
        if abs(denominator) < 1e-6:
            raise ValueError('Corner ray grazes the fitted depth plane')
        scale = -d / denominator
        if not np.isfinite(scale) or scale <= 0:
            raise ValueError('Corner ray intersects behind the camera')
        return ray * scale

    corners_3d = np.array([intersect(pixel) for pixel in corners])
    center_3d = intersect(_project(h, np.array([[.5, .5]]))[0])
    x_axis = corners_3d[1] - corners_3d[0]
    x_axis -= normal * (normal @ x_axis)
    x_norm = np.linalg.norm(x_axis)
    if x_norm < 1e-8:
        raise ValueError('Degenerate tag X axis')
    x_axis /= x_norm
    y_axis = corners_3d[0] - corners_3d[3]
    y_axis -= normal * (normal @ y_axis) + x_axis * (x_axis @ y_axis)
    y_norm = np.linalg.norm(y_axis)
    if y_norm < 1e-8:
        raise ValueError('Degenerate tag Y axis')
    y_axis /= y_norm
    z_axis = np.cross(x_axis, y_axis)
    z_axis /= np.linalg.norm(z_axis)
    if z_axis @ center_3d > 0:
        y_axis = -y_axis
        z_axis = np.cross(x_axis, y_axis)
        z_axis /= np.linalg.norm(z_axis)
    rotation = np.column_stack((x_axis, y_axis, z_axis))
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = center_3d
    side_lengths = np.linalg.norm(np.roll(corners_3d, -1, axis=0) - corners_3d, axis=1)
    # This is only an algebraic ray/intersection round trip: each 3D corner was
    # constructed on its input pixel ray, so it is not an independent pose-fit score.
    projected_corners = np.column_stack((corners_3d[:, 0] * k[0] / corners_3d[:, 2] + k[2],
                                         corners_3d[:, 1] * k[1] / corners_3d[:, 2] + k[3]))
    roundtrip_error = float(np.sqrt(np.mean(np.sum((projected_corners - corners) ** 2, axis=1))))
    return {
        'status': 'OBSERVED_DEPTH_PROVISIONAL', 'qualified': False,
        'camera_from_tag': transform.tolist(),
        'black_edge_side_lengths_m': side_lengths.tolist(),
        'plane': {'normal_camera': normal.tolist(), 'd_m': float(d),
                  'rms_m': rms, 'inlier_count': int(inlier_mask.sum()),
                  'sample_count': int(len(points))},
        'ray_intersection_roundtrip_error_px': roundtrip_error,
        'module_count': int(len(cells)), 'valid_module_count': int(good.sum()),
        'unique_sample_pixel_count': int(len(unique_centers)),
    }
