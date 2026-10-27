"""Observed RGB-D geometry; candidates are not qualified grasp predictions."""
import numpy as np

from workshop.digital_twin.principal_axes import canonical_principal_axes


def fuse_depth(stack, roi, min_valid_fraction=.75):
    a = np.asarray(stack)
    if a.dtype != np.uint16 or a.ndim != 3 or not len(a):
        raise ValueError('Expected nonempty uint16 NxHxW depth stack')
    x0, y0, x1, y1 = roi
    if not (0 <= x0 < x1 <= a.shape[2] and 0 <= y0 < y1 <= a.shape[1]):
        raise ValueError('Invalid ROI')
    if not 0 < min_valid_fraction <= 1:
        raise ValueError('Invalid valid fraction')
    a = a[:, y0:y1, x0:x1]
    count = (a != 0).sum(axis=0)
    ordered = np.sort(np.where(a == 0, 65535, a), axis=0)
    lo = np.maximum(count - 1, 0) // 2
    hi = count // 2
    median = (np.take_along_axis(ordered, lo[None], axis=0)[0].astype(float)
              + np.take_along_axis(ordered, hi[None], axis=0)[0]) / 2
    median[count == 0] = 0
    deviations = np.sort(np.where(a != 0, np.abs(a.astype(float) - median), np.inf), axis=0)
    mad = (np.take_along_axis(deviations, lo[None], axis=0)[0]
           + np.take_along_axis(deviations, hi[None], axis=0)[0]) / 2
    mad[count == 0] = 0
    return dict(median=median, mad=mad, count=count,
                stable_mask=count >= np.ceil(len(a) * min_valid_fraction))


def _weighted_plane(points, weights):
    center = np.average(points, axis=0, weights=weights)
    _, singular, vt = np.linalg.svd((points - center) * np.sqrt(weights[:, None]), full_matrices=False)
    if len(singular) < 3 or singular[1] < 1e-9:
        raise ValueError('Degenerate plane points')
    normal = vt[-1]
    d = -normal @ center
    if d < 0:
        normal, d = -normal, -d
    return normal, float(d)


def fit_support_plane(points, weights=None, threshold_m=.002, seed=17):
    p = np.asarray(points, dtype=float)
    if p.ndim != 2 or p.shape[1] != 3 or len(p) < 3 or not np.isfinite(p).all():
        raise ValueError('Expected finite Nx3 points')
    w = np.ones(len(p)) if weights is None else np.asarray(weights, dtype=float)
    if w.shape != (len(p),) or not np.isfinite(w).all() or np.any(w < 0) or (w > 0).sum() < 3 or threshold_m <= 0:
        raise ValueError('Invalid weights or threshold')
    rng = np.random.default_rng(seed)
    best, best_score = None, -1
    valid = np.flatnonzero(w > 0)
    for _ in range(120):
        sample = p[rng.choice(valid, 3, replace=False)]
        n = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = np.linalg.norm(n)
        if norm < 1e-12:
            continue
        n /= norm
        mask = np.abs(p @ n - sample[0] @ n) <= threshold_m
        score = w[mask].sum()
        if score > best_score:
            best, best_score = mask, score
    if best is None or (best & (w > 0)).sum() < 3:
        raise ValueError('No nondegenerate support plane')
    q, base_w = p[best], w[best]
    n, d = _weighted_plane(q, base_w)
    for _ in range(8):
        residual = np.abs(q @ n + d)
        huber = np.minimum(1., threshold_m / np.maximum(residual, 1e-12))
        n, d = _weighted_plane(q, base_w * huber)
    residual = np.abs(p @ n + d)
    inliers = residual <= threshold_m
    return dict(normal=n, d=d, inlier_mask=inliers,
                median_m=float(np.median(residual[inliers])),
                p95_m=float(np.percentile(residual[inliers], 95)),
                inlier_p95_m=float(np.percentile(residual[inliers], 95)),
                residuals_m=residual, all_p95_m=float(np.percentile(residual, 95)))


def camera_ray_plane(pixel, intrinsics, normal, d, height_m=0.):
    n = np.asarray(normal, dtype=float)
    if n.shape != (3,) or not np.isfinite(n).all() or not np.isclose(np.linalg.norm(n), 1.):
        raise ValueError('Expected unit plane normal')
    fx, fy = intrinsics['fx'], intrinsics['fy']
    if fx <= 0 or fy <= 0:
        raise ValueError('Invalid focal length')
    u, v = pixel
    ray = np.array([(u - intrinsics['ppx']) / fx, (v - intrinsics['ppy']) / fy, 1.])
    denominator = n @ ray
    if abs(denominator) < 1e-9:
        raise ValueError('Parallel ray')
    distance = (height_m - d) / denominator
    if not np.isfinite(distance) or distance <= 0:
        raise ValueError('Intersection behind camera')
    return distance * ray


def footprint_grasps(contourpixels, plane, intrinsics, height_m, maxopening_m, *, contour_height_m=None):
    pixels = np.asarray(contourpixels, dtype=float)
    if pixels.ndim != 2 or pixels.shape[1] != 2 or len(pixels) < 3 or height_m <= 0 or maxopening_m <= 0:
        raise ValueError('Invalid contour, height or opening')
    n, d = np.asarray(plane['normal']), plane['d']
    if contour_height_m is not None and (not np.isfinite(contour_height_m) or not 0 <= contour_height_m <= height_m):
        raise ValueError('Contour height must lie within object height')
    projection_height = 0. if contour_height_m is None else contour_height_m
    points = np.array([camera_ray_plane(px, intrinsics, n, d, projection_height) for px in pixels])
    origin = points.mean(axis=0)
    _, s, vt = np.linalg.svd(points - origin, full_matrices=False)
    if s[1] < 1e-9:
        raise ValueError('Degenerate silhouette')
    vt, _ = canonical_principal_axes(s, vt, sample_count=len(points))
    x = vt[0] - n * (vt[0] @ n)
    x /= np.linalg.norm(x)
    y = np.cross(n, x)
    basis = np.column_stack((x, y))
    coordinates = (points - origin) @ basis
    lower, upper = coordinates.min(axis=0), coordinates.max(axis=0)
    center = origin + basis @ ((lower + upper) / 2) + n * (height_m / 2 - projection_height)
    result = []
    for axis in range(2):
        opening = float(upper[axis] - lower[axis])
        if opening <= maxopening_m:
            direction = basis[:, axis]
            contacts = np.array([center - direction * opening / 2, center + direction * opening / 2])
            result.append(dict(center=center.copy(), contacts=contacts, closing_axis=direction.copy(),
                               opening_m=opening, approach_direction=-n.copy(),
                               contact_above_support=bool(np.all(contacts @ n + d > 0)),
                               finger_clearance_verified=False, executable=False,
                               projection_assumption=('support-plane approximation' if contour_height_m is None else 'provided contour height'),
                               qualification='observed_silhouette_unqualified'))
    return result
