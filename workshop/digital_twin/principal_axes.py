"""Coordinate-frame directions for numerically unresolved principal subspaces.

This explicit tie policy is not a universal LAPACK error bound. Spectra retain
caller units (singular values or covariance eigenvalues). Work is bounded by
coordinate dimension, never by the point population.
"""
import numpy as np


def canonical_principal_axes(spectrum, axes, *, sample_count):
    """Return canonical orthonormal rows and a mask of tied directions.

    Inputs are a finite descending spectrum and orthonormal rows in dimension
    two or three. Singleton rows, including their existing signs, are copied
    without arithmetic. Callers retain their historical singleton sign policy.
    """
    values = np.asarray(spectrum, dtype=np.float64)
    rows = np.asarray(axes, dtype=np.float64)
    dimension = len(values)
    if (values.shape != (dimension,) or dimension not in (2, 3)
            or rows.shape != (dimension, dimension)
            or not np.isfinite(values).all() or not np.isfinite(rows).all()
            or np.any(values[1:] > values[:-1])
            or isinstance(sample_count, (bool, np.bool_))
            or not isinstance(sample_count, (int, np.integer)) or sample_count <= 0):
        raise ValueError('finite descending 2D/3D principal axes and positive sample count required')
    epsilon = np.finfo(np.float64).eps
    tolerance = 8 * epsilon * max(sample_count, dimension) * float(np.max(np.abs(values)))
    result = rows.copy()
    tied = np.zeros(dimension, dtype=bool)
    start = 0
    while start < dimension:
        end = start + 1
        # Compare complete cluster spread, never chain adjacent small gaps.
        while end < dimension and values[start] - values[end] <= tolerance:
            end += 1
        if end - start > 1:
            tied[start:end] = True
            if end - start == dimension:
                return np.eye(dimension), tied
            # The only proper tied subspace is a pair in 3D. Its untied
            # complement avoids dependence on arbitrary tied solver rows.
            untied = rows[2 if start == 0 else 0]
            projector = np.eye(dimension) - np.outer(untied, untied)
            for index in range(start, end):
                projected = projector.copy()
                for previous in range(start, index):
                    direction = result[previous]
                    projected -= np.outer(direction, direction @ projected)
                norms = np.sum(projected * projected, axis=0)
                maximum = float(np.max(norms))
                anchor = int(np.flatnonzero(maximum - norms <= 64 * epsilon)[0])
                direction = projected[:, anchor]
                direction = direction / np.sqrt(float(direction @ direction))
                if direction[anchor] < 0:
                    direction = -direction
                result[index] = direction
        start = end
    return result, tied
