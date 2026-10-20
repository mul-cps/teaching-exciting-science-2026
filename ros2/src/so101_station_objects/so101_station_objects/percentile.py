"""Small finite-array linear percentiles for component geometry metrics."""
import numpy as np


def linear_percentile(values, q, axis=None):
    """Match NumPy linear interpolation; delegate other array cases to NumPy.

    Component metrics use finite float64 or uint8 vectors and Nx3
    arrays with axis 0. Keep the general implementation for nonfinite inputs,
    other axes/shapes/dtypes, including float32 data or quantile weights.
    """
    if (type(values) is not np.ndarray
            or values.dtype not in (np.dtype("float64"), np.dtype("uint8"))
            or values.ndim not in (1, 2)
            or (values.ndim == 2 and values.shape[1] != 3)
            or (axis is not None and (type(axis) is not int or axis != 0))
            or not values.size or not np.isfinite(values).all()):
        return np.percentile(values, q, axis=axis)
    quantiles = np.asarray(q)
    if (quantiles.ndim > 1 or (quantiles.dtype.kind not in "iu"
                and quantiles.dtype != np.dtype("float64"))):
        return np.percentile(values, q, axis=axis)
    quantiles = quantiles.astype(np.float64, copy=False)
    if (not np.isfinite(quantiles).all()
            or (quantiles < 0).any() or (quantiles > 100).any()):
        return np.percentile(values, q, axis=axis)

    data = values.reshape(-1) if axis is None else values
    count = data.shape[0]
    indices = (count - 1) * np.true_divide(quantiles, 100)
    if data.ndim == 1 and quantiles.ndim == 0:
        lower = int(indices)
        upper = min(lower + 1, count - 1)
        weight = indices - lower
        partitioned = np.partition(data, (lower, upper))
        low, high = partitioned[lower], partitioned[upper]
        difference = high - low
        # NumPy switches arithmetic at 0.5 to retain its rounding behavior.
        return (high - difference * (1 - weight) if weight >= .5
                else low + difference * weight)

    lower = np.floor(indices).astype(np.intp)
    upper = np.minimum(lower + 1, count - 1)
    partitioned = np.partition(
        data, np.concatenate((lower.reshape(-1), upper.reshape(-1))), axis=0)
    low, high = partitioned[lower], partitioned[upper]
    weight = indices - lower
    if data.ndim == 2:
        weight = weight[..., None]
    difference = high - low
    result = np.add(low, difference * weight)
    np.subtract(high, difference * (1 - weight), out=result,
                where=weight >= .5, casting="unsafe", dtype=result.dtype)
    return result[()] if result.ndim == 0 else result
