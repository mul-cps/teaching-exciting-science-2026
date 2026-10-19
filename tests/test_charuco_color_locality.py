"""Guard the board color hot path without changing geometric fit semantics."""

import importlib.util
from pathlib import Path
import numpy as np
import pytest

PATH = (
    Path(__file__).resolve().parents[1]
    / "ros2/src/so101_station_perception/so101_station_perception/charuco_plane.py"
)
spec = importlib.util.spec_from_file_location("plane_locality_under_test", PATH)
plane = importlib.util.module_from_spec(spec)
spec.loader.exec_module(plane)


def test_color_reductions_only_visit_selected_board_samples(monkeypatch):
    seen = []

    class TracedColor(np.ndarray):
        def max(self, *args, **kwargs):
            seen.append(self.shape)
            return super().max(*args, **kwargs)

        def min(self, *args, **kwargs):
            seen.append(self.shape)
            return super().min(*args, **kwargs)

    color = np.full((1080, 1920, 3), 200, dtype=np.uint8)
    original = np.asarray
    monkeypatch.setattr(
        plane.np,
        "asarray",
        lambda value, *a, **kw: (
            value.view(TracedColor) if value is color else original(value, *a, **kw)
        ),
    )
    k = np.array([[1000.0, 0, 960.0], [0, 1000.0, 540.0], [0, 0, 1.0]])
    pose = np.eye(4)
    pose[:3, 3] = [-0.0875, -0.0625, 0.65]
    _, _, metrics = plane.fit_depth_plane(
        np.full((1080, 1920), 0.65, np.float32), k, np.zeros(5), pose, color_image=color
    )
    assert metrics["plane_inliers"] >= 80
    assert seen
    assert all(np.prod(shape[:-1]) <= 8000 for shape in seen)


@pytest.mark.parametrize(
    "shape,dtype",
    [((1080, 1920), np.uint8), ((1080, 1920, 3), np.float32), ((1, 1, 3), np.uint8)],
)
def test_color_validation_unchanged(shape, dtype):
    with pytest.raises(ValueError, match="color image must be uint8"):
        plane.fit_depth_plane(
            np.ones((1080, 1920), np.float32),
            np.eye(3),
            np.zeros(5),
            np.eye(4),
            color_image=np.zeros(shape, dtype),
        )
