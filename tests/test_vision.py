import pytest
cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")
from workshop.vision import classify_bgr


@pytest.mark.parametrize("bgr,expected", [((0, 0, 255), "red"), ((255, 0, 0), "blue"), ((100, 100, 100), "unknown")])
def test_color(bgr, expected):
    img = np.full((80, 80, 3), bgr, dtype=np.uint8)
    assert classify_bgr(img) == expected


def test_empty_roi_fails_closed():
    with pytest.raises(ValueError):
        classify_bgr(np.zeros((40, 40, 3), dtype=np.uint8), (1, 1, 1, 1))
