"""CPU-only classification in a fixed pickup ROI; no machine learning required."""


def classify_bgr(frame, roi=(0.35, 0.35, 0.65, 0.65), min_fraction=0.08):
    import cv2
    import numpy as np

    height, width = frame.shape[:2]
    x1, y1, x2, y2 = roi
    patch = frame[int(y1 * height):int(y2 * height), int(x1 * width):int(x2 * width)]
    if patch.size == 0:
        raise ValueError("Pickup ROI is empty; check camera framing")
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    # Red crosses hue=0. Saturation thresholds suppress grey belt/background.
    red = cv2.inRange(hsv, (0, 95, 65), (10, 255, 255)) | cv2.inRange(hsv, (170, 95, 65), (179, 255, 255))
    blue = cv2.inRange(hsv, (95, 95, 65), (135, 255, 255))
    fractions = {"red": float(np.count_nonzero(red)) / red.size, "blue": float(np.count_nonzero(blue)) / blue.size}
    color = max(fractions, key=fractions.get)
    return color if fractions[color] >= min_fraction and fractions[color] > 1.5 * fractions["blue" if color == "red" else "red"] else "unknown"


class OpenCVCamera:
    """Reads ONE image only after the conveyor is stopped."""

    def __init__(self, source=0):
        import cv2
        backend = cv2.CAP_FFMPEG if isinstance(source, str) and source.startswith(("udp://", "rtsp://")) else cv2.CAP_ANY
        self.capture = cv2.VideoCapture(source, backend)
        if not self.capture.isOpened():
            raise RuntimeError(f"Unable to open camera source {source!r}")

    def detect(self):
        # Drop a few stale buffered frames while belt is stationary.
        for _ in range(4):
            ok, frame = self.capture.read()
            if not ok:
                raise RuntimeError("Camera frame missing; belt must stay stopped")
        return classify_bgr(frame)

    def close(self):
        self.capture.release()
