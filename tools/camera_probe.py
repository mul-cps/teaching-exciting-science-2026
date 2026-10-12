"""Test any OpenCV capture source (UVC camera, video file, externally exposed stream)."""
import argparse


def main():
    p = argparse.ArgumentParser()
    p.add_argument("source", nargs="?", default="0")
    p.add_argument("--output", default="camera-test.jpg")
    a = p.parse_args()
    import cv2
    source = int(a.source) if a.source.isdigit() else a.source
    capture = cv2.VideoCapture(source)
    if not capture.isOpened():
        raise SystemExit(f"Cannot open {source!r}; see docs/cameras.md")
    try:
        ok, frame = capture.read()
        if not ok or not cv2.imwrite(a.output, frame):
            raise SystemExit("Could not read/save frame")
        print(f"Saved {a.output}; resolution {frame.shape[1]}x{frame.shape[0]}")
    finally:
        capture.release()


if __name__ == "__main__":
    main()
