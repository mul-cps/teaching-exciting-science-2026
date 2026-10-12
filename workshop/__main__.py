"""Run `python -m workshop` on any laptop, no dependencies or devices."""
import argparse
from .factory import Factory, SimBelt, SimCamera, SimRobot


def main():
    parser = argparse.ArgumentParser(description="CPU-first Candy Factory")
    parser.add_argument("--cycles", type=int, default=5)
    parser.add_argument("--camera", help="Optional USB camera number, video file, or URL; simulation actuators only")
    args = parser.parse_args()
    if not 1 <= args.cycles <= 100:
        parser.error("--cycles must be from 1 to 100")

    camera = SimCamera()
    if args.camera is not None:
        from .vision import OpenCVCamera
        source = int(args.camera) if args.camera.isdigit() else args.camera
        camera = OpenCVCamera(source)
    factory = Factory(SimBelt(), camera, SimRobot())
    try:
        for _ in range(args.cycles):
            factory.one_cycle()
    finally:
        if hasattr(camera, "close"):
            camera.close()
    print("COUNTS:", factory.counts)


if __name__ == "__main__":
    main()
