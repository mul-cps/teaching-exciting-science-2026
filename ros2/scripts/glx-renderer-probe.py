#!/usr/bin/env python3
"""Create a temporary 1x1 GLX context and print its actual renderer string."""

from pathlib import Path
import os
import sys


PACKAGE_SOURCE = Path(__file__).resolve().parents[1] / "src/so101_station_gpu"
if PACKAGE_SOURCE.is_dir():
    sys.path.insert(0, str(PACKAGE_SOURCE))

from so101_station_gpu.gpu_renderer import probe_renderer  # noqa: E402


def main():
    print(f"DISPLAY={os.environ.get('DISPLAY', '')}")
    print(f"LIBGL_ALWAYS_SOFTWARE={os.environ.get('LIBGL_ALWAYS_SOFTWARE', 'unset')}")
    for key, value in probe_renderer().items():
        print(f"{key}={value}")


if __name__ == "__main__":
    main()
