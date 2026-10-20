#!/usr/bin/env python3
"""Run the installed read-only alignment node on an explicit nonzero ROS domain."""
import argparse
import os
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bindings-file", type=Path, required=True)
    parser.add_argument("--domain", type=int, default=76)
    arguments = parser.parse_args()
    if not 1 <= arguments.domain <= 232:
        parser.error("ROS domain must be in 1..232; domain 0 is forbidden")
    bindings = arguments.bindings_file.expanduser().resolve(strict=True)
    environment = dict(os.environ, ROS_DOMAIN_ID=str(arguments.domain), ROS_LOCALHOST_ONLY="1",
                       OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1")
    os.execvpe("ros2", ["ros2", "launch", "so101_station_perception", "live_tag_alignment.launch.py",
                        "bindings_file:="+str(bindings)], environment)


if __name__ == "__main__":
    main()
