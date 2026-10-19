#!/usr/bin/env python3
"""Hash/ELF-loader startup validation only: never execute the camera entrypoint."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import statistics

ROOT=Path(__file__).resolve().parents[1]
MODULE=ROOT/'ros2/src/so101_station_perception/so101_station_perception/camera_startup.py'


def benchmark(prefix,sdk_prefix,samples=5):
    if not 3<=samples<=10:raise ValueError('use3..10 bounded startup samples')
    spec=importlib.util.spec_from_file_location('camera_startup',MODULE)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    results=[module.validate_repair(prefix,sdk_prefix) for _ in range(samples)]
    values=[r['validation_ms'] for r in results]
    return {'schema':'camera-startup-validation-benchmark-v1','scope':'full201-source+SO/SDK hashes, CMake flags, actual ELF --list under camera preload; no camera entrypoint/device IO; shared host, filesystem cache uncontrolled',
            'samples_ms':values,'mean_ms':statistics.mean(values),'p95_ms':sorted(values)[-1],
            'first_sample_ms':values[0],'module_sha256':hashlib.sha256(MODULE.read_bytes()).hexdigest(),
            'validated_repair':results[-1],'physical_camera_started':False,'qualified':False}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix',type=Path,required=True)
    parser.add_argument('--sdk-prefix',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();report=benchmark(args.prefix,args.sdk_prefix)
    with args.output.open('x') as stream:json.dump(report,stream,indent=2)
    print(json.dumps(report,indent=2))
