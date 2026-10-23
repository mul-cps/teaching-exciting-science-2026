"""Offline only: validate every preparation stage and every float arithmetic ID."""
import argparse
from pathlib import Path
import re
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--tools', type=Path, required=True)
    parser.add_argument('--build', type=Path, required=True)
    args = parser.parse_args()
    expected = {
        'alignment-shaders': {'splat': False, 'finalize': False},
        'geometry-shaders': {'geometry_coordinates': True, 'geometry_support_scan': False,
                             'geometry_support_offsets': False, 'geometry_support_scatter': True,
                             'geometry_valid_count': False},
    }
    for directory, entries in expected.items():
        for entry, needs_fp64 in entries.items():
            path = args.build/directory/(entry+'.spv')
            subprocess.run([str(args.tools/'spirv-val'), '--target-env', 'vulkan1.1', str(path)], check=True)
            text = subprocess.check_output([str(args.tools/'spirv-dis'), str(path)], text=True)
            decorated = set(re.findall(r'OpDecorate\s+(%\w+)\s+NoContraction', text))
            arithmetic = set(re.findall(r'(%\w+)\s*=\s*Op(?:FAdd|FSub|FMul|FDiv|Dot)\b', text))
            if missing := arithmetic-decorated:
                raise AssertionError(f'{entry}: arithmetic lacks NoContraction: {sorted(missing)}')
            if ' Fma ' in text:
                raise AssertionError(f'{entry}: fused arithmetic is not reference qualified')
            if ('OpCapability Float64' in text) != needs_fp64:
                raise AssertionError(f'{entry}: unexpected FP64 capability')
            print(f'{entry}: validation PASS; {len(arithmetic)} arithmetic IDs all NoContraction; Float64={needs_fp64}')


if __name__ == '__main__':
    main()
