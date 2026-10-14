"""Build/display an offline RGB-D + MuJoCo station twin; never open hardware."""
import argparse
import os
from pathlib import Path
import time

# Select headless EGL before importing MuJoCo, including when only rendering.
os.environ.setdefault('MUJOCO_GL', 'egl')

from .scene import prepare


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', type=Path, required=True)
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=Path.home() / '.local/state/exciting-science/digital-twin')
    parser.add_argument('--base-yaw',type=float,help='User visual mount-yaw prior in degrees; not measured registration')
    parser.add_argument('--model-convention',choices=('new','old'),default='new',help='Official native calibration hypothesis; does not change hardware calibration')
    parser.add_argument('--image', type=Path, help='Explicit saved RGB frame paired with --depth')
    parser.add_argument('--depth', type=Path, help='Explicit aligned saved NPY depth paired with --image')
    parser.add_argument('--no-pose-series', action='store_true', help='Do not reuse old multi-pose capture after scene movement')
    parser.add_argument('--home', type=Path, help='Saved measured-home JSON, never read hardware')
    parser.add_argument('--original-snapshot', action='store_true')
    parser.add_argument('--render-only', action='store_true')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8096)
    args = parser.parse_args(argv)
    report = prepare(args.snapshot, args.calibration, args.output, latest=not args.original_snapshot, home_path=args.home, image_override=args.image, depth_override=args.depth, pose_series=not args.no_pose_series, convention=args.model_convention, base_yaw=args.base_yaw)
    from .viewer import render, serve, set_pose
    from .kinematics import export_pose_pair
    if args.render_only:
        import mujoco
        model = mujoco.MjModel.from_xml_path(str(args.output / 'scene.xml'))
        data = mujoco.MjData(model)
        set_pose(model, data, report['arm'].get('model_joint_degrees', report['arm']['nominal_joint_degrees']))
        import json
        report['kinematics'], report['visual_fit_kinematics'] = export_pose_pair(model, data, report['arm'])
        from .comparison import compare_camera
        report['camera_comparison'] = compare_camera(model, data, report, args.output)
        (args.output / 'scene-evidence.json').write_text(json.dumps(report, indent=2) + '\n')
        render(model, data, args.output / 'mujoco-render.png')
        print(args.output / 'mujoco-render.png')
        return 0
    server = serve(args.output, args.port, args.host)
    try:
        while True: time.sleep(1)
    except KeyboardInterrupt:
        server.stop()
    return 0


if __name__ == '__main__': raise SystemExit(main())
