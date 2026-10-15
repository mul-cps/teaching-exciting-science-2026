"""Build/display an offline RGB-D + MuJoCo station twin; never open hardware."""
import argparse
import json
import os
from pathlib import Path
import time

# Select headless EGL before importing MuJoCo, including when only rendering.
os.environ.setdefault('MUJOCO_GL', 'egl')

from .scene import prepare


DEFAULT_CAMERA_ARM_CALIBRATION = Path.home() / '.cache/cps-tag-calibration-held/branch-consistency-report.json'
DEFAULT_CAMERA_TARGET_CALIBRATION = (Path.home() / '.local/share/exciting-science/station/evidence/2026-10-14/'
                                     'realsense-station/charuco-table-7x5/camera-from-board.json')
DEFAULT_PHASE_FILE = Path('/tmp/cps-pick-place-phase.json')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', type=Path, required=True)
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--camera-arm-calibration', type=Path, default=DEFAULT_CAMERA_ARM_CALIBRATION,
                        help='Saved provisional base_from_camera transform; never opens hardware')
    parser.add_argument('--camera-target-calibration', type=Path, default=DEFAULT_CAMERA_TARGET_CALIBRATION,
                        help='Saved camera_from_target ChArUco pose; never opens the camera')
    parser.add_argument('--phase-file', type=Path, default=DEFAULT_PHASE_FILE,
                        help='Read-only controller phase JSON; samples older than 0.5 s are marked stale')
    parser.add_argument('--output', type=Path, default=Path.home() / '.local/state/exciting-science/digital-twin')
    parser.add_argument('--keyframes', type=Path, help='Saved keyframe folder with synchronized manifest.jsonl; use its newest synchronized pose')
    parser.add_argument('--rgbd-keyframes', type=Path,
                        help='Saved RGB-D keyframe folder to display and poll; never access camera')
    parser.add_argument('--rgbd-live-folder', type=Path,
                        help='Saved latest RGB-D plus qualified latest.json metadata; reject stale/unpaired samples and poll files only')
    parser.add_argument('--fiducial-calibration', type=Path,
                        help='Optional saved 25h9 wrist-tag fit JSON; draws an unqualified comparison from paired RGB-D only')
    parser.add_argument('--register-icp',action='store_true',help='Iterate measured follower points to official CAD with --base-yaw hard prior')
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
    if args.rgbd_live_folder is not None and (args.keyframes is not None or args.rgbd_keyframes is not None
            or args.image is not None or args.depth is not None or args.original_snapshot):
        parser.error('--rgbd-live-folder cannot be combined with keyframe, explicit image/depth, or original-snapshot selection')
    selected_keyframe = None
    if args.keyframes is not None:
        from .scene import digest
        from .evidence_bridge import select_latest_keyframe_pair
        selected_keyframe = select_latest_keyframe_pair(args.keyframes)
        if selected_keyframe is None: parser.error('no synchronized saved keyframe within 160 ms')
        args.output.mkdir(parents=True,exist_ok=True)
        args.home=args.output/'selected-keyframe-pose.json'
        args.home.write_text(json.dumps({'raw_position':selected_keyframe['pose_raw'],'source_keyframe':selected_keyframe['keyframe_record'],'manifest_sha256':digest(selected_keyframe['manifest'])},indent=2)+'\n')
        args.image=selected_keyframe['image'];args.depth=selected_keyframe['depth']
        args.no_pose_series=True
    rgbd_keyframes = args.rgbd_keyframes or args.keyframes
    if rgbd_keyframes is None:
        inferred_keyframes = args.snapshot / 'tag-calibration-held' / 'keyframes'
        if (inferred_keyframes / 'manifest.jsonl').is_file():
            rgbd_keyframes = inferred_keyframes
    watch_latest_saved_camera = args.image is None and args.depth is None and not args.original_snapshot
    selected_pair = selected_keyframe if selected_keyframe is not None else None
    camera_watch_path = None
    if args.rgbd_live_folder is not None:
        from .evidence_bridge import select_latest_rgbd_pair
        selected_pair = select_latest_rgbd_pair(args.rgbd_live_folder)
        if selected_pair is None:
            parser.error('--rgbd-live-folder has no fresh, synchronized latest RGB/depth pair (limit 0.5 s)')
        args.image, args.depth = selected_pair['image'], selected_pair['depth']
        camera_watch_path = args.rgbd_live_folder
        watch_latest_saved_camera = True
    elif watch_latest_saved_camera and rgbd_keyframes is not None:
        from .evidence_bridge import select_latest_keyframe_pair
        selected_pair = select_latest_keyframe_pair(rgbd_keyframes)
        if selected_pair is not None:
            args.image, args.depth = selected_pair['image'], selected_pair['depth']
            camera_watch_path = rgbd_keyframes
    if watch_latest_saved_camera and selected_pair is None:
        from .evidence_bridge import select_latest_rgbd_pair
        selected_pair = select_latest_rgbd_pair(args.snapshot)
        if selected_pair is not None:
            args.image, args.depth = selected_pair['image'], selected_pair['depth']
            camera_watch_path = args.snapshot
    watch_latest_saved_camera = watch_latest_saved_camera and selected_pair is not None
    from .evidence_bridge import (camera_sample_status, load_camera_arm_registration,
                                  pointcloud_cache_status, read_phase_evidence,
                                  raw_pose_to_joint_degrees, scene_from_camera_world_delta,
                                  write_pointcloud_provenance)
    registration = load_camera_arm_registration(args.camera_arm_calibration,
                                                args.camera_target_calibration)
    fiducial_calibration = None
    if args.fiducial_calibration is not None:
        from .fiducial import validate_fiducial_calibration
        fiducial_calibration = validate_fiducial_calibration(
            json.loads(args.fiducial_calibration.read_text()))
    report = prepare(args.snapshot, args.calibration, args.output, latest=not args.original_snapshot,
        home_path=args.home, image_override=args.image, depth_override=args.depth,
        pose_series=not args.no_pose_series, convention=args.model_convention,
        base_yaw=args.base_yaw, register_icp=args.register_icp,
        external_camera_arm_registration=registration)
    report['camera_sample_timestamp_wall'] = (selected_pair['timestamp_wall'] if selected_pair else
        min(Path(report['image']).stat().st_mtime, Path(report['depth']).stat().st_mtime))
    report['camera_sample_timestamp_monotonic'] = selected_pair.get('timestamp_monotonic') if selected_pair else None
    report['camera_sample_pose_raw'] = selected_pair.get('pose_raw') if selected_pair else None
    report['camera_sample_pose_monotonic'] = selected_pair.get('pose_monotonic') if selected_pair else None
    report['camera_sample_pose_delta_seconds'] = selected_pair.get('pose_sample_delta_seconds') if selected_pair else None
    report['camera_sample_pose_qualified_sync'] = selected_pair.get('pose_qualified_sync') if selected_pair else None
    report['camera_sample_timestamp_source'] = ('synchronized keyframe manifest' if selected_pair and selected_pair.get('pose_synchronized')
                                                  else selected_pair.get('timestamp_source', 'paired RGB/depth file mtime') if selected_pair
                                                  else 'paired RGB/depth file mtime')
    report['camera_keyframe'] = ({'index': selected_pair.get('keyframe_index'),
        'phase': selected_pair.get('phase'), 'pose_raw': selected_pair.get('pose_raw'),
        'pose_synchronized': selected_pair.get('pose_synchronized'),
        'phase_event_delta_s': selected_pair.get('phase_event_delta_s'),
        'recorded': True} if selected_pair and selected_pair.get('pose_synchronized') else None)
    write_pointcloud_provenance(report, args.output)
    scene_world_from_camera = report['world_from_camera']
    world_from_scene = scene_from_camera_world_delta(scene_world_from_camera,
                                                      registration['world_from_camera'])
    report['scene_world_from_camera'] = scene_world_from_camera
    report['world_from_camera'] = registration['world_from_camera']
    report['world_from_scene'] = world_from_scene.tolist()
    report['camera_arm_registration'] = registration
    report['fiducial_overlay'] = ({'enabled': True, 'source_status': 'SOURCE UNQUALIFIED',
        'source': str(args.fiducial_calibration), 'dictionary': 'DICT_APRILTAG_25h9',
        'tag_id': 0, 'tag_size_m': .0194,
        'association': 'initial wrist ROI x=330..440,y=200..290, then nearest track within 40 px; ambiguous candidates rejected',
        'prediction_source': 'qualified paired actual raw pose; never virtual slider or commanded goal'}
        if fiducial_calibration is not None else {'enabled': False})
    report['watch_latest_saved_camera'] = watch_latest_saved_camera
    report['frame_convention'] = 'ChArUco board origin; target y/z flipped so world +Z is up; scene points re-expressed from table-fit frame'
    report['arm']['provisional_world_from_base'] = registration['world_from_base']
    report['limitations'].append('camera-to-arm transform combines two held but unqualified visual calibrations; provisional only')
    phase = read_phase_evidence(args.phase_file)
    report['phase_evidence'] = {**phase, 'path': str(args.phase_file)}
    if phase.get('pose_raw'):
        saved_calibration = json.loads(args.calibration.read_text())
        phase_degrees = raw_pose_to_joint_degrees(phase['pose_raw'], saved_calibration)
        report['arm']['raw_position'] = phase['pose_raw']
        report['arm']['nominal_joint_degrees'] = phase_degrees
        report['arm']['model_joint_degrees'] = phase_degrees
    report['camera_sample_status'] = camera_sample_status(report, phase,
                                                          cache_reusable=pointcloud_cache_status(report, args.output)['reusable'])
    (args.output / 'scene-evidence.json').write_text(json.dumps(report, indent=2) + '\n')
    from .viewer import render, serve, set_pose
    from .kinematics import export_pose_pair
    if args.render_only:
        import mujoco
        model = mujoco.MjModel.from_xml_path(str(args.output / 'scene.xml'))
        data = mujoco.MjData(model)
        from .evidence_bridge import apply_world_registration
        apply_world_registration(model, data, registration, scene_world_from_camera)
        set_pose(model, data, report['arm'].get('model_joint_degrees', report['arm']['nominal_joint_degrees']))
        report['kinematics'], report['visual_fit_kinematics'] = export_pose_pair(model, data, report['arm'])
        if report['camera_sample_status']['freshness'] == 'fresh':
            from .comparison import compare_registration
            report['registration_camera_comparison'] = compare_registration(model,data,report,args.output)
            report['camera_comparison'] = report['registration_camera_comparison']['after']
        else:
            report['registration_camera_comparison'] = {'status': 'SKIPPED_STALE_POSE_OR_CAMERA_SAMPLE',
                'reason': report['camera_sample_status']['reason']}
            report['camera_comparison'] = report['registration_camera_comparison']
        (args.output / 'scene-evidence.json').write_text(json.dumps(report, indent=2) + '\n')
        render(model, data, args.output / 'mujoco-render.png')
        print(args.output / 'mujoco-render.png')
        return 0
    server = serve(args.output, args.port, args.host, phase_file=args.phase_file,
                   calibration_path=args.calibration,
                   watch_latest_saved_camera=camera_watch_path,
                   fiducial_calibration=fiducial_calibration)
    try:
        while True: time.sleep(1)
    except KeyboardInterrupt:
        server.stop()
    return 0


if __name__ == '__main__': raise SystemExit(main())
