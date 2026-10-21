#!/usr/bin/env python3
"""Render/install disabled fwmini4 user units; never enable, start or bootstrap."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
ROLES = ('owner','camera','tags','movegroup','objects','scene','leader','apis')
UNITS = {role: 'so101-fwmini4-'+role+'.service' for role in ROLES}
RESTART_ROLES = ('camera','tags','movegroup','objects','scene','leader')
SOURCES = ('tools/install_station_user_units.py','tools/station_process.py',
           'tools/freeze_ros_joint_limits.py',
           'ros2/src/so101_station_bringup/launch/station_live.launch.py',
           'ros2/src/so101_station_control/launch/moveit_resident.launch.py',
           'ros2/src/so101_station_perception/launch/perception.launch.py',
           'ros2/src/so101_station_operator/launch/operator.launch.py',
           'ros2/src/so101_station_operator/scripts/operator_api',
           'ros2/src/so101_station_control/so101_station_control/operator_restart.py',
           'ros2/src/so101_station_control/so101_station_control/trajectory_server.py')
COMMON_KEYS = {'station_root','station_config','station_python','state_file','station_lock','log_dir',
               'joint_limit_contract','calibration_dir','camera_repair_prefix','camera_sdk_prefix',
               'enable_hardware','external_state_only','rviz','use_station_api','use_operator_panel_api',
               'use_ev3_tunnel','web_twin_host','color_mask_backend'}


def native_parameters():
    camera = '/camera'
    nodes = {key:camera for key in ('rgb_image','depth_image','rgb','depth')}
    nodes.update(tags='/so101_tag_observations',objects='/so101_colored_objects',
                 objects_status='/so101_colored_objects',scene_status='/so101_scene_updater',
                 joint_states='/follower/trajectory_server',movegroup='/follower/move_group')
    endpoints = dict(rgb_image='/camera/color/image_raw',depth_image='/camera/aligned_depth_to_color/image_raw',
                     rgb='/camera/color/camera_info',depth='/camera/aligned_depth_to_color/camera_info',
                     tags='/so101_perception/tag_detections',objects='/so101/objects',
                     objects_status='/so101/grasp_status',scene_status='/so101_scene/status',
                     joint_states='/follower/joint_states',movegroup_scene='/follower/monitored_planning_scene')
    return {
        'operator_service_restarts':'true','operator_leader_required':'true','operator_recovery_profile':'native_ros',
        'operator_native_mask_backend':'vulkan','operator_native_pair_delta_s':'0.04',
        'operator_native_publisher_nodes_json':json.dumps(nodes,sort_keys=True,separators=(',',':')),
        **{'operator_'+key+'_topic':value for key,value in endpoints.items()},
        **{'operator_'+key+'_unit':UNITS[role] for key,role in (
            ('camera','camera'),('depth','camera'),('tags','tags'),('movegroup','movegroup'),
            ('objects','objects'),('scene','scene'),('leader_readonly','leader'))},
        'operator_maintenance_journal':str(Path.home()/'.local/share/exciting-science/station/operator-maintenance-pending.json'),
        'recovery_depth_topic':endpoints['depth_image'],'recovery_camera_info_topic':endpoints['rgb'],
        'marker_depth_topic':endpoints['depth_image'],'objects_depth_topic':endpoints['depth_image'],
        'track_charuco_world':'true','marker_allow_derived_source':'false','objects_allow_derived_source':'false',
        'serial_no':'243322071783','color_profile':'640,480,30','depth_profile':'640,480,30',
        'ev3_left_end_deg':'10.0','ev3_right_end_deg':'530.0',
    }


def station_module():
    try:
        from tools import station_process
    except ModuleNotFoundError:
        import station_process
    return station_process


def build_manifest(args):
    if args.profile!='fwmini4' or args.wrapper or args.wrapper_command or args.domain not in (None,177):
        raise ValueError('native units require fixed fwmini4 wrapper and domain177')
    result = dict(schema='so101-fwmini4-units-v1',uid=os.getuid(),root=str(ROOT),domain=177,
                  wrapper=str(Path.home()/'.local/bin/station-fwmini4'),
                  parameters={**station_module().station_launch_parameters(args),**native_parameters()},
                  sources={path:hashlib.sha256((ROOT/path).read_bytes()).hexdigest() for path in SOURCES})
    validate_manifest(result)
    return result


def validate_manifest(value):
    if (not isinstance(value,dict) or set(value)!={'schema','uid','root','domain','wrapper','parameters','sources'} or
            value.get('schema')!='so101-fwmini4-units-v1' or type(value.get('uid')) is not int or value['uid']!=os.getuid() or
            type(value.get('domain')) is not int or value['domain']!=177 or value.get('root')!=str(ROOT) or
            value.get('wrapper')!=str(Path.home()/'.local/bin/station-fwmini4')):
        raise ValueError('native unit manifest identity/schema/wrapper/domain mismatch')
    fixed = native_parameters()
    params = value['parameters']
    if not isinstance(params,dict) or set(params)!=COMMON_KEYS|set(fixed):
        raise ValueError('native unit manifest parameter set is not exact')
    for key,item in params.items():
        if not isinstance(item,str) or not 1<=len(item)<=4096 or any(ord(c)<32 or ord(c)==127 for c in item):
            raise ValueError('native unit parameter is invalid: '+key)
    fixed.update(station_root=str(ROOT),station_python=str(Path.home()/'.local/share/exciting-science/station/venv/bin/python'),
                 camera_repair_prefix=str(ROOT/'ros2/.cache/realsense-cpu-texture-bounds/install'),
                 enable_hardware='true',external_state_only='false',rviz='true',use_station_api='true',
                 use_operator_panel_api='true',web_twin_host='127.0.0.1',color_mask_backend='vulkan')
    if any(params[key]!=expected for key,expected in fixed.items()) or params['use_ev3_tunnel'] not in ('true','false'):
        raise ValueError('native unit fixed recipe/allowlist/backend differs')
    for key in ('station_config','state_file','station_lock','log_dir','joint_limit_contract','calibration_dir','camera_sdk_prefix'):
        if not Path(params[key]).is_absolute():
            raise ValueError('native unit path must be absolute: '+key)
    expected_sources = {path:hashlib.sha256((ROOT/path).read_bytes()).hexdigest() for path in SOURCES}
    if value['sources']!=expected_sources:
        raise ValueError('native unit source cohort changed; render/review again')
    return value


def load_manifest(path):
    if path.stat().st_size>65536:
        raise ValueError('native unit manifest exceeds64KiB')
    return validate_manifest(json.loads(path.read_text()))


def digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':')).encode()).hexdigest()


def component_command(manifest,role):
    validate_manifest(manifest)
    if role not in ROLES and role!='rviz':
        raise ValueError('unknown native component role')
    return [manifest['wrapper'],'ros-vulkan','ros2','launch','so101_station_bringup','station_live.launch.py',
            *(key+':='+value for key,value in sorted(manifest['parameters'].items())),'component:='+role]


def unit_quote(value):
    # ExecStart ':' disables environment expansion. '%' remains a systemd specifier.
    return '"'+str(value).replace('\\','\\\\').replace('"','\\"').replace('%','%%')+'"'


def unit_text(manifest,role):
    if role not in ROLES:
        raise ValueError('no unit for this role')
    command=component_command(manifest,role)
    return '\n'.join([
        '# Generated disabled native station recipe '+digest(manifest),
        '[Unit]','Description=SO101 fwmini4 '+role+' (explicit manual startup only)',
        '[Service]','Type=exec','Restart=no','KillMode=control-group','KillSignal=SIGINT',
        'TimeoutStartSec=30','TimeoutStopSec=23','UMask=0077',
        'WorkingDirectory='+str(ROOT).replace('\\','\\\\').replace('%','%%'),
        'Environment="ROS_DOMAIN_ID=177" "ROS_LOCALHOST_ONLY=1"',
        'ExecStart=:'+ ' '.join(unit_quote(word) for word in command),'',
    ])


def unit_directory():
    return Path(os.environ.get('XDG_CONFIG_HOME',str(Path.home()/'.config')))/'systemd/user'


def write_installation(manifest,directory,manifest_path):
    validate_manifest(manifest)
    directory,manifest_path=Path(directory),Path(manifest_path)
    outputs={directory/UNITS[role]:unit_text(manifest,role) for role in ROLES}
    outputs[manifest_path]=json.dumps(manifest,indent=2,sort_keys=True)+'\n'
    for path,text in outputs.items():
        if path.is_symlink() or (path.exists() and path.read_text()!=text):
            raise RuntimeError('Refusing to overwrite existing/unowned file: '+str(path))
    for path,text in outputs.items():
        path.parent.mkdir(parents=True,exist_ok=True)
        if path.exists():continue
        descriptor,name=tempfile.mkstemp(prefix=path.name+'.',dir=path.parent)
        try:
            with os.fdopen(descriptor,'w') as stream:
                stream.write(text);stream.flush();os.fsync(stream.fileno())
            # Atomic no-clobber publication; retain any concurrently created file.
            os.link(name,path)
        finally:
            Path(name).unlink(missing_ok=True)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['render','install'])
    parser.add_argument('--output-dir',type=Path)
    parser.add_argument('--manifest',type=Path)
    opts,remaining=parser.parse_known_args(argv)
    args=station_module().parse_arguments(['start','--deployment','user-units',*remaining])
    value=build_manifest(args)
    if opts.action=='render':
        if opts.output_dir is None:parser.error('render requires --output-dir')
        directory=opts.output_dir
        manifest_path=opts.manifest or directory/'manifest.json'
    else:
        if opts.output_dir is not None:parser.error('install uses only the current user unit directory')
        directory=unit_directory()
        manifest_path=opts.manifest or Path.home()/'.local/share/exciting-science/station/fwmini4-units.json'
    write_installation(value,directory,manifest_path)
    print('Wrote disabled recipe '+str(manifest_path)+'; no reload, enable, start or bootstrap performed.')


if __name__=='__main__':
    try:main()
    except (ValueError,RuntimeError,OSError,KeyError) as error:
        print('station units: '+str(error),file=sys.stderr);raise SystemExit(2)
