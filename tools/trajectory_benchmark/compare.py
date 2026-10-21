#!/usr/bin/env python3
"""Bounded offline timing comparison. Never imports a ROS node or contacts hardware."""
from __future__ import annotations
import argparse
import ast
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
PERIOD = .02

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def write(path, data):
    path.write_text(json.dumps(data, indent=2, sort_keys=True, allow_nan=False) + '\n')
    return sha(path)

def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module

def bridge(helper, names, points, start):
    # Execute precisely the existing pure timing function without importing its ROS node.
    path = ROOT/'ros2/src/so101_station_control/so101_station_control/trajectory_server.py'
    tree = ast.parse(path.read_text())
    fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'prepare_with_controller_timing')
    ns = {'copy': copy}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), str(path), 'exec'), ns)
    factor, command, prepared = ns[fn.name](helper, names, points,
        start_radians=start, max_jerk=2.0,
        motion_profile=helper.RosMotionProfile('default', .125, .2, 2.0))
    method = 'uniform_controller_timing'
    candidate = helper.prepare_collinear_quintic(names, points, start_radians=start,
        max_velocity=.125, max_acceleration=.2, max_jerk=2., period=PERIOD)
    if candidate is not None and candidate[1].duration < prepared.duration:
        command, prepared, _ = candidate
        factor = prepared.duration / points[-1]['time']
        method = 'monotone_collinear_rest_to_rest_quintic'
    return [{'time': s.time, 'positions': list(s.positions)} for s in prepared.samples], {
        'trajectory_duration_s': prepared.duration, 'retiming_factor': factor, 'timing_method': method,
        'sample_joint_names': list(helper.ARM_JOINTS), 'command_points': command}

def hermite(points):
    """C2 quintic Hermite interpolation preserving final MoveIt p/v/a at every knot."""
    segments = []
    for p, q in zip(points, points[1:]):
        dt = q['time']-p['time']
        if dt <= 0: raise ValueError('Nonincreasing MoveIt knot time')
        c = np.zeros((6, len(p['positions'])))
        c[0], c[1], c[2] = p['positions'], np.array(p['velocities'])*dt, np.array(p['accelerations'])*dt**2/2
        rhs = np.stack([np.array(q['positions'])-c[:3].sum(axis=0),
                        np.array(q['velocities'])*dt-c[1]-2*c[2],
                        np.array(q['accelerations'])*dt**2-2*c[2]])
        c[3:] = np.linalg.solve(np.array([[1,1,1],[3,4,5],[6,12,20]]), rhs)
        segments.append((p['time'],dt,c))
    rows = []
    end = points[-1]['time']
    for t in np.arange(math.ceil(end/PERIOD-1e-12)+1)*PERIOD:
        t0,dt,c = next((s for s in segments if t <= s[0]+s[1]), segments[-1])
        u = np.clip((min(t,end)-t0)/dt,0,1)
        row={'time':float(t)}
        for order,field in enumerate(('positions','velocities','accelerations','jerk')):
            co=c.copy()
            for _ in range(order): co=np.arange(1,len(co))[:,None]*co[1:]/dt
            row[field]=np.polynomial.polynomial.polyval(u,co).tolist()
        if t > end:
            for field in ('velocities','accelerations','jerk'): row[field]=[0.]*len(c[0])
        rows.append(row)
    # Exact extrema of each segment polynomial (including both sides of knots).
    peaks=[]
    for order in (1,2,3):
        peak=0.
        for _,dt,c in segments:
            co=c.copy()
            for _ in range(order): co=np.arange(1,len(co))[:,None]*co[1:]/dt
            for joint in co.T:
                derivative=np.arange(1,len(joint))*joint[1:]
                roots=np.polynomial.polynomial.polyroots(derivative)
                candidates=[0.,1.]+[float(r.real) for r in roots if abs(r.imag)<1e-8 and 0<r.real<1]
                peak=max(peak,max(abs(np.polynomial.polynomial.polyval(candidates,joint))))
        peaks.append(peak)
    return rows,dict(zip(('analytic_peak_velocity_rad_s','analytic_peak_acceleration_rad_s2','analytic_peak_jerk_rad_s3'),peaks))

def distances(points, path):
    best=np.full(len(points),np.inf)
    for a,b in zip(path,path[1:]):
        delta=b-a; norm=delta@delta
        f=np.zeros(len(points)) if norm==0 else np.clip((points-a)@delta/norm,0,1)
        best=np.minimum(best,np.linalg.norm(points-a-f[:,None]*delta,axis=1))
    return best

def metrics(rows, names, frozen, frozen_names):
    q=np.array([r['positions'] for r in rows])[:,[names.index(j) for j in frozen_names]]
    path=np.array([p['positions'] for p in frozen])
    derivatives=[]; d=np.pad(q, ((3,3),(0,0)), mode="edge")
    for _ in range(3):
        d=np.diff(d,axis=0)/PERIOD
        derivatives.append(float(np.max(abs(d))) if d.size else 0.)
    velocity=np.diff(q,axis=0)
    reversals=[]
    for col in velocity.T:
        signs=np.sign(col[abs(col)>1e-10]); reversals.append(int(np.count_nonzero(signs[1:]!=signs[:-1])))
    return {'command_grid_duration_s':rows[-1]['time'], 'sample_count':len(rows),
        'finite_difference_peak_velocity_rad_s':derivatives[0],
        'finite_difference_peak_acceleration_rad_s2':derivatives[1],
        'finite_difference_peak_jerk_rad_s3':derivatives[2],
        'reversals_per_joint':dict(zip(frozen_names,reversals)),
        'max_sample_distance_to_input_polyline_l2_rad':float(max(distances(q,path))),
        'max_input_waypoint_distance_to_sample_polyline_l2_rad':float(max(distances(path,q))),
        'endpoint_error_linf_rad':float(np.max(abs(q[[0,-1]]-path[[0,-1]]))),
        'finite_difference_limit_pass':all(x<=limit*(1+1e-6) for x,limit in zip(derivatives,(.125,.2,2.)))}

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--artifact',type=Path,action='append',required=True,help='Existing controller artifact or exported input.json (read only)')
    ap.add_argument('--urdf',type=Path,required=True)
    ap.add_argument('--srdf',type=Path,required=True)
    ap.add_argument('--backend',type=Path,required=True)
    ap.add_argument('--prefix',type=Path,required=True,help='Installed ROS/MoveIt prefix')
    ap.add_argument('--config',type=Path,action='append',default=[],help='Freeze additional station/model configuration')
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    if args.output.exists(): raise ValueError('Output must not already exist; preserve frozen experiments')
    args.output.mkdir(parents=True)
    for name,path in [('model.urdf',args.urdf),('model.srdf',args.srdf)]: shutil.copyfile(path,args.output/name)
    helper=load_module(ROOT/'workshop/station/ros_trajectory.py','offline_timing_helper')
    provenance={str(p.resolve()):sha(p) for p in [args.urdf,args.srdf,args.backend,*args.config,
        ROOT/'workshop/station/ros_trajectory.py',ROOT/'ros2/src/so101_station_control/so101_station_control/trajectory_server.py',Path(__file__),Path(__file__).with_name('moveit_timing.cpp')]}
    versions={}
    for pkg in ('moveit_core','ruckig','rclcpp'):
        package=args.prefix/'share'/pkg/'package.xml'
        if package.exists(): versions[pkg]=ET.parse(package).getroot().findtext('version'); provenance[str(package)]=sha(package)
    libraries={str(p):sha(p) for pattern in ('libmoveit_trajectory_processing.so*','libruckig.so*') for p in (args.prefix/'lib').glob(pattern) if p.is_file()}
    report={'schema':1,'status':'OFFLINE_ONLY_NOT_OWNER_ADMISSIBLE_NOT_COLLISION_APPROVED',
        'limits':{'velocity_rad_s':.125,'acceleration_rad_s2':.2,'jerk_rad_s3':2.},
        'period_s':PERIOD,'totg':{'path_tolerance_rad':.001,'resample_dt_s':.1,'min_angle_change_rad':.001},
        'ruckig':{'mitigate_overshoot':True,'overshoot_threshold_rad':.001},
        'versions':versions,'conda_backend_packages':{p.stem:json.loads(p.read_text())['version'] for p in (args.prefix/'conda-meta').glob('*.json') if any(x in p.name for x in ('moveit-core','ruckig'))},'source_sha':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        'provenance':provenance,'libraries':libraries,'cases':[]}
    for index,artifact_path in enumerate(args.artifact):
        case_dir=args.output/f'case-{index:02d}';case_dir.mkdir()
        artifact=json.loads(artifact_path.read_text())
        names=artifact['joint_names']
        if 'original_points' in artifact:
            points=copy.deepcopy(artifact['original_points'])
            start={j:math.radians(artifact['start_state']['pose'][j]) for j in names}
        else:
            points=copy.deepcopy(artifact['points']);start=artifact['start_radians']
        # The current bridge anchors its first knot to the measured start: freeze that geometry for every arm.
        points[0]['positions']=[start[j] for j in names]
        frozen={'joint_names':names,'points':points,'start_radians':start}
        case={'source':str(artifact_path.resolve()),'source_sha256':sha(artifact_path),
            'frozen_input_sha256':write(case_dir/'input.json',frozen),'arms':{}}
        path_file=case_dir/'path.txt'
        path_file.write_text(f'{len(points)} {len(names)}\n'+' '.join(names)+'\n'+'\n'.join(' '.join(format(x,'.17g') for x in p['positions']) for p in points)+'\n')
        began=time.perf_counter()
        try:
            rows,details=bridge(helper,names,points,start)
            case['arms']['A_current_bridge']={**details,**metrics(rows,details['sample_joint_names'],points,names),
                'wall_ms':(time.perf_counter()-began)*1000,'samples_sha256':write(case_dir/'A-samples.json',rows)}
        except ValueError as error: case['arms']['A_current_bridge']={'error':str(error)}
        result=subprocess.run([str(args.backend.resolve()),str(args.urdf.resolve()),str(args.srdf.resolve()),str(path_file.resolve()),'.125','.2','2'],capture_output=True,text=True,timeout=60)
        (case_dir/'backend.stderr').write_text(result.stderr)
        if result.returncode:
            case['backend_error']=result.stderr;report['cases'].append(case);continue
        output=json.loads(result.stdout);write(case_dir/'moveit-knots.json',output)
        case['backend_wall_ms']={k:v for k,v in output.items() if k.endswith('_wall_ms')}
        for label,fn in [('B_moveit_then_current_bridge',lambda:bridge(helper,names,output['ruckig'],start)),
                         ('C_direct_derivative_hermite',lambda:hermite(output['ruckig']))]:
            began=time.perf_counter()
            try:
                rows,details=fn();sample_names=details.get('sample_joint_names',names)
                case['arms'][label]={**details,**metrics(rows,sample_names,points,names),
                    'interpolation_wall_ms':(time.perf_counter()-began)*1000,
                    'sample_joint_names':list(sample_names),'samples_sha256':write(case_dir/(label[0]+'-samples.json'),rows)}
            except ValueError as error:case['arms'][label]={'error':str(error)}
        case['moveit_knot_end_derivatives']={side:{field:output['ruckig'][i][field] for field in ('velocities','accelerations')} for side,i in [('start',0),('end',-1)]}
        report['cases'].append(case)
    write(args.output/'report.json',report)
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
