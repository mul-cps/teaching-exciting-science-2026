"""Bounded stationary mesh/observed-point diagnostics, never correction authority."""
from collections import deque
import hashlib
import math
from pathlib import Path
import struct
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


class BudgetExceeded(ValueError):
    """The cooperative per-fit wall budget expired; no candidate is published."""


class Deadline:
    def __init__(self, seconds=.2, clock=time.monotonic):
        if not .001 <= seconds <= .5:
            raise ValueError('fit wall budget must be positive and <=500ms')
        self.clock, self.start, self.seconds = clock, clock(), seconds

    def check(self, now=None):
        if (self.clock() if now is None else now) - self.start >= self.seconds:
            raise BudgetExceeded('fit wall budget exceeded')


def rigid(value):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0,0,0,1], atol=1e-8)
            or not np.allclose(matrix[:3,:3].T @ matrix[:3,:3], np.eye(3), atol=1e-7)
            or not np.isclose(np.linalg.det(matrix[:3,:3]), 1., atol=1e-7)):
        raise ValueError('proper finite SE3 required')
    return matrix


class StationaryHistory:
    JOINTS = ('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper')

    def __init__(self):
        self.samples = deque(maxlen=128)

    def add(self, stamp, pose, *, now):
        if (not math.isfinite(stamp) or not 0 < stamp <= now or now-stamp > 2.
                or set(pose) != set(self.JOINTS) or not all(math.isfinite(v) for v in pose.values())):
            return False
        self.samples.append((stamp, dict(pose)))
        return True

    def select(self, stamp, *, now):
        if not 0 <= now-stamp <= .5:
            raise ValueError('cloud stale or future')
        valid = sorted(((s,p) for s,p in self.samples if 0 <= now-s <= 2.),key=lambda item:item[0])
        if not valid:
            raise ValueError('no joint history')
        nearest = min(valid,key=lambda x:(abs(x[0]-stamp),x[0]))
        if abs(nearest[0]-stamp) > .05:
            raise ValueError('unpaired joints')
        boundary = stamp-.5
        older = [(s,p) for s,p in valid if s <= boundary]
        if not older or boundary-older[-1][0] > .1:
            raise ValueError('stationary window not bracketed')
        window = [older[-1]]+[(s,p) for s,p in valid if boundary < s <= stamp]
        if stamp-window[-1][0]>.1 or any(b[0]-a[0]>.1+1e-9 for a,b in zip(window,window[1:])):
            raise ValueError('stationary history has gaps')
        values = np.array([[p[j] for j in self.JOINTS] for _,p in window])
        if np.max(np.ptp(values,axis=0)) > .004:
            raise ValueError('joint state moving')
        return nearest


class MeshSamples:
    """Once-load all bounded binary STL visuals in their actual URDF link frame."""
    def __init__(self, description, resolve_package, *, samples=4000):
        if len(description)>2_000_000 or not 100 <= samples <= 8000:
            raise ValueError('model/sample cap exceeded')
        try:root=ET.fromstring(description)
        except ET.ParseError as error:raise ValueError('invalid model XML') from error
        visuals, total, metadata = [],0,[]
        self.all_links=frozenset(link.get('name') for link in root.findall('link'))
        if len(self.all_links)>64:raise ValueError('model link cap exceeded')
        self.base_floor_z=None
        for link in root.findall('link'):
            for visual in link.findall('visual'):
                mesh=visual.find('geometry/mesh')
                if mesh is None or len(visuals)>=32:
                    raise ValueError('all visual geometry must be bounded binary STL')
                filename=mesh.get('filename','')
                if filename.startswith('package://'):
                    package,_,relative=filename[10:].partition('/')
                    filename=str(Path(resolve_package(package))/relative)
                path=Path(filename)
                if not path.is_absolute() or path.suffix.lower()!='.stl' or path.stat().st_size>30_000_084:
                    raise ValueError('bounded absolute STL required')
                raw=path.read_bytes()
                if len(raw)<84: raise ValueError('truncated STL')
                count=struct.unpack_from('<I',raw,80)[0];total+=count
                if not count or total>600000 or len(raw)!=84+count*50:
                    raise ValueError('model triangle cap/layout invalid')
                records=np.frombuffer(raw,dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')]),offset=84)
                origin=visual.find('origin')
                xyz=np.fromstring('0 0 0' if origin is None else origin.get('xyz','0 0 0'),sep=' ')
                rpy=np.fromstring('0 0 0' if origin is None else origin.get('rpy','0 0 0'),sep=' ')
                scale=np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
                if any(a.shape!=(3,) or not np.isfinite(a).all() for a in (xyz,rpy,scale)) or np.any(scale<=0):
                    raise ValueError('invalid visual origin/scale')
                vertices=records['v']*scale @ Rotation.from_euler('xyz',rpy).as_matrix().T+xyz
                if link.get('name')=='base_link':
                    z=float(vertices[:,:,2].min())
                    self.base_floor_z=z if self.base_floor_z is None else min(z,self.base_floor_z)
                cross=np.cross(vertices[:,1]-vertices[:,0],vertices[:,2]-vertices[:,0])
                areas=np.linalg.norm(cross,axis=1)*.5
                if not np.isfinite(vertices).all() or areas.sum()<=0: raise ValueError('invalid surface')
                visuals.append((link.get('name'),vertices,cross,areas))
                metadata.append({'link':link.get('name'),'path':str(path),'sha256':hashlib.sha256(raw).hexdigest(),'triangles':count})
        if not visuals: raise ValueError('no visual surfaces')
        area=sum(a.sum() for _,_,_,a in visuals)
        rng=np.random.default_rng(17);self.local=[]
        weights=np.array([areas.sum()/area for _,_,_,areas in visuals])
        counts=np.floor(weights*(samples-len(visuals))).astype(int)+1
        counts[np.argsort(-weights)[:samples-int(counts.sum())]]+=1
        for (link,vertices,cross,areas),count in zip(visuals,counts):
            faces=rng.choice(len(areas),size=count,p=areas/areas.sum())
            uv=rng.random((count,2));uv[uv.sum(axis=1)>1]=1-uv[uv.sum(axis=1)>1]
            triangles=vertices[faces]
            points=triangles[:,0]+uv[:,:1]*(triangles[:,1]-triangles[:,0])+uv[:,1:]*(triangles[:,2]-triangles[:,0])
            normals=cross[faces]/np.linalg.norm(cross[faces],axis=1)[:,None]
            self.local.append((link,points,normals))
        self.links=tuple(sorted({v[0] for v in self.local}))
        self.model_sha256=hashlib.sha256(description.encode()).hexdigest()
        self.provenance={'model_sha256':self.model_sha256,'visuals':metadata,'triangles':total,
                         'samples':sum(len(p) for _,p,_ in self.local),'qualified':False,
                         'base_link_cad_floor_z_m':self.base_floor_z}

    def at(self, base_from_links):
        points,normals,labels=[],[],[]
        for link,p,n in self.local:
            matrix=rigid(base_from_links[link]);r=matrix[:3,:3]
            points.append(p@r.T+matrix[:3,3]);normals.append(n@r.T)
            labels.append(np.repeat(link,len(p)))
        return np.concatenate(points),np.concatenate(normals),np.concatenate(labels)


def visible_samples(points,normals,labels,camera_from_base,k,width,height):
    """Approximate front-facing sample z-buffer; never fills unobserved geometry."""
    matrix=rigid(camera_from_base);p=points@matrix[:3,:3].T+matrix[:3,3]
    n=normals@matrix[:3,:3].T
    valid=(p[:,2]>.01)&(np.einsum('ij,ij->i',n,-p)>0)
    ids=np.flatnonzero(valid);p=p[ids]
    uv=p@np.asarray(k).reshape(3,3).T;uv=np.rint(uv[:,:2]/uv[:,2:]).astype(int)
    keep=(uv[:,0]>=0)&(uv[:,0]<width)&(uv[:,1]>=0)&(uv[:,1]<height)
    ids,uv=ids[keep],uv[keep]
    key=uv[:,1]*width+uv[:,0];order=np.lexsort((p[keep,2],key));ids,key=ids[order],key[order]
    if not len(ids): raise ValueError('no visible model samples')
    keep=np.r_[True,key[1:]!=key[:-1]];ids=ids[keep]
    return points[ids],normals[ids],labels[ids]


def fit_registration(points,normals,labels,observed,*,deadline=None):
    """One bounded fixed-correspondence robust point-to-plane proposal.

    Sample distance is not an exact-triangle residual or physical calibration.
    """
    deadline=deadline or Deadline();deadline.check()
    points,normals,labels,observed=map(np.asarray,(points,normals,labels,observed))
    if (points.ndim!=2 or points.shape[1]!=3 or normals.shape!=points.shape
            or len(labels)!=len(points) or observed.ndim!=2 or observed.shape[1]!=3
            or not 40<=len(points)<=8000 or not 40<=len(observed)<=4000
            or not np.isfinite(points).all() or not np.isfinite(normals).all()
            or not np.isfinite(observed).all()):
        raise ValueError('bounded finite model/cloud arrays required')
    tree=cKDTree(points);distance,indices=tree.query(observed,k=1,workers=1)
    keep=distance<=.02;target=observed[keep];indices=indices[keep]
    if len(target)<40: raise ValueError('insufficient observed/model coverage')
    if len(target)>2000:
        selected=np.linspace(0,len(target)-1,2000,dtype=int);target,indices=target[selected],indices[selected]
    q,n=points[indices],normals[indices]
    scale=max(.01,float(np.sqrt(np.mean(np.sum((q-q.mean(axis=0))**2,axis=1)))))
    jac=np.column_stack((n,np.cross(q,n)/scale))
    singular=np.linalg.svd(jac,compute_uv=False);rank=int(np.count_nonzero(singular>singular[0]*1e-3))
    report={'qualified':False,'applied':False,'state':'proposal_unqualified','candidate_delta':None,
            'observed_population':len(target),'input_observed_points':len(observed),
            'overlap_fraction':float(np.mean(keep)),
            'per_link_points':{str(link):int(np.count_nonzero(labels[indices]==link)) for link in np.unique(labels[indices])},
            'observability':{'rank':rank,'scaled_singular_values':singular.tolist(),'rotation_scale_m':scale},
            'method':'fixed observed population; sampled visual surface; one robust point-to-plane round'}
    before_distance=np.linalg.norm(target-q,axis=1)
    before_plane=np.abs(np.einsum('ij,ij->i',target-q,n))
    report['before']={'median_surface_sample_m':float(np.median(before_distance)),
                      'p95_surface_sample_m':float(np.percentile(before_distance,95)),
                      'p95_abs_point_to_plane_m':float(np.percentile(before_plane,95))}
    if rank<6:
        report['state']='insufficient_observability';return report
    deadline.check()
    def residual(x):
        deadline.check()
        rotation=Rotation.from_rotvec(x[3:]).as_matrix()
        return np.einsum('ij,ij->i',q@rotation.T+x[:3]-target,n@rotation.T)
    bound=np.array([.01]*3+[math.radians(2.)]*3)
    result=least_squares(residual,np.zeros(6),bounds=(-bound,bound),loss='soft_l1',f_scale=.002,max_nfev=20)
    deadline.check();x=result.x.copy()
    x[:3]*=min(1.,.01/max(np.linalg.norm(x[:3]),1e-15))
    x[3:]*=min(1.,math.radians(2.)/max(np.linalg.norm(x[3:]),1e-15))
    rotation=Rotation.from_rotvec(x[3:]).as_matrix()
    transformed=(target-x[:3])@rotation
    distance,after_indices=tree.query(transformed,k=1,workers=1)
    plane=np.abs(np.einsum('ij,ij->i',transformed-points[after_indices],normals[after_indices]))
    matrix=np.eye(4);matrix[:3,:3]=rotation;matrix[:3,3]=x[:3]
    report['candidate_delta']=matrix.tolist()
    report['after']={'median_surface_sample_m':float(np.median(distance)),
                     'p95_surface_sample_m':float(np.percentile(distance,95)),
                     'p95_abs_point_to_plane_m':float(np.percentile(plane,95))}
    report['bound_reached']=bool(np.linalg.norm(x[:3])>=.00999 or np.linalg.norm(x[3:])>=math.radians(1.999))
    report['optimizer']={'success':bool(result.success),'evaluations':int(result.nfev)}
    deadline.check();report['elapsed_ms']=(deadline.clock()-deadline.start)*1000
    return report


def marker_residual(mount,observation,camera_from_link,stamp,model_sha256,*,camera_frame=None):
    if mount is None or observation is None:
        return {'state':'unavailable_missing_mount_or_observation','qualified':False}
    try:
        if (not isinstance(mount.get('link'),str) or not mount['link']
                or type(mount.get('marker_id')) is not int or mount['marker_id']<0
                or type(observation.get('marker_id')) is not int
                or type(observation.get('detection_count')) is not int
                or mount['schema']!='so101-link-marker-v1' or mount['model_sha256']!=model_sha256
                or observation['association']!='unique_id' or observation.get('detection_count')!=1
                or (camera_frame is not None and observation.get('frame_id')!=camera_frame)
                or observation['dictionary']!=mount['dictionary'] or observation['marker_id']!=mount['marker_id']
                or not math.isfinite(observation['stamp']) or abs(observation['stamp']-stamp)>.05
                or observation['stamp']>stamp):
            raise ValueError('invalid marker binding')
        expected=rigid(camera_from_link)@rigid(mount['link_from_marker'])
        delta=np.linalg.inv(expected)@rigid(observation['camera_from_marker'])
        return {'state':'diagnostic_residual','qualified':False,'mount_qualified':mount.get('qualified') is True,
                'translation_m':float(np.linalg.norm(delta[:3,3])),
                'rotation_deg':float(np.linalg.norm(Rotation.from_matrix(delta[:3,:3]).as_rotvec())*180/math.pi)}
    except (KeyError,TypeError,ValueError):
        return {'state':'unavailable_invalid_binding','qualified':False}


def owner_eligibility(state):
    """Describe why current measured state cannot support a diagnostic fit."""
    if state.get('failed_endpoint_hold') is not None:
        return 'failed endpoint hold active; operator recovery required'
    if state.get('phase') not in (None,'idle','ready') or state.get('trajectory_goal_id') is not None:
        return 'owner motion phase/trajectory active'
    if state.get('camera_recovery_phase') is not None:
        return 'camera recovery active'
    component=state.get('component_command')
    if component is not None:
        completed=component.get('completed_monotonic') if isinstance(component,dict) else None
        if (not isinstance(component,dict) or component.get('status')!='complete'
                or type(completed) not in (int,float) or not math.isfinite(completed)
                or not 0<=completed<=state.get('monotonic',-1)):
            return 'component command active, failed or unknown'
    if state.get('component_restore_pending'):
        return 'component restoration pending'
    if state.get('violations')!=[]:
        return 'owner violations unavailable or nonempty'
    raw=state.get('raw',{});bounds=state.get('arm_bounds',{})
    if not isinstance(bounds,dict) or bounds.get('classification')!='normal':
        return 'normal arm bounds required'
    for key in ('admitted_min','admitted_max'):
        values=bounds.get(key)
        if (not isinstance(values,dict) or set(values)!=set(StationaryHistory.JOINTS)
                or any(type(v) is not int or not 0<=v<=4095 for v in values.values())):
            return 'complete integer admitted bounds required'
    if (not isinstance(raw,dict) or set(raw)!=set(StationaryHistory.JOINTS)
            or any(type(raw[j]) is not int or not bounds['admitted_min'][j]<=raw[j]<=bounds['admitted_max'][j]
                   for j in StationaryHistory.JOINTS)):
        return 'raw counts outside admitted bounds'
    torque=state.get('torque')
    if (not isinstance(torque,dict) or set(torque)!=set(StationaryHistory.JOINTS)
            or any(type(v) is not int or v not in (0,1) for v in torque.values())):
        return 'complete measured torque state required'
    return None


def owner_acquisition(item,joints,cohort,*,now):
    """Validate acquisition/identity separately from whether fitting is allowed.

    Source-pin preparation may record an ineligible failure state honestly;
    fitting always additionally calls owner_eligibility through owner_bracket.
    """
    from datetime import datetime
    state=item['state'];session=cohort.get('owner_session_id')
    if not isinstance(session,str) or not session:raise ValueError('pinned owner_session_id required')
    utc=datetime.fromisoformat(state['timestamp_utc'].replace('Z','+00:00'))
    if utc.tzinfo is None:raise ValueError('owner UTC lacks timezone')
    stamp=utc.timestamp();mono=state['monotonic']
    if (type(mono) not in (int,float) or not math.isfinite(mono)
            or state.get('schema')!=1 or state.get('event')!='sample'
            or not 0<=now-stamp<=1. or stamp>item['utc_after']+.005
            or not 0<=item['mono_after']-item['mono_before']<=.02
            or not 0<=item['utc_after']-item['utc_before']<=.02
            or not item['utc_before']-item['mono_after']-.005<=stamp-mono<=item['utc_after']-item['mono_before']+.005):
        raise ValueError('owner acquisition clocks/schema invalid or stale')
    for key in ('owner_session_id','arm_id','calibration_sha256','owner_source_sha256'):
        if state.get(key)!=cohort.get(key):raise ValueError('owner '+key+' differs from source cohort')
    if (set(state['raw'])!=set(StationaryHistory.JOINTS)
            or any(type(v) is not int or not 0<=v<=4095 for v in state['raw'].values())
            or set(state['pose'])!=set(StationaryHistory.JOINTS)
            or any(type(v) not in (float,int) or not math.isfinite(v) for v in state['pose'].values())
            or max(abs(math.radians(state['pose'][j])-joints[j]) for j in StationaryHistory.JOINTS[:-1])>.01):
        raise ValueError('owner raw/pose incomplete or disagrees with ROS')
    return stamp,state


def owner_bracket(reads,stamp,joints,cohort,*,now):
    """Bind eligible measured acquisitions on both sides, never restamp."""
    valid=[];reason='exposure lacks owner bracket'
    for item in reads:
        try:
            acquired,state=owner_acquisition(item,joints,cohort,now=now)
            fault=owner_eligibility(state)
            if fault is not None:raise ValueError(fault)
            valid.append((acquired,state))
        except (KeyError,TypeError,ValueError,OverflowError) as error:reason=str(error)
    before=[v for v in valid if v[0]<=stamp];after=[v for v in valid if v[0]>=stamp]
    if not before or not after:raise ValueError(reason)
    a,b=max(before,key=lambda v:v[0]),min(after,key=lambda v:v[0])
    if stamp-a[0]>.1 or b[0]-stamp>.1 or a[1]['raw']!=b[1]['raw']:
        raise ValueError('owner bracket moving/unpaired')
    return {'before_stamp':a[0],'after_stamp':b[0],'raw':a[1]['raw'],
            'owner_source_sha256':a[1]['owner_source_sha256'],
            'arm_id':a[1]['arm_id'],'owner_session_id':cohort['owner_session_id'],
            'calibration_sha256':a[1]['calibration_sha256']}


def validate_base_pin(actual,pinned):
    actual,pinned=rigid(actual),rigid(pinned)
    if not np.allclose(actual,pinned,rtol=0.,atol=1e-8):
        raise ValueError('world_from_base differs from pinned registration; repin/reset required')
    return actual


def ground_metrics(initial,candidate,*,cad_floor_z=None):
    """User-declared flat-base diagnostic only; no new optimizer constraints."""
    initial=rigid(initial)
    def metrics(matrix):
        matrix=rigid(matrix)
        tilt=math.degrees(math.acos(float(np.clip(matrix[2,2],-1.,1.))))
        height=None if cad_floor_z is None else float((matrix@[0.,0.,cad_floor_z,1.])[2])
        return {'base_up_tilt_deg':tilt,'cad_floor_origin_height_m':height}
    before=metrics(initial);after=metrics(candidate) if candidate is not None else None
    return {'constraint_source':'user-declared flat base; world +Z up; CAD minimum-z plane origin',
            'qualified':False,'current':before,'candidate':after,
            'candidate_ground_violation':None if after is None else bool(after['base_up_tilt_deg']>1e-5 or
                (after['cad_floor_origin_height_m'] is not None and abs(after['cad_floor_origin_height_m'])>1e-5)),
            'interpretation':'free SE3 diagnostic proposal may violate physical flat-base constraint; never applied'}


def make_source_cohort(description,owner,device_info,cloud_gid,*,station_config_path,
                       station_config_bytes,calibration_path,calibration_bytes,
                       base_registration_path,base_registration_bytes,world_from_base):
    """Prepare explicit runtime pins, including an honest fit-ineligibility reason.

    Inputs are actual captured bytes/ROS data. No files, hardware, TF or model
    are changed here, and no model version/serial/identity transform is guessed.
    """
    import json
    def digest(data):return hashlib.sha256(data).hexdigest()
    def sha(value):
        return isinstance(value,str) and len(value)==64 and all(c in '0123456789abcdef' for c in value)
    for path in (station_config_path,calibration_path,base_registration_path):
        if not isinstance(path,str) or not Path(path).is_absolute():raise ValueError('absolute source paths required')
    if not isinstance(description,str) or not 0<len(description)<=2_000_000:
        raise ValueError('bounded actual expanded URDF required')
    try:model=ET.fromstring(description)
    except ET.ParseError as error:raise ValueError('invalid expanded model XML') from error
    if model.tag!='robot':raise ValueError('actual expanded robot model required')
    config=json.loads(station_config_bytes);base=json.loads(base_registration_bytes)
    arm=config['arm'];calibration=arm['calibration'];cal_sha=digest(calibration_bytes)
    if (arm['id']!=owner.get('arm_id') or calibration.get('sha256')!=cal_sha
            or owner.get('calibration_sha256')!=cal_sha
            or Path(calibration['path'])!=Path(calibration_path)):
        raise ValueError('actual owner/config/calibration identity mismatch')
    sources=owner.get('owner_source_sha256')
    if (not isinstance(sources,dict) or not sources or
            any(not isinstance(k,str) or not k or not sha(v) for k,v in sources.items())):
        raise ValueError('complete actual owner source hashes required')
    session=owner.get('owner_session_id')
    if not isinstance(session,str) or not session:raise ValueError('current owner generation required')
    serial=device_info.get('serial_number');firmware=device_info.get('firmware_version')
    if not isinstance(serial,str) or not serial.strip() or not isinstance(firmware,str) or not firmware.strip():
        raise ValueError('actual DeviceInfo serial and firmware required')
    if (not isinstance(cloud_gid,str) or not 16<=len(cloud_gid)<=64 or len(cloud_gid)%2
            or any(c not in '0123456789abcdef' for c in cloud_gid) or not cloud_gid.strip('0')):
        raise ValueError('actual cloud publisher GID required')
    if (base.get('parent_frame')!='charuco_board' or base.get('child_frame')!='base_link'
            or base.get('transform_direction')!='world_from_base'):
        raise ValueError('canonical base registration direction/frames mismatch')
    pinned=validate_base_pin(world_from_base,base['world_from_base'])
    fault=owner_eligibility(owner)
    return {'schema':'so101-registration-source-v1','model_sha256':digest(description.encode()),
            'arm_id':owner['arm_id'],'owner_session_id':session,'calibration_sha256':cal_sha,
            'owner_source_sha256':dict(sources),'camera_serial':serial,'cloud_publisher_gid':cloud_gid,
            'station_config':station_config_path,'station_config_sha256':digest(station_config_bytes),
            'base_registration_path':base_registration_path,
            'base_registration_sha256':digest(base_registration_bytes),'world_from_base':pinned.tolist(),
            'calibration_path':calibration_path,'qualified':False,
            'fit_eligibility_at_capture':{'eligible':fault is None,'reason':fault},
            'base_registration_qualified':base.get('qualified',False),
            'base_registration_model_sha256':base.get('model_urdf_sha256')}
