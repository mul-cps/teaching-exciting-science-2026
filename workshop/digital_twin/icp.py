"""Bounded point-to-plane CAD / measured RGB-D registration, offline only."""
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def oriented_cad_surface(model, data, count=16000, seed=23, body_name=None):
    """Vectorized official visual samples; optional body isolates fixed-base CAD."""
    import mujoco
    body=None if body_name is None else mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,body_name)
    if body is not None and body<0:raise ValueError('unknown CAD sampling body')
    rng=np.random.default_rng(seed);triangles=[];geoms=[]
    for index in range(model.ngeom):
        if model.geom_group[index]!=2 or model.geom_type[index]!=mujoco.mjtGeom.mjGEOM_MESH:continue
        if body is not None and model.geom_bodyid[index]!=body:continue
        mesh=model.geom_dataid[index];start,length=model.mesh_vertadr[mesh],model.mesh_vertnum[mesh]
        first,faces=model.mesh_faceadr[mesh],model.mesh_facenum[mesh]
        triangle=model.mesh_vert[start:start+length][model.mesh_face[first:first+faces]]
        triangles.append(triangle);geoms.append(np.full(len(triangle),index))
    if not triangles:raise ValueError('no visual CAD meshes for sampling body')
    triangles=np.concatenate(triangles);geoms=np.concatenate(geoms)
    normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]);area=np.linalg.norm(normals,axis=1)
    chosen=rng.choice(len(triangles),count,p=area/area.sum());triangle=triangles[chosen];ids=geoms[chosen]
    uv=rng.random((count,2));uv[uv.sum(axis=1)>1]=1-uv[uv.sum(axis=1)>1]
    local=triangle[:,0]+uv[:,:1]*(triangle[:,1]-triangle[:,0])+uv[:,1:]*(triangle[:,2]-triangle[:,0])
    normal=normals[chosen]/np.maximum(area[chosen,None],1e-12)
    rotations=data.geom_xmat[ids].reshape(-1,3,3)
    world=np.einsum('nij,nj->ni',rotations,local)+data.geom_xpos[ids]
    normal=np.einsum('nij,nj->ni',rotations,normal)
    base=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'arm_base')
    basis=data.xmat[base].reshape(3,3)
    return (world-data.xpos[base])@basis,normal@basis


def follower_cloud_mask(rgb, optical_points, valid, world_points=None):
    """Visible follower, including its stationary native base; excludes distant wall."""
    from scipy.ndimage import binary_closing,binary_fill_holes
    y,x=np.indices(valid.shape)
    roi=(x>285)&(x<515)&(y>90)&(y<355)
    white=(rgb.min(axis=-1)>105)&(rgb.max(axis=-1).astype(float)-rgb.min(axis=-1)<45)
    mask=binary_fill_holes(binary_closing(white&roi,iterations=3))
    if world_points is not None: mask &= world_points[...,2]>.006
    return mask&valid&(optical_points[...,2]>.35)&(optical_points[...,2]<.72)


def point_to_plane_icp(cad, normals, observed, initial_translation, yaw_prior_deg, iterations=24, yaw_radius_deg=30, initial_yaw_deg=None):
    """Correspondences observed->CAD; robust point-to-plane, hard mount-yaw prior.

    Table up is fixed, so yaw cannot silently reverse the user-selected branch.
    Partial observed surfaces do not force fabricated unseen backs to match.
    """
    cad,normals,observed=map(lambda value:np.asarray(value,float),(cad,normals,observed))
    if len(cad)<30 or len(observed)<30:raise ValueError('insufficient point-to-plane registration data')
    observed=observed[::max(1,(len(observed)+4999)//5000)]
    tree=cKDTree(cad);yaw=np.deg2rad(yaw_prior_deg);radius=np.deg2rad(yaw_radius_deg)
    initial_yaw=yaw if initial_yaw_deg is None else np.deg2rad(initial_yaw_deg)
    if not yaw-radius<=initial_yaw<=yaw+radius:raise ValueError('initial yaw outside mount prior')
    parameters=np.r_[initial_translation,initial_yaw];history=[]
    def distances(parameters):
        rotation=Rotation.from_euler('z',parameters[3]).as_matrix()
        local=(observed-parameters[:3])@rotation
        return tree.query(local,workers=1)
    before,_=distances(parameters)
    for iteration in range(iterations):
        distance,indices=distances(parameters)
        cutoff=max(.012,.055*(.94**iteration))
        keep=distance<cutoff
        if keep.sum()<30:break
        source,target,normal=cad[indices[keep]],observed[keep],normals[indices[keep]]
        def residual(candidate):
            rotation=Rotation.from_euler('z',candidate[3]).as_matrix()
            vectors=source@rotation.T+candidate[:3]-target
            return np.einsum('ni,ni->n',vectors,normal@rotation.T)
        fit=least_squares(residual,parameters,
                          bounds=([-.5,-.5,-.1,yaw-radius],[.5,.5,.4,yaw+radius]),
                          loss='soft_l1',f_scale=.004,max_nfev=18)
        movement=float(np.linalg.norm(fit.x[:3]-parameters[:3]));angle=float(abs(fit.x[3]-parameters[3]))
        parameters=fit.x;distance,_=distances(parameters)
        history.append({'iteration':iteration,'correspondences':int(keep.sum()),'cutoff_m':cutoff,
                        'euclidean_median_m':float(np.median(distance)),'euclidean_p95_m':float(np.percentile(distance,95)),
                        'point_to_plane_median_m':float(np.median(np.abs(residual(parameters)))),
                        'translation_step_m':movement,'yaw_step_deg':float(np.rad2deg(angle))})
        if movement<1e-5 and angle<1e-5:break
    after,_=distances(parameters);transform=np.eye(4)
    transform[:3,:3]=Rotation.from_euler('z',parameters[3]).as_matrix();transform[:3,3]=parameters[:3]
    return {'world_from_arm_base':transform.tolist(),'position_m':parameters[:3].tolist(),
            'yaw_deg':float(np.rad2deg(parameters[3])%360),'initial_position_m':list(initial_translation),
            'initial_yaw_deg':float(np.rad2deg(initial_yaw)), 'yaw_prior_deg':yaw_prior_deg,'yaw_radius_deg':yaw_radius_deg,
            'table_up_fixed':True,'observed_samples':len(observed),'cad_samples':len(cad),'iterations':history,
            'before_median_m':float(np.median(before)),'before_p95_m':float(np.percentile(before,95)),
            'after_median_m':float(np.median(after)),'after_p95_m':float(np.percentile(after,95)),
            'fraction_within_10mm':float(np.mean(after<.01)),
            'status':'measured-point registration candidate; single pose, approximate segmentation; globalIK unqualified'}


def fit_fixed_base(cad, normals, observed_world, world_from_camera, initial_translation,
                   yaw_prior_deg, iterations=30, yaw_radius_deg=30):
    """Stationary base-only registration with withheld points and seed sensitivity.

    Caller supplies observed fixed-base points after temporal/ROI segmentation.
    CAD must contain only arm_base visuals; moving-link geometry is excluded.
    No fitted joint offsets, physical activation, or IK qualification is implied.
    """
    transform=np.asarray(world_from_camera,float)
    if transform.shape!=(4,4) or not np.all(np.isfinite(transform)):
        raise ValueError('world_from_camera must be a finite rigid transform')
    if (not np.allclose(transform[3],[0,0,0,1]) or
            not np.allclose(transform[:3,:3].T@transform[:3,:3],np.eye(3),atol=1e-6) or
            not np.isclose(np.linalg.det(transform[:3,:3]),1,atol=1e-6)):
        raise ValueError('world_from_camera must be a proper rigid transform')
    observed=np.asarray(observed_world,float)
    if len(observed)<80:raise ValueError('insufficient fixed-base training and holdout points')
    training,holdout=observed[::2],observed[1::2]
    seeds=[yaw_prior_deg-yaw_radius_deg*.5,yaw_prior_deg,yaw_prior_deg+yaw_radius_deg*.5]
    fits=[point_to_plane_icp(cad,normals,training,initial_translation,yaw_prior_deg,
                            iterations,yaw_radius_deg,seed) for seed in seeds]
    best=min(fits,key=lambda fit:fit['after_median_m'])
    world_base=np.asarray(best['world_from_arm_base'])
    distances=cKDTree(np.asarray(cad)).query((holdout-world_base[:3,3])@world_base[:3,:3],workers=1)[0]
    positions=np.asarray([fit['position_m'] for fit in fits])
    angles=np.unwrap(np.deg2rad([fit['yaw_deg'] for fit in fits]))
    return {'camera_from_arm_base':(np.linalg.inv(transform)@world_base).tolist(),
            'world_from_arm_base':world_base.tolist(),'fit':best,
            'holdout':{'samples':len(holdout),'median_m':float(np.median(distances)),
                       'p95_m':float(np.percentile(distances,95)),
                       'fraction_within_10mm':float(np.mean(distances<.01))},
            'sensitivity':{'seed_translation_axis_span_m':np.ptp(positions,axis=0).tolist(),
                           'seed_yaw_span_deg':float(np.rad2deg(np.ptp(angles)))},
            'global_ik_qualified':False,
            'limitations':['alternating-point holdout shares the same capture; not independent pose validation',
                           'seed sensitivity is not a confidence interval',
                           'caller fixed-base segmentation and table-plane uncertainty remain'],
            'status':'fixed-base camera registration candidate; moving joints excluded; globalIK unqualified'}
