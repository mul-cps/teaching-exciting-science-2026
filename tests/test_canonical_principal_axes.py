"""Analytic coordinate-frame policy; expected axes never call the candidate helper."""
import importlib
import sys
from pathlib import Path
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2/src/so101_station_objects'))

def canonical(spectrum, axes, count=8):
    from workshop.digital_twin.principal_axes import canonical_principal_axes
    return canonical_principal_axes(spectrum, axes, sample_count=count)

def test_rotated_tied_pair_uses_coordinate_axes():
    q = np.array([[.6, .8, 0.], [-.8, .6, 0.], [0., 0., 1.]])
    axes, tied = canonical([2., 2., 0.], q)
    np.testing.assert_array_equal(axes, np.eye(3))
    np.testing.assert_array_equal(tied, [True, True, False])

def test_public_footprint_tied_basis_is_canonical(monkeypatch):
    module = importlib.import_module('workshop.digital_twin.grasp_geometry')
    solve = np.linalg.svd
    def rotated(points, **kwargs):
        u, s, vt = solve(points, **kwargs)
        vt[:2] = np.array([[.6, .8, 0.], [-.8, .6, 0.]])
        return u, s, vt
    monkeypatch.setattr(np.linalg, 'svd', rotated)
    result = module.footprint_grasps([[-1,-1],[1,-1],[1,1],[-1,1]],
                                   dict(normal=np.array([0.,0.,-1.]), d=1.),
                                   dict(fx=100.,fy=100.,ppx=0.,ppy=0.), .02, .1)
    np.testing.assert_array_equal(result[0]['closing_axis'], [1.,0.,0.])
    assert result[0]['opening_m'] == pytest.approx(.02)

@pytest.mark.parametrize('dimension', [2, 3])
@pytest.mark.parametrize('angle', [0., .3, 1.2])
@pytest.mark.parametrize('reflection', [1., -1.])
def test_full_tie_identity_under_rotations_and_reflections(dimension, angle, reflection):
    c, s = np.cos(angle), np.sin(angle)
    q = np.eye(dimension); q[:2,:2] = [[c,s],[-s,c]]; q[0] *= reflection
    axes, mask = canonical(np.ones(dimension), q)
    np.testing.assert_array_equal(axes, np.eye(dimension))
    assert mask.all()

@pytest.mark.parametrize('spectrum', [[3.,1.,0.], [3.,2.,1.]])
def test_singletons_are_byte_preserved(spectrum):
    rows = np.array([[-.6,.8,-0.],[-.8,-.6,0.],[0.,-0.,-1.]])
    axes, mask = canonical(spectrum, rows)
    assert axes.tobytes() == rows.tobytes() and not mask.any()

@pytest.mark.parametrize('angle', [.1,.9,2.4])
def test_rank_one_nullspace_uses_untied_complement(angle):
    normal = np.array([1.,1.,0.])/np.sqrt(2)
    u = np.array([1.,-1.,0.])/np.sqrt(2); v = np.array([0.,0.,1.])
    rows = np.array([normal, np.cos(angle)*u+np.sin(angle)*v,
                     -np.sin(angle)*u+np.cos(angle)*v])
    axes, mask = canonical([4.,0.,0.], rows)
    # Z has projected norm1 and wins before the tied X/Y projected norm1/2.
    np.testing.assert_allclose(axes, [normal,v,u], atol=4e-16, rtol=0)
    np.testing.assert_array_equal(mask, [False,True,True])
    assert axes[0].tobytes() == rows[0].tobytes()

def test_non_chaining_and_immediate_spectrum_boundary():
    eps = np.finfo(float).eps
    q = np.array([[.6,.8,0.],[-.8,.6,0.],[0.,0.,1.]])
    _, mask = canonical([1.,1.-16*eps,1.-32*eps], q, count=1)
    np.testing.assert_array_equal(mask,[True,True,False])
    boundary = 1.-24*eps  # dimension3 gives exactly24eps at sample_count1.
    for value, expected in [(np.nextafter(boundary,1.),True),(boundary,True),
                             (np.nextafter(boundary,0.),False)]:
        _, mask = canonical([1.,value,0.],q,count=1)
        assert bool(mask[0]) == expected

def test_anchor_equal_norm_prefers_coordinate_order():
    normal = np.ones(3)/np.sqrt(3)
    u = np.array([1.,-1.,0.])/np.sqrt(2)
    v = np.cross(normal,u)
    axes, _ = canonical([2.,2.,0.],[u,v,normal])
    # All initial norms tie: X then Y, each with positive chosen coordinate.
    expected_x = np.array([2.,-1.,-1.])/np.sqrt(6)
    expected_y = np.array([0.,1.,-1.])/np.sqrt(2)
    np.testing.assert_allclose(axes[:2], [expected_x,expected_y], atol=5e-16,rtol=0)


def objects_module():
    return importlib.import_module('so101_station_objects.objects')

def synthetic_component(monkeypatch, *, rectangle=False):
    module = objects_module()
    x,y = np.meshgrid(np.linspace(-.02,.02,10),np.linspace(-.01 if rectangle else -.02,.01 if rectangle else .02,10))
    points = np.column_stack((x.ravel(), y.ravel(), np.full(x.size,.78)))
    monkeypatch.setattr(module,'camera_points',lambda *a: points.copy())
    transform = np.diag([1.,-1.,-1.,1.]); transform[2,3]=.8
    intrinsics = dict(fx=100.,fy=100.,ppx=0.,ppy=0.)
    contour = np.array([[-2,-2],[2,-2],[2,2],[-2,2]],dtype=float)
    return module, (contour,np.zeros((x.size,2)),np.full(x.size,.78),intrinsics,np.zeros(5),
                    transform,dict(normal=np.array([0.,0.,1.]),d=0.),'red',.1,None,.25)

def rotate_svd(monkeypatch, angle):
    original = np.linalg.svd
    def solve(a, **kwargs):
        u,s,vt = original(a,**kwargs)
        if len(s)>=2 and abs(s[0]-s[1]) <= 8*np.finfo(float).eps*max(a.shape)*max(abs(s)):
            r=np.array([[np.cos(angle),np.sin(angle)],[-np.sin(angle),np.cos(angle)]])
            vt[:2] = r @ vt[:2]
        return u,s,vt
    monkeypatch.setattr(np.linalg,'svd',solve)


def test_support_box_and_grasp_public_outputs_ignore_tied_solver_rotation(monkeypatch):
    module,args=synthetic_component(monkeypatch)
    first=module._component_geometry(*args)
    rotate_svd(monkeypatch,.71)
    second=module._component_geometry(*args)
    for field in ['position','orientation_matrix','dimensions_m','dimension_uncertainty_m']:
        np.testing.assert_allclose(second[field],first[field],rtol=0,atol=1e-15)
    np.testing.assert_allclose(second['orientation_matrix'],np.eye(3),rtol=0,atol=1e-15)
    np.testing.assert_allclose(second['grasps'][0]['closing_axis'],first['grasps'][0]['closing_axis'],rtol=0,atol=1e-15)
    assert second['grasps'][0]['opening_m']==pytest.approx(first['grasps'][0]['opening_m'],abs=1e-15)
    assert not second['execution_eligible'] and not second['ik_validated']


def test_validation_complete_output_under_tied_svd_basis(monkeypatch):
    from so101_station_objects.colored_object_validation import validate_colored_component
    rgb=np.full((10,10,3),[255,0,0],dtype=np.uint8)
    arguments=(rgb,np.full((10,10),.5),np.ones((10,10),bool),dict(fx=100.,fy=100.,ppx=4.5,ppy=4.5))
    first=validate_colored_component(*arguments)
    rotate_svd(monkeypatch,.63)
    second=validate_colored_component(*arguments)
    assert second==first
    np.testing.assert_allclose(second.metrics['visible_extents_m'],[.045,.045,0.],atol=1e-15,rtol=0)


def test_visible_covariance_uses_board_frame_not_rotated_equal_moments(monkeypatch):
    module=objects_module()
    # Equal second moments but different percentile extents: two X extremes,
    # eight Y extremes, each replicated3 times; original evidence is unchanged.
    xy=np.array([[2.,0.],[-2.,0.]]+[[0.,1.],[0.,-1.]]*4)*.01
    points=np.column_stack((np.repeat(xy,3,axis=0),np.full(30,.5)))
    monkeypatch.setattr(module,'camera_points',lambda *a:points.copy())
    args=(np.zeros((30,2)),np.full(30,.5),{},None,np.eye(4),None,'red')
    first=module._visible_component_geometry(*args)
    original=np.linalg.eigh
    def rotated(a):
        values,columns=original(a);r=np.array([[.6,.8],[-.8,.6]])
        columns[:,-2:]=columns[:,-2:] @ r
        return values,columns
    monkeypatch.setattr(np.linalg,'eigh',rotated)
    second=module._visible_component_geometry(*args)
    np.testing.assert_allclose(second['orientation_matrix'],np.eye(3),atol=4e-16,rtol=0)
    np.testing.assert_allclose(second['dimensions_m'],[.04,.02,.001],atol=1e-15,rtol=0)
    for k in first:
        if isinstance(first[k],np.ndarray):np.testing.assert_allclose(second[k],first[k],atol=1e-15,rtol=0)
        else:assert second[k]==first[k]


@pytest.mark.parametrize('steps,first_wins',[(0,True),(1,True),(63,True),(64,True),(65,False),(1000,False)])
def test_final_grasp_ranking_roundoff_boundary(monkeypatch,steps,first_wins):
    module,args=synthetic_component(monkeypatch,rectangle=True)
    # Width .0625 has spacing2**-57 below it;32eps*width equals64 such ULPs.
    width=.0625
    narrower=width
    for _ in range(steps):narrower=np.nextafter(narrower,0.)
    def candidates(*a,**kw):
        return [dict(opening_m=w,center=np.array([0.,0.,.79]),closing_axis=np.array(axis),contacts=np.zeros((2,3)))
                for w,axis in [(width,[1.,0.,0.]),(narrower,[0.,1.,0.])]]
    monkeypatch.setattr(module,'footprint_grasps',candidates)
    result=module._component_geometry(*args)
    np.testing.assert_array_equal(result['grasps'][0]['closing_axis'],[1.,0.,0.] if first_wins else [0.,-1.,0.])


def test_maxopening_filter_remains_exact_at_nextafter():
    from workshop.digital_twin.grasp_geometry import footprint_grasps
    args=([[-1,-1],[1,-1],[1,1],[-1,1]],dict(normal=np.array([0.,0.,-1.]),d=1.),dict(fx=100.,fy=100.,ppx=0.,ppy=0.),.02)
    exact=footprint_grasps(*args,.02)
    assert len(exact)==2
    assert footprint_grasps(*args,np.nextafter(.02,0.))==[]
    assert len(footprint_grasps(*args,np.nextafter(.02,1.)))==2


def test_separated_complete_report_unchanged_from_singleton_policy(monkeypatch):
    module=objects_module()
    rgb=np.full((120,160,3),128,dtype=np.uint8);rgb[50:60,65:90]=[255,0,0]
    depth=np.full((120,160),800,dtype=np.uint16);depth[50:60,65:90]=780
    transform=np.diag([1.,-1.,-1.,1.]);transform[2,3]=.8
    args=(rgb,depth,dict(fx=500.,fy=500.,ppx=80.,ppy=60.),np.zeros(5),.001,transform)
    first=module.analyze_rgbd(*args)
    def singleton(s,a,**kw):return a,np.zeros(len(s),bool)
    for name in ['workshop.digital_twin.grasp_geometry','so101_station_objects.colored_object_validation','so101_station_objects.objects']:
        monkeypatch.setattr(importlib.import_module(name),'canonical_principal_axes',singleton)
    second=module.analyze_rgbd(*args)
    def equal(a,b):
        if isinstance(a,np.ndarray):np.testing.assert_array_equal(a,b)
        elif isinstance(a,dict):
            assert a.keys()==b.keys()
            for k in a:equal(a[k],b[k])
        elif isinstance(a,(tuple,list)):
            assert len(a)==len(b)
            for x,y in zip(a,b):equal(x,y)
        else:assert a==b
    assert first['objects']
    equal(first,second)

@pytest.mark.parametrize('dimension',[2,3])
def test_permuted_reflected_zero_spectrum_is_full_identity(dimension):
    rows=np.eye(dimension)[::-1].copy();rows[0]*=-1
    axes,mask=canonical(np.zeros(dimension),rows)
    np.testing.assert_array_equal(axes,np.eye(dimension));assert mask.all()


def test_tied_visible_signs_are_not_undone_by_dominant_coordinate(monkeypatch):
    module=objects_module();eps=np.finfo(float).eps
    n=np.array([np.sqrt(.5+eps),np.sqrt(.5-eps),0.])
    u=np.array([n[1],-n[0],0.]);z=np.array([0.,0.,1.])
    a,b=np.meshgrid(np.linspace(-.02,.02,10),np.linspace(-.02,.02,10))
    points=np.array([0.,0.,.5])+a.ravel()[:,None]*u+b.ravel()[:,None]*z
    monkeypatch.setattr(module,'camera_points',lambda *a:points.copy())
    original=np.linalg.eigh
    def alternative(covariance):
        values,_=original(covariance)
        return values,np.column_stack((n,(u+z)/np.sqrt(2),(u-z)/np.sqrt(2)))
    monkeypatch.setattr(np.linalg,'eigh',alternative)
    result=module._visible_component_geometry(np.zeros((100,2)),np.full(100,.5),{},None,np.eye(4),None,'red')
    # Canonical second row has negative dominant Y, but positive X anchor.
    np.testing.assert_allclose(result['orientation_matrix'],np.column_stack((z,u,n)),atol=4e-16,rtol=0)


def test_projected_anchor_roundoff_boundary_neighbors():
    eps=np.finfo(float).eps
    # Search adjacent input encodings straddling the declared anchor gap. The
    # expected second direction follows the analytic XY plane complement.
    first=np.sqrt(.5+32*eps)
    witnesses=[]
    for delta in range(-8,9):
        x=first
        for _ in range(abs(delta)):x=np.nextafter(x,np.inf if delta>0 else 0.)
        y=np.sqrt(1.-x*x);normal=np.array([x,y,0.]);u=np.array([y,-x,0.]);z=np.array([0.,0.,1.])
        # Explicit analytic coordinate projections, independent of candidate.
        px=np.array([1.-x*x,-x*y,0.]);py=np.array([-x*y,1.-y*y,0.])
        gap=float(py@py-px@px);prefer_x=gap<=64*eps
        axes,_=canonical([3.,0.,0.],[normal,u,z])
        expected=u if prefer_x else -u
        np.testing.assert_allclose(axes[2],expected,atol=5e-16,rtol=0)
        witnesses.append(prefer_x)
    assert any(witnesses) and not all(witnesses)
