"""Packed observed RGB stays aligned with the same bounded finite XYZ samples."""
import ast
import inspect
from types import SimpleNamespace as NS
import numpy as np
import pytest
from workshop.digital_twin.point_cloud_samples import sampled_cloud, encode_rgb8, decode_rgb8


def message(points, words, *, name='rgb', datatype=7, endian=False, height=1, padding=0):
    points=np.asarray(points,np.float32);width=len(points)//height;step=20;row_step=width*step+padding
    raw=bytearray(row_step*height);prefix='>' if endian else '<'
    fields=[NS(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z'))]
    for i in range(3):
        np.ndarray((height,width),dtype=prefix+'f4',buffer=raw,offset=i*4,strides=(row_step,step))[:]=points[:,i].reshape(height,width)
    if words is not None:
        fields.append(NS(name=name,offset=16,datatype=datatype,count=1))
        np.ndarray((height,width),dtype=prefix+'u4',buffer=raw,offset=16,strides=(row_step,step))[:]=np.asarray(words).reshape(height,width)
    return NS(height=height,width=width,point_step=step,row_step=row_step,data=raw,fields=fields,is_bigendian=endian)


@pytest.mark.parametrize('endian',[False,True])
@pytest.mark.parametrize('datatype',[6,7])
@pytest.mark.parametrize('name',['rgb','rgba'])
def test_packed_rgb_endian_float_or_integer_bits_preserve_nan_color_and_xyz_alignment(endian,datatype,name):
    points=[[1,2,3],[np.nan,2,3],[4,5,6],[7,8,9]]
    # 0xffff0000 has a NaN float interpretation, but valid measured RGB bits.
    msg=message(points,[0xffff0000,0xff00ff00,0xff0000ff,0xff123456],name=name,datatype=datatype,endian=endian,height=2,padding=12)
    xyz,rgb,source=sampled_cloud(msg)
    assert xyz==pytest.approx(np.array([[1,2,3],[4,5,6],[7,8,9]]))
    assert np.array_equal(rgb,[[255,0,0],[0,0,255],[18,52,86]])
    assert source=='packed_'+name


def test_sampling_cap_keeps_corresponding_colors_and_handles_realsense_stale_row_step():
    p=np.column_stack((np.arange(100),np.zeros(100),np.ones(100)))
    msg=message(p,np.arange(100));msg.row_step=6144000
    xyz,rgb,source=sampled_cloud(msg,max_points=12)
    assert len(xyz)<=12 and np.array_equal(rgb[:,2],xyz[:,0].astype(np.uint8))


def test_actual_live_capture_first_point_decodes_measured_rgb_at_offset16():
    raw=bytes.fromhex('405b9b3eca8da0be30dd443f000000006d626100')
    msg=message([[0,0,0]],[0]);msg.data=raw
    _,rgb,source=sampled_cloud(msg)
    assert rgb.tolist()==[[97,98,109]] and source=='packed_rgb'


def test_fallback_only_if_color_absent_and_malformed_present_color_is_rejected():
    msg=message([[0,0,1]],None)
    assert sampled_cloud(msg)[1:] == (None,'uniform_fallback_rgb_absent')
    msg=message([[0,0,1]],[0]);msg.fields[-1].datatype=2
    with pytest.raises(ValueError,match='present RGB'):sampled_cloud(msg)


def test_browser_rgb_base64_is_exact_bounded_and_uint8():
    colors=np.array([[255,0,0],[0,128,255]],np.uint8)
    value=encode_rgb8(colors)
    assert len(value)==4*len(colors) and np.array_equal(decode_rgb8(value,2),colors)
    with pytest.raises(ValueError):decode_rgb8(value,3)
    with pytest.raises(ValueError):decode_rgb8('A'*48004,12001)
    with pytest.raises(ValueError):encode_rgb8(np.ones((2,3),float))


def test_shared_decoder_only_loops_over_three_xyz_fields_not_points():
    tree=ast.parse(inspect.getsource(sampled_cloud))
    loops=[n for n in ast.walk(tree) if isinstance(n,(ast.For,ast.While))]
    assert len(loops)==1 and ast.unparse(loops[0].iter)=="enumerate(('x', 'y', 'z'))"
    comps=[n for n in ast.walk(tree) if isinstance(n,(ast.ListComp,ast.SetComp,ast.DictComp,ast.GeneratorExp))]
    assert len(comps)==1 and ast.unparse(comps[0].generators[0].iter)=='message.fields'
