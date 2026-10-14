"""Observable RGB-D front-surface mesh; never complete unseen object backs."""
from pathlib import Path
import numpy as np


def surface_mesh(points, rgb, valid, step=3, max_edge=.022):
    points = points[::step, ::step]; colors = rgb[::step, ::step]; valid = valid[::step, ::step]
    height, width = valid.shape
    indices = np.arange(height * width).reshape(height, width)
    a, b, c, d = indices[:-1, :-1], indices[:-1, 1:], indices[1:, :-1], indices[1:, 1:]
    faces = np.concatenate((np.stack((a, c, b), axis=-1).reshape(-1, 3), np.stack((b, c, d), axis=-1).reshape(-1, 3)))
    flat = points.reshape(-1, 3)
    selected = valid.ravel()[faces].all(axis=1)
    triangles = flat[faces]
    for left, right in ((0, 1), (1, 2), (2, 0)):
        selected &= np.linalg.norm(triangles[:, left] - triangles[:, right], axis=1) < max_edge
    # RGB-D optical winding may face away from the table-world viewer; renderer uses two-sided material.
    return flat.astype(np.float32), faces[selected], colors.reshape(-1, 3)


def write_surface(output, points, rgb, valid):
    output = Path(output)
    vertices, faces, colors = surface_mesh(points, rgb, valid)
    np.savez_compressed(output / 'measured-surface.npz', vertices=vertices, faces=faces, colors=colors)
    # MuJoCo mesh texture UVs reference the exact measured photograph.
    h, w = valid[::3, ::3].shape
    uv = np.stack(np.meshgrid(np.arange(w)*3/(rgb.shape[1]-1), 1-np.arange(h)*3/(rgb.shape[0]-1)), axis=-1).reshape(-1, 2)
    with (output / 'measured-surface.obj').open('w') as stream:
        for vertex in vertices: stream.write('v ' + ' '.join(map(str, vertex)) + '\n')
        for coord in uv: stream.write('vt ' + ' '.join(map(str, coord)) + '\n')
        for face in faces: stream.write('f ' + ' '.join(f'{i+1}/{i+1}' for i in face) + '\n')
    from PIL import Image
    Image.fromarray(rgb).save(output / 'measured-texture.png')
    return {'vertices': len(vertices), 'triangles': len(faces), 'maximum_triangle_edge_m': .022,
            'description': 'visible RGB-D surfaces only; no inferred backs; arm mask removed; texture is measured photograph'}


def colored_pieces(rgb, world, valid):
    """Report observed color components in sorter ROI; classification is approximate."""
    from scipy.ndimage import label
    y,x=np.indices(valid.shape)
    r,g,b=np.moveaxis(rgb.astype(float),-1,0)
    roi=valid & (x<300) & (y>140) & (y<360)
    masks={'red':(r>g*1.35)&(r>b*1.35)&(r>85),
           'green':(g>r*1.25)&(g>b*1.12)&(g>65),
           'blue':(b>r*1.3)&(b>g*1.05)&(b>65),
           'yellow':(r>105)&(g>95)&(b<np.minimum(r,g)*.7)}
    components=[]
    for color,mask in masks.items():
        labels,count=label(mask & roi)
        for index in range(1,count+1):
            selected=labels==index
            if selected.sum()<25: continue
            components.append({'color_candidate':color,'pixel_count':int(selected.sum()),
                               'world_surface_centroid_m':np.median(world[selected],axis=0).tolist(),
                               'pixel_bounds':[int(x[selected].min()),int(y[selected].min()),int(x[selected].max()),int(y[selected].max())]})
    return {'components':components,'status':'observed color surfaces, not object identities or qualified grasp poses'}
