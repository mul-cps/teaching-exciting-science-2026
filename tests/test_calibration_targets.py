import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import cv2
import numpy as np
import pytest


def module(name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).parents[1]/'tools'/f'{name}.py')
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


generator = module('generate_calibration_targets')
solver = module('estimate_board_pose')


def raster(svg, scale=10):
    root = ET.fromstring(svg)
    width, height = [float(root.attrib[a].removesuffix('mm')) for a in ('width','height')]
    image = np.full((round(height*scale), round(width*scale)),255,np.uint8)
    for rectangle in root.findall('.//{http://www.w3.org/2000/svg}g/{http://www.w3.org/2000/svg}rect'):
        x,y,w,h = [float(rectangle.attrib[a]) for a in ('x','y','width','height')]
        image[round(y*scale):round((y+h)*scale),round(x*scale):round((x+w)*scale)] = 0
    return image


def test_svg_dimensions_detectable_ids_and_no_overwrite(tmp_path):
    generator.generate(tmp_path)
    metadata = json.loads((tmp_path/'charuco.json').read_text())
    root = ET.parse(tmp_path/'charuco.svg').getroot()
    assert (root.attrib['width'], root.attrib['height']) == ('120mm','160mm')
    image = raster((tmp_path/'charuco.svg').read_text())
    board = cv2.aruco.CharucoBoard((5,7),.02,.014,cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    corners, ids, _, markers = cv2.aruco.CharucoDetector(board).detectBoard(image)
    assert len(ids) == 24
    assert set(markers.ravel()) == set(metadata['ids'])
    for index in (0,1):
        svg = (tmp_path/f'apriltag-{index}.svg').read_text()
        assert ET.fromstring(svg).attrib['width'] == '60mm'
        _, ids, _ = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)).detectMarkers(raster(svg))
        assert ids.ravel().tolist() == [index]
    with pytest.raises(FileExistsError):
        generator.generate(tmp_path)


def test_synthetic_pose_and_insufficient_corners():
    board = cv2.aruco.CharucoBoard((5,7),.02,.014,cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    matrix = np.array([[600.,0,320],[0,600,240],[0,0,1]])
    rotation = np.array([.2,-.1,.05])
    translation = np.array([-.04,-.06,.6])
    ids = np.arange(24,dtype=np.int32).reshape(-1,1)
    corners = cv2.projectPoints(board.getChessboardCorners(), rotation, translation,matrix,np.zeros(5))[0]
    result = solver.solve_corners(board,corners,ids,matrix,np.zeros(5))
    actual = np.asarray(result['camera_from_target'])
    np.testing.assert_allclose(actual[:3,:3],cv2.Rodrigues(rotation)[0],atol=1e-5)
    np.testing.assert_allclose(actual[:3,3],translation,atol=1e-6)
    assert result['reprojection_p95_px'] < .001
    assert not result['qualified']
    with pytest.raises(ValueError):
        solver.solve_corners(board,corners[:5],ids[:5],matrix,np.zeros(5))
    with pytest.raises(ValueError):
        solver.solve_corners(board,None,None,matrix,np.zeros(5))


def test_rendered_board_pose(tmp_path):
    generator.generate(tmp_path)
    metadata = json.loads((tmp_path/'charuco.json').read_text())
    image = raster((tmp_path/'charuco.svg').read_text())
    result = solver.estimate(image,metadata,{'fx':1000,'fy':1000,'ppx':600,'ppy':800,'coeffs':[0]*5})
    np.testing.assert_allclose(np.asarray(result['camera_from_target'])[:3,3],[-.05,-.07,.1],atol=1e-4)
    assert len(result['corner_ids']) == 24
