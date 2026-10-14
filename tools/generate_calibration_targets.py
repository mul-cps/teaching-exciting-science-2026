"""Generate dimensioned vector calibration targets; print at 100% scale."""
import argparse
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np


def vector_svg(image, width_mm, height_mm, margin_mm):
    height, width = image.shape
    sx = (width_mm - 2 * margin_mm) / width
    sy = (height_mm - 2 * margin_mm) / height
    rectangles = []
    for y, row in enumerate(image):
        edges = np.diff(np.r_[False, row < 128, False].astype(int))
        for start, stop in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)):
            rectangles.append(f'<rect x="{margin_mm + start*sx:.9f}" y="{margin_mm+y*sy:.9f}" width="{(stop-start)*sx:.9f}" height="{sy:.9f}"/>')
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{width_mm}mm" height="{height_mm}mm" viewBox="0 0 {width_mm} {height_mm}">'
            f'<rect width="{width_mm}" height="{height_mm}" fill="white"/><g fill="black">' + ''.join(rectangles) + '</g></svg>')


def generate(output, overwrite=False):
    output = Path(output)
    names = ['charuco.svg', 'charuco.json', 'apriltag-0.svg', 'apriltag-0.json', 'apriltag-1.svg', 'apriltag-1.json']
    if not overwrite and any((output / name).exists() for name in names):
        raise FileExistsError('Targets exist; use --overwrite explicitly')
    output.mkdir(parents=True, exist_ok=True)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = cv2.aruco.CharucoBoard((5, 7), .02, .014, dictionary)
    targets = [('charuco', board.generateImage((1000, 1400), marginSize=0, borderBits=1), 120, 160,
                {'kind': 'charuco', 'dictionary': 'DICT_4X4_50', 'squares': [5,7], 'square_length_m': .02,
                 'marker_length_m': .014, 'ids': board.getIds().ravel().tolist()})]
    for marker_id in (0, 1):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        targets.append((f'apriltag-{marker_id}', cv2.aruco.generateImageMarker(dictionary, marker_id, 400, borderBits=1), 60, 60,
                        {'kind': 'apriltag', 'dictionary': 'DICT_APRILTAG_36h11', 'ids': [marker_id], 'marker_length_m': .04}))
    for name, image, width, height, metadata in targets:
        svg = vector_svg(image, width, height, 10).encode()
        (output / f'{name}.svg').write_bytes(svg)
        metadata.update(schema='calibration-target-v1', units='metres', quiet_margin_m=.01,
                        print_size_m=[width/1000, height/1000], svg_file=f'{name}.svg',
                        svg_sha256=hashlib.sha256(svg).hexdigest(), print_instruction='Print at 100% scale; measure dimensions before use')
        (output / f'{name}.json').write_text(json.dumps(metadata, indent=2) + '\n')
    return names


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--overwrite', action='store_true')
    args = parser.parse_args()
    generate(args.output, args.overwrite)
