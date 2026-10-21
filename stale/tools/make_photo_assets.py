"""Crop reusable conceptual workshop photos from AI-generated overview illustrations.

These are NOT photographs of our actual robot, camera or conveyor. They are
illustrative assets and must be labeled as such in any published material.
Requires Pillow only to rebuild; generated crops are included in the repo.
"""
from pathlib import Path
from PIL import Image, ImageOps

ROOT = Path(__file__).resolve().parents[1]
DIR = ROOT / 'docs' / 'assets'
SOURCE = DIR / 'sources'
ARCHIVE = DIR / 'archive'
ARCHIVE.mkdir(exist_ok=True)

# source filename, (left, top, right, bottom), output filename, suitable use
CROPS = [
    ('generated-overview-02.png', (12, 127, 853, 453), 'scene-sorting-station.jpg'),
    ('generated-overview-02.png', (868, 128, 1070, 435), 'scene-gripper.jpg'),
    ('generated-overview-02.png', (1080, 128, 1525, 446), 'scene-vision-monitor.jpg'),
    ('generated-overview-02.png', (17, 136, 438, 450), 'scene-ev3-conveyor.jpg'),
    ('generated-overview-01.png', (1227, 501, 1520, 714), 'scene-sort-result.jpg'),
    ('generated-overview-01.png', (615, 506, 880, 648), 'scene-teleoperation.jpg'),
]


def main():
    for name, crop, dest in CROPS:
        with Image.open(SOURCE / name) as img:
            assert img.width >= crop[2] and img.height >= crop[3], (name, crop, img.size)
            part = ImageOps.exif_transpose(img).crop(crop).convert('RGB')
            part.save(ARCHIVE / dest, quality=91, optimize=True)
        print(dest, part.size)


if __name__ == '__main__':
    main()
