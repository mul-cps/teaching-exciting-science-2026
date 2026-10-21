"""Build the offline learner ZIP, excluding instructor answers and hardware code."""
import argparse
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('student-workshop.zip'))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    names = ['exercises/' + name for name in ('01_python.py', '02_robot.py', '03_vision.py', '04_sort.py', '05_visual_sort.py', 'check_setup.py', 'student_visuals.py')]
    names += ['workshop/' + name for name in ('__init__.py', '__main__.py', 'factory.py', 'live_observer.py')]
    names += ['notebooks/' + name for name in ('Robot_Candy_Factory_Student.ipynb', 'Robot_Candy_Factory_Live_SO101.ipynb', 'requirements.txt', 'launch.py', 'README.md')]
    names += [str(p.relative_to(root)) for p in sorted((root/'notebooks/assets').glob('*')) if p.is_file()]
    names += ['docs/student-exercises.md', 'docs/student-worksheet.md', 'docs/workshop-resources.md']
    names += ['docs/' + name for name in ('student-handbook.docx', 'student-handbook.pdf', 'student-worksheet.docx', 'student-worksheet.pdf') if (root/'docs'/name).exists()]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with ZipFile(args.output, 'w', ZIP_DEFLATED) as archive:
        for name in names:
            archive.write(root/name, name)
        archive.writestr('START-HERE.txt', (root/'notebooks/README.md').read_text())
    print('Created:', args.output.resolve())


if __name__ == '__main__':
    main()
