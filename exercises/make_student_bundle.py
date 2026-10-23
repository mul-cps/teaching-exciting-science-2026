"""Build the offline learner ZIP, excluding instructor answers and hardware code."""
import argparse
from pathlib import Path
from zipfile import ZipFile, ZipInfo, ZIP_DEFLATED


def bundle_input_names(root):
    """Explicit learner allowlist; never include private manifests or instructor paths."""
    names = ['exercises/' + name for name in ('01_python.py', '02_robot.py', '03_vision.py', '04_sort.py', '05_visual_sort.py', 'check_setup.py', 'student_visuals.py')]
    names += ['workshop/' + name for name in ('__init__.py', '__main__.py', 'factory.py', 'live_observer.py')]
    names += ['notebooks/' + name for name in ('Robot_Candy_Factory_Student.ipynb', 'Robot_Candy_Factory_Live_SO101.ipynb', 'requirements.txt', 'launch.py', 'README.md')]
    names += [str(p.relative_to(root)) for p in sorted((root/'notebooks/assets').glob('*')) if p.is_file()]
    names += ['docs/student-exercises.md', 'docs/student-worksheet.md', 'docs/workshop-resources.md']
    # The learner Blockly route is simulation only: no ROS/physical adapter.
    names += ['tools/run_blockly_station.py', 'workshop/blockly_program.py',
              'workshop/blockly_recording.py', 'workshop/blockly_simulation_state.py', 'workshop/blockly_skills.py', 'workshop/blockly_client.py', 'workshop/blockly_ros_api.py', 'docs/blockly-student.md',
              'docs/activity-cards.md', 'docs/assets/fixed-fixture-sequence.svg',
              'docs/assets/blockly/simulation-complete.png']
    names += ['web/blockly-station/'+name for name in (
        'index.html','app.js','style.css','curriculum.json','ros-blocky-port.js',
        'README.md','LICENSE.ros-blocky','THIRD_PARTY.md')]
    names += [str(p.relative_to(root)) for p in sorted((root/'web/blockly-station/vendor').rglob('*')) if p.is_file()]
    names += ['web/blockly-station/examples/'+name for name in (
        '01_ev3.json','02_teach_replay.json','03_color_factory.json','04_arm_teach.json','05_reset_station.json')]
    names += [str(p.relative_to(root)) for p in sorted((root/'docs/assets/lecture/tm-samples').rglob('*.png'))]
    names += ['docs/' + name for name in ('student-handbook.docx', 'student-handbook.pdf', 'student-worksheet.docx', 'student-worksheet.pdf', 'activity-cards.docx', 'activity-cards.pdf') if (root/'docs'/name).exists()]
    return sorted(set(names))


def bundle_entries(root):
    entries={name:(root/name).read_bytes() for name in bundle_input_names(root)}
    entries['START-HERE.txt']=(root/'notebooks/README.md').read_bytes()
    return entries


def bundle_bytes(root):
    import io
    target=io.BytesIO()
    with ZipFile(target, 'w', ZIP_DEFLATED) as archive:
        for name,data in sorted(bundle_entries(root).items()):
            entry=ZipInfo(name,(2026,10,22,0,0,0))
            entry.compress_type=ZIP_DEFLATED
            archive.writestr(entry,data)
    return target.getvalue()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('docs/student-workshop.zip'))
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    expected=bundle_bytes(root)
    if args.check:
        if not args.output.exists() or args.output.read_bytes()!=expected:
            raise SystemExit('Learner bundle drift: '+str(args.output))
        print('Verified:',args.output.resolve())
        return
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_bytes(expected)
    print('Created:',args.output.resolve())


if __name__=='__main__':main()
