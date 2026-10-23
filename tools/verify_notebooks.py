"""Execute canonical notebooks in fresh offline kernels; assert teaching outcomes."""
import argparse
import hashlib
import os
import tempfile
from contextlib import contextmanager
from pathlib import Path
import nbformat
from nbclient import NotebookClient
from nbconvert import HTMLExporter
ROOT=Path(__file__).resolve().parents[1]


def execute(n):
    return NotebookClient(n,timeout=90,kernel_name='python3',resources={'metadata':{'path':str(ROOT/'notebooks')}}).execute()


@contextmanager
def capture_environment(**values):
    previous = {key:os.environ.get(key) for key in values}
    os.environ.update(values)
    try: yield
    finally:
        for key, value in previous.items():
            if value is None: os.environ.pop(key, None)
            else: os.environ[key] = value


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('/tmp/workshop-beginner-notebook-check'))
    args=parser.parse_args(); args.output.mkdir(parents=True,exist_ok=True); report=[]
    for edition in ('Student','Instructor','Live_SO101'):
        p=ROOT/'notebooks'/f'Robot_Candy_Factory_{edition}.ipynb'; n=nbformat.read(p,as_version=4)
        with capture_environment(WORKSHOP_LIVE="0"):
            execute(n)
        text='\n'.join(o.get('text','') for cell in n.cells if cell.cell_type=='code' for o in cell.outputs)
        assert 'Relative prediction: 35 degrees' in text and 'Absolute prediction: 15 degrees' in text
        assert 'invented' in text
        if edition=='Live_SO101':
            assert sum('image/png' in o.get('data',{}) for cell in n.cells if cell.cell_type=='code' for o in cell.outputs) == 2
        if edition!='Live_SO101':
            assert "{'left': 2, 'right': 2, 'reject': 1}" in text
            assert 'STOP: missing observation' in text
            assert "['Ready', 'ev3:eject', 'ev3:left_end']" in text
            assert 'SIMULATION only: True' in text
            assert 'Synthetic fixed-fixture rehearsal:' in text
            assert 'Physical evidence: False' in text
        nbformat.write(n,args.output/p.name)
        html,_=HTMLExporter().from_notebook_node(n); (args.output/(p.stem+'.html')).write_text(html)
        report.append(f'{p.name} sha256={hashlib.sha256(p.read_bytes()).hexdigest()} fresh_kernel=PASS no_ROS=PASS')
    # A real bounded observer wrapper subprocess fails explicitly in an isolated
    # fixture. No ROS installation, discovery or devices are involved.
    with tempfile.TemporaryDirectory(prefix='workshop-observer-unavailable-') as folder:
        wrapper=Path(folder)/'missing-observer'
        wrapper.write_text('#!/bin/sh\necho "isolated LIVE unavailable fixture" >&2\nexit 2\n')
        wrapper.chmod(0o755)
        live=nbformat.read(ROOT/'notebooks/Robot_Candy_Factory_Live_SO101.ipynb',as_version=4)
        live.cells.append(nbformat.v4.new_code_cell('assert LIVE is True\nassert report["samples"] == []\nassert report["mode"] == "LIVE unavailable — no measurements"'))
        with capture_environment(WORKSHOP_LIVE='1',WORKSHOP_ROS_WRAPPER=str(wrapper),WORKSHOP_ROS_WRAPPER_COMMAND='run'):
            execute(live)
        text='\n'.join(o.get('text','') for cell in live.cells if cell.cell_type=='code' for o in cell.outputs)
        assert 'LIVE unavailable — no measurements' in text
        assert 'isolated LIVE unavailable fixture' in text
        nbformat.write(live,args.output/'live-unavailable-fixture.ipynb')
        report.append('Live notebook launch flag: actual wrapper subprocess fails explicitly; empty measurements and no DEMO fallback: PASS')
    base=nbformat.read(ROOT/'notebooks/Robot_Candy_Factory_Student.ipynb',as_version=4)
    def source(eid): return next(c.source for c in base.cells if c.metadata.get('example_id')==eid)
    checks=nbformat.v4.new_notebook(cells=[nbformat.v4.new_code_cell(source('setup'))])
    variants=[('["red", "red", "blue", "unknown"]','{"red": "right", "blue": "left"}',{'left':1,'right':2,'reject':1}),('["green", "red", None, 42, [], {}, "unknown", ""]','{"red": "left", "blue": "right", "green": "left"}',{'left':2,'right':0,'reject':6}),('["red", "blue", "red", "unknown", "blue"]','{"red": "middle", "blue": "right"}',{'left':0,'right':2,'reject':3})]
    for objects,rules,expected in variants:
        code=source('extension').replace('OBJECTS = ["red", "blue", "red", "unknown", "blue"]','OBJECTS = '+objects).replace('RULES = {"red": "left", "blue": "right"}','RULES = '+rules)+'\nassert factory.counts == '+repr(expected)
        checks.cells += [nbformat.v4.new_code_cell(code),nbformat.v4.new_code_cell(code)]
    execute(checks); nbformat.write(checks,args.output/'challenge-variants.ipynb')
    report.append('Optional routing variants: asymmetric swap, green, invalid reports, invalid destination, repeated cells reset: PASS')
    (args.output/'verification.txt').write_text('\n'.join(report)+'\n');print('\n'.join(report))

if __name__=='__main__': main()
