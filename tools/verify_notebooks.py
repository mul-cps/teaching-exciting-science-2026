"""Execute each notebook in a fresh kernel and export visible results to evidence."""
import argparse
import hashlib
from pathlib import Path
import nbformat
from nbclient import NotebookClient
from nbconvert import HTMLExporter
ROOT = Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('/home/bjoern/.local/share/exciting-science/station/evidence/2026-10-21-sorting/workshop-beginner-rebuild/notebooks'))
    args=parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    report=[]
    for edition in ('Student','Instructor'):
        source=ROOT/'notebooks'/f'Robot_Candy_Factory_{edition}.ipynb'
        n=nbformat.read(source, as_version=4)
        NotebookClient(n, timeout=90, kernel_name='python3', resources={'metadata': {'path': str(ROOT/'notebooks')}}).execute()
        outputs=[o for c in n.cells if c.cell_type=='code' for o in c.outputs]
        pictures=sum(o.output_type=='display_data' and 'image/png' in o.get('data',{}) for o in outputs)
        assert pictures >= 6, (edition, pictures)
        text='\n'.join(o.get('text','') for o in outputs)
        assert "{'left': 2, 'right': 2, 'reject': 1}" in text
        nbformat.write(n,args.output/source.name)
        body,_=HTMLExporter().from_notebook_node(n)
        (args.output/(source.stem+'.html')).write_text(body)
        report.append(f'{source.name} sha256={hashlib.sha256(source.read_bytes()).hexdigest()} code_cells={sum(c.cell_type=="code" for c in n.cells)} figures={pictures} fresh_kernel=PASS')
    base = nbformat.read(ROOT/'notebooks/Robot_Candy_Factory_Student.ipynb', as_version=4)
    setup = next(c.source for c in base.cells if c.cell_type=='code')
    factory = next(c.source for c in base.cells if c.cell_type=='code' and 'factory = Factory(' in c.source)
    variants = [
        ('flipped asymmetric', '["red", "red", "blue", "unknown"]', '{"red": "right", "blue": "left"}', {'left':1,'right':2,'reject':1}),
        ('green and invalid reports', '["green", "red", None, 42, [], {}, "unknown", ""]', '{"red": "left", "blue": "right", "green": "left"}', {'left':2,'right':0,'reject':6}),
        ('invalid destination', '["red", "blue", "red", "unknown", "blue"]', '{"red": "middle", "blue": "right"}', {'left':0,'right':2,'reject':3}),
    ]
    checks=nbformat.v4.new_notebook(cells=[nbformat.v4.new_code_cell(setup)])
    for name, objects, rules, expected in variants:
        source=factory.replace('OBJECTS = ["red", "blue", "red", "unknown", "blue"]', 'OBJECTS = '+objects).replace('RULES = {"red": "left", "blue": "right"}', 'RULES = '+rules)
        source+='\nassert factory.counts == '+repr(expected)+'\nprint('+repr(name+' PASS')+')'
        checks.cells.extend([nbformat.v4.new_code_cell(source),nbformat.v4.new_code_cell(source)])
    checks.cells.append(nbformat.v4.new_code_cell('counts = compare_observations(["red", "blue"], ["blue", None], {"red":"left", "blue":"right"})\nassert counts == {"left":0, "right":1, "reject":1}'))
    NotebookClient(checks, timeout=90, kernel_name='python3', resources={'metadata': {'path': str(ROOT/'notebooks')}}).execute()
    nbformat.write(checks,args.output/'challenge-variants.ipynb')
    report.append('challenge variants: flip asymmetric, green, None, numeric, list, dictionary, unknown, empty, invalid destination, sensor mismatch; repeated cells reset counts: PASS')
    (args.output/'verification.txt').write_text('\n'.join(report)+'\n')
    print('\n'.join(report))

if __name__=='__main__': main()
