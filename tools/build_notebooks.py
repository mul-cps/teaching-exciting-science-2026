"""Generate every textual teaching artifact from curriculum/workshop.json.

--check compares bytes without writing. DOCX/PDF/PPTX are format adapters built
by build_materials.py; this command verifies their authored source projections.
"""
import argparse
import json
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from workshop.curriculum import load_curriculum, learner_projection, STAGES
GENERATED = '<!-- Generated from curriculum/workshop.json; edit the source, then run tools/build_notebooks.py. -->\n\n'



def schedule(c):
    lines=['| Minutes | Duration | Session |','|---|---|---|']; start=0
    for s in c['sessions']:
        end=start+s['minutes']; lines.append(f"| {start}-{end} | {s['minutes']} | {s['title']} |"); start=end
    return '\n'.join(lines)


def rotations(c):
    sections=['## Pair rotations','Each 15-minute slot includes 2 minutes to save and transition. Driver edits; observer predicts and checks. Swap roles halfway. One instructor controls the physical integration station.']
    for circuit in c['rotations']:
        sections.append('### '+next(s['title'] for s in c['sessions'] if s['id']==circuit['id']))
        rows=['| Slot | Pair A | Pair B | Pair C |','|---|---|---|---|']
        for i in range(3): rows.append('| '+str(i+1)+' | '+' | '.join(circuit['stations'][(i+j)%3] for j in range(3))+' |')
        sections.append('\n'.join(rows))
    return '\n\n'.join(sections)


def mission_markdown(m, *, instructor=False):
    lines=[f"## {m['title']}",f"Mission `{m['id']}` · {m['minutes']} minutes · {m['goal']}"]
    names={'worked_example':'Worked example','success':'Success check','extension':'Optional challenge','offline_route':'Offline route'}
    for k in STAGES: lines.append('**'+names.get(k,k.title())+'** '+m[k])
    lines += ['**Hint 1** '+m['hints'][0], '**Hint 2** '+m['hints'][1]]
    if instructor: lines.append('**Instructor guidance** '+m['instructor_notes'])
    return '\n\n'.join(lines)


def notebook(c, instructor=False, live=False):
    cells=[]
    def cell(kind,text,example_id=None,mission_id=None):
        meta={}
        if example_id: meta['example_id']=example_id
        if mission_id: meta['mission_id']=mission_id
        item={'cell_type':kind,'metadata':meta,'source':text.splitlines(True),'id':f"{'instructor' if instructor else 'student'}-{len(cells):03}"}
        if kind=='code': item.update(execution_count=None,outputs=[])
        cells.append(item)
    cell('markdown','# '+('Robot and twin observation' if live else c['title'])+'\n\n'+c['intro']+'\n\n'+c['evidence_rule']+'\n\nSave a pair copy. Run cells from the top with Shift+Enter. The essential route works offline. Read the explicit evidence and capture-mode labels below.\n\n'+schedule(c))
    cell('code',c['examples']['live_bootstrap' if live else 'notebook_setup']['code'],example_id='setup')
    if live:
        spec=c['live_notebook']
        cell('markdown', spec['intro']+'\n\n15-minute observation rotation. Worked example, predict/change/run/observe/explain and success criteria are on the teach demonstration mission card.')
        for item in spec['cells']:
            if 'example_id' in item:
                eid=item['example_id']
                cell('code',c['examples'][eid]['code'],example_id=eid,mission_id='teach-demo')
            else:
                cell('markdown',item['text'],mission_id='teach-demo')
    else:
        cell('markdown','![Actual station photograph](assets/station-photo.jpg)\n\nActual supplied station photograph. It does not prove current commissioning or a successful pickup.')
        seen=set()
        for m in c['missions']:
            cell('markdown',mission_markdown(m,instructor=instructor),mission_id=m['id'])
            eid=m['example_id']
            if eid not in seen:
                cell('code',c['examples'][eid]['code'],example_id=eid,mission_id=m['id']); seen.add(eid)
            if m['id']=='python':
                cell('code',c['examples']['shared']['code'],example_id='shared',mission_id=m['id'])
                cell('markdown','On the prepared shared station, the instructor can submit the same `program` with `run_program(program, physical=False)` from `workshop.blockly_client`. That explicit simulation endpoint must already be running. The executable cell above uses the same validator and interpreter locally. Physical submission requires the supervised prepared station and its admission checks.')
        cell('markdown',rotations(c))
        cell('markdown','## Optional color destinations extension\n\nComplete the fixed-fixture core first. This separate simulated routing example does not teach physical grasping. Predict each route. Change an input, run, observe routes and counts, and explain a limitation. Try three reds and one blue before swapping rules. Unknown or invalid reports reject.')
        cell('code',c['examples']['extension']['code'],example_id='extension')
    return {'cells':cells,'metadata':{'curriculum_schema':1,'kernelspec':{'display_name':'Python 3','language':'python','name':'python3'},'language_info':{'name':'python','version':'3.11'}},'nbformat':4,'nbformat_minor':5}


def outputs(c):
    result={}
    def put(name,text): result[name]=text if isinstance(text,str) else json.dumps(text,indent=2)+'\n'
    put('web/blockly-station/curriculum.json',learner_projection(c))
    plan='# Beginner robot factory lesson plan\n\n'+c['intro']+'\n\n'+c['evidence_rule']+'\n\n'+schedule(c)+'\n\n'+rotations(c)
    put('docs/lesson-plan.md',GENERATED+plan)
    put('docs/instructor.md',GENERATED+'# Beginner robot factory instructor guide\n\n'+c['intro']+'\n\n'+c['evidence_rule']+'\n\nPrepare local Blockly, learner notebooks and printed cards before class. Keep recovery, restart and physical enable controls with the instructor. No essential activity needs an external website. Use the paper route when a station gate is incomplete. Historical commissioning evidence remains in the operational runbooks.\n\n'+schedule(c)+'\n\n'+rotations(c)+'\n\n'+'\n\n'.join(mission_markdown(m,instructor=True) for m in c['missions']))
    worksheet=['# Robot factory worksheet',c['intro'],'Pair names: __________________  Driver first: __________  Observer first: __________',schedule(c)]
    for m in c['missions']:
        worksheet += ['## '+m['title'],m['goal'],'Predict: ______________________________','One change: ___________________________','Run and observed evidence: ______________________________','Explain and limit: ______________________________','Evidence source (circle): PAPER / SIMULATION / PHYSICAL','Success check: '+m['success']]
    put('docs/student-worksheet.md',GENERATED+'\n\n'.join(worksheet)+'\n')
    put('docs/activity-cards.md',GENERATED+'# Robot factory practical mission cards\n\n'+c['intro']+'\n\n'+c['evidence_rule']+'\n\n'+schedule(c)+'\n\n'+rotations(c)+'\n\n'+'\n\n'.join(mission_markdown(m) for m in c['missions']))
    put('docs/blockly-student.md',GENERATED+'# Local Blockly robot factory\n\nStart `python tools/run_blockly_station.py`, then open http://127.0.0.1:8877. Choose a mission, predict, edit one thing, run, inspect and explain. Save your workspace before changing missions. STOP ends the program; later blocks must not run. Reset simulation state between experiments. Missing recordings are a reason to inspect and prepare, never a reason to invent a physical success.\n\n'+c['evidence_rule']+'\n\n'+'\n\n'.join(mission_markdown(m) for m in c['missions'] if m['id'] in ('blocks','observe','integrate','improve')))
    put('docs/workshop-resources.md',GENERATED+'# Offline workshop resources\n\n'+c['intro']+'\n\n'+schedule(c)+'\n\n'+'\n\n'.join('## '+r['title']+'\n\n`'+r['command']+'`' for r in c['resources'])+'\n\nLearner handouts: student-handbook.pdf, student-worksheet.pdf and activity-cards.pdf. Source and scripts remain usable without downloads. External training websites are optional instructor enrichment after the essential local route.\n')
    notes=['# Workshop slide speaker notes','Eight introductory slides total 20 minutes, followed by two slide cards per practical mission. Both cards share the mission duration; they do not add session time.']
    for i,s in enumerate(c['slides'],1): notes += [f"## Slide {i}: {s['title']}",f"{s['minutes']} minutes",s['notes']]
    for i,m in enumerate(c['missions']):
        for part in range(2):
            notes += [f"## Slide {9+i*2+part}: {m['title']} card {part+1}",f"{m['minutes']} minutes total for this activity, shared across its two cards.",mission_markdown(m,instructor=True)]
    put('docs/lecture-notes.md',GENERATED+'\n\n'.join(notes)+'\n')
    for edition in ('Student','Instructor'):
        n=notebook(c,edition=='Instructor'); put(f'notebooks/Robot_Candy_Factory_{edition}.ipynb',n)
        if edition=='Student':
            body=[]
            for cell in n['cells']:
                source=''.join(cell['source'])
                if cell['cell_type']=='markdown': body.append(source.replace('(assets/station-photo.jpg)','(../notebooks/assets/station-photo.jpg)'))
                else: body.append('```python\n'+source+'\n```')
            put('docs/student-exercises.md',GENERATED+'\n\n'.join(body)+'\n')
    put('notebooks/Robot_Candy_Factory_Live_SO101.ipynb',notebook(c,live=True))
    # Each script is generated from exactly the code cells used in the notebook.
    for eid,e in c['examples'].items():
        if not e.get('script'): continue
        code=e['code']
        if eid=='sequence': code=c['examples']['shared']['code']
        header='"""Generated from curriculum/workshop.json. '+('Optional color-routing extension.' if eid=='extension' else 'Offline beginner activity.')+'\nRun with the prepared Python; no physical commands are sent.\n"""\n'
        pre='import sys\nfrom pathlib import Path\nsys.path.insert(0, str(Path(__file__).resolve().parents[1]))\n\n'
        put('exercises/'+e['script'],header+(pre if eid in ('shared','extension') else '')+code)
    put('notebooks/README.md', GENERATED+'# Offline classroom notebooks\n\n'+c['intro']+'\n\nOn the prepared PC run `./station notebook`, open `Robot_Candy_Factory_Student.ipynb` and save a pair copy. Run cells from the top with Shift+Enter. Restart and rerun from the top after a confusing edit. Every executable cell uses invented inputs or explicit simulation; no learner cell imports ROS or commands hardware.\n\nIn the extracted learner ZIP, keep all folders together. Use the prepared Python to run `python notebooks/launch.py`. No installation or internet is a classroom task. Without Jupyter, run `python exercises/check_setup.py`, then scripts 01, 02 and 03. Script 04 is the optional color-routing extension, and 05 renders that extension as an offline HTML report.\n\nStart local Blockly with `python tools/run_blockly_station.py`; open http://127.0.0.1:8877 and choose the first mission. `./station sim` is the prepared-PC shortcut. Both languages share validated command JSON and the same interpreter.\n\n`Robot_Candy_Factory_Live_SO101.ipynb` defaults to invented DEMO observations. The prepared instructor launcher `tools/launch_live_workshop.py --live --ros-wrapper /path/to/wrapper` sets WORKSHOP_LIVE=1 for read-only live topic capture in a bounded wrapper subprocess. Missing LIVE data stay explicitly unavailable; there is no DEMO fallback. No cell sends motion or changes modes.\n\n'+c['evidence_rule']+'\n')
    # An editable standalone sequence diagram, derived from the slide text.
    steps=c['slides'][3]['lines']; boxes=[]
    for i,t in enumerate(steps):
        import textwrap
        x=20+i*230
        lines=textwrap.wrap(t, width=22)
        spans=''.join(f'<tspan x="{x+105}" dy="{0 if j==0 else 22}">{line}</tspan>' for j,line in enumerate(lines))
        boxes.append(f'<rect x="{x}" y="25" width="210" height="90" rx="8" fill="#e4eff0" stroke="#17354b"/><text x="{x+105}" y="{70-(len(lines)-1)*11}" text-anchor="middle" font-family="sans-serif" font-size="16">{spans}</text>')
        if i<4: boxes.append(f'<path d="M {x+210} 70 h 20" stroke="#197f80" stroke-width="3"/>')
    put('docs/assets/fixed-fixture-sequence.svg','<svg xmlns="http://www.w3.org/2000/svg" width="1170" height="145" viewBox="0 0 1170 145">'+''.join(boxes)+'</svg>\n')
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('--check',action='store_true'); args=parser.parse_args(argv)
    differences=[]
    for name,text in outputs(load_curriculum()).items():
        p=ROOT/name
        if args.check:
            if not p.exists() or p.read_text()!=text: differences.append(name)
        else:
            p.parent.mkdir(parents=True,exist_ok=True); p.write_text(text)
    if differences: raise SystemExit('Generated curriculum drift: '+', '.join(differences))
    print('Canonical curriculum '+('verified' if args.check else 'generated')+' (240 minutes, eight intro slides).')

if __name__=='__main__': main()
