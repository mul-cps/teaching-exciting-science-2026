"""Real headless browser rehearsal of canonical beginner missions, offline only."""
import argparse
import json
import tempfile
from pathlib import Path
from playwright.sync_api import sync_playwright
parser=argparse.ArgumentParser();parser.add_argument('--url',default='http://127.0.0.1:8877');parser.add_argument('--screenshot',default='/tmp/blockly-browser.png');args=parser.parse_args()
with sync_playwright() as p:
    browser=p.chromium.launch(headless=True)
    page=browser.new_page(viewport={'width':1440,'height':1100});errors=[]
    page.on('pageerror',lambda error:errors.append(str(error)))
    page.goto(args.url);page.wait_for_function('window.stationStatus && currentMission')
    assert page.evaluate('window.stationStatus.physical') is False
    curriculum=page.evaluate("fetch('curriculum.json').then(r=>r.json())")
    assert len(page.locator('#mission-choice option').all())==len(curriculum['missions'])
    for mission in curriculum['missions']:
        page.select_option('#mission-choice',mission['id'])
        page.wait_for_function('expected=>JSON.stringify(compile())===JSON.stringify(expected)',arg=mission['starter'])
        assert page.locator('#mission-steps li').count()==5
        assert page.locator('#mission-success').inner_text()=='Success: '+mission['success']
        assert page.evaluate('toolbox.contents.map(c=>c.name)')==mission['toolbox']
        assert 'Instructor maintenance' not in page.evaluate('toolbox.contents.map(c=>c.name)')
    # Simple first mission runs without recordings or physical assumptions.
    page.select_option('#mission-choice',curriculum['missions'][0]['id'])
    page.click('#run');page.wait_for_function("window.stationStatus.job.state==='complete'")
    page.click('#python');page.wait_for_function("document.getElementById('code').textContent.includes('run_program')")
    source=page.locator('#code').inner_text();compile(source,'export.py','exec')
    assert 'physical=False' in source
    with page.expect_download() as download:page.click('#download-python')
    with tempfile.TemporaryDirectory() as directory:
        path=Path(directory)/'program.py';download.value.save_as(path);assert path.read_text()==source
    page.locator('#preview button').last.click()
    with page.expect_download() as download:page.click('#save')
    with tempfile.TemporaryDirectory() as directory:
        path=Path(directory)/'blocks.json';download.value.save_as(path)
        saved=json.loads(path.read_text());assert saved['schema']=='so101-blockly-workspace-v1'
        page.evaluate('workspace.clear()');page.locator('#load').set_input_files(path)
        page.wait_for_function('workspace.getAllBlocks(false).length>0')
    # Explicit conveyor state changes and replay+jaw do not synthesize grasp.
    page.evaluate("loadWorkspace({blocks:{languageVersion:0,blocks:[astChain([{op:'ev3',command:'eject'},{op:'ev3',command:'left_end'},{op:'jaw',gap_mm:0},{op:'replay',name:'demo',speed:.5}])]}})")
    previous=page.evaluate('window.stationStatus.job.id');page.click('#run')
    page.wait_for_function("old=>window.stationStatus.job.id!==old&&window.stationStatus.job.state==='complete'",arg=previous)
    state=page.evaluate('window.stationStatus.simulation_state')
    assert state['brick']=='pickup' and state['conveyor_position']=='left'
    assert not state['grasp_verified'] and state['placed_count']==0
    # Screenshot a useful progressive mission with a concrete state observation.
    conveyor=next(m for m in curriculum['missions'] if 'Conveyor' in m['toolbox'])
    page.select_option('#mission-choice',conveyor['id']);page.evaluate('workspace.zoomToFit()')
    page.screenshot(path=args.screenshot,full_page=True)
    page.click('#stop');page.wait_for_function("window.stationStatus.simulation_state.segment==='stopped'")
    assert not errors,errors
    browser.close()
print('PASS: canonical missions, progressive toolbox, exact AST, executable Python, save/open, state and STOP; simulation only')
