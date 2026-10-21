"""Headless offline browser rehearsal. Start run_blockly_station.py first."""
import argparse
import json
import tempfile
from pathlib import Path
from playwright.sync_api import sync_playwright
parser=argparse.ArgumentParser(); parser.add_argument('--url',default='http://127.0.0.1:8877');parser.add_argument('--screenshot',default='/tmp/blockly-browser.png');args=parser.parse_args()
with sync_playwright() as p:
    browser=p.chromium.launch(headless=True)
    page=browser.new_page(viewport={'width':1440,'height':1000}); errors=[]
    page.on('pageerror',lambda error:errors.append(str(error)))
    page.goto(args.url); page.wait_for_function("window.stationStatus && !window.stationStatus.physical")
    assert 'SIMULATION' in page.locator('#mode').inner_text()
    page.click('#run');page.wait_for_function("window.stationStatus.job.state === 'complete'",timeout=10000)
    assert 'ev3:eject' in page.locator('#trace').inner_text()
    with page.expect_download() as download: page.click('#save')
    with tempfile.TemporaryDirectory() as directory:
        path=Path(directory)/'blocks.json';download.value.save_as(path)
        saved=json.loads(path.read_text());assert saved['schema']=='so101-blockly-workspace-v1'
        page.evaluate('workspace.clear()');page.locator('#load').set_input_files(path)
        page.wait_for_function('workspace.getAllBlocks(false).length === 4')
    # All three offline starters are downloadable, importable and compile to their stated tasks.
    log_text='Record my_short_move first with the jaw fixed. In simulation only, change the replay name to demo.'
    unknown_text='Unknown object: ask the instructor. No dispensing or movement.'
    def ev3(command): return {'op':'ev3','command':command}
    factory=[ev3('connect'),{'op':'repeat','count':2,'body':[{'op':'if_color','color':'red',
        'then':[ev3('eject'),ev3('left_end')],
        'else':[{'op':'if_color','color':'blue','then':[ev3('eject'),ev3('right_end')],
                 'else':[{'op':'log','message':unknown_text},{'op':'stop'}]}]}]}]
    starters={
        'ev3':[ev3(command) for command in ('connect','home','left_end','right_end')],
        'teach':[{'op':'log','message':log_text},{'op':'replay','name':'my_short_move','speed':.5,'timing':'original'}],
        'factory':factory,
    }
    for choice,expected in starters.items():
        page.select_option('#starter-choice',choice);page.click('#load-starter')
        page.wait_for_function('expected => {try{return JSON.stringify(compile()) === JSON.stringify(expected);}catch{return false;}}',arg=expected)
        with page.expect_download() as download:page.click('#download-starter')
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'starter.json';download.value.save_as(path)
            data=json.loads(path.read_text());assert data['schema']=='so101-blockly-workspace-v1'
            page.evaluate('workspace.clear()');page.locator('#load').set_input_files(path)
            page.wait_for_function('expected => {try{return JSON.stringify(compile()) === JSON.stringify(expected);}catch{return false;}}',arg=expected)
        page.evaluate('workspace.zoomToFit()')
        shot=Path(args.screenshot)
        page.screenshot(path=str(shot.with_name(shot.stem+'-starter-'+choice+shot.suffix)))
    def run_to(state='complete'):
        previous=page.evaluate('window.stationStatus.job.id')
        page.click('#run')
        page.wait_for_function('(old) => window.stationStatus.job.id !== old && window.stationStatus.job.state !== "running"',arg=previous)
        assert page.evaluate('window.stationStatus.job.state')==state
    page.select_option('#starter-choice','ev3');page.click('#load-starter')
    page.wait_for_function('expected => {try{return JSON.stringify(compile()) === JSON.stringify(expected);}catch{return false;}}',arg=starters['ev3']);run_to()
    assert 'ev3:right_end' in page.locator('#trace').inner_text()
    page.select_option('#starter-choice','teach');page.click('#load-starter')
    page.wait_for_function('expected => {try{return JSON.stringify(compile()) === JSON.stringify(expected);}catch{return false;}}',arg=starters['teach'])
    page.evaluate("workspace.getAllBlocks(false).find(block=>block.type==='station_replay').setFieldValue('demo','NAME')")
    run_to()  # demo is selected deliberately only in this simulator test.
    for color in ('red','blue','unknown'):
        page.select_option('#starter-choice','factory');page.click('#load-starter')
        page.wait_for_function('expected => {try{return JSON.stringify(compile()) === JSON.stringify(expected);}catch{return false;}}',arg=factory)
        with page.expect_response('**/api/simulation/color'):page.select_option('#sim-color',color)
        run_to('failed' if color=='unknown' else 'complete')
        if color=='unknown':
            events=page.evaluate('window.stationStatus.events')
            tail=events[events.index(unknown_text):]
            assert tail[0]==unknown_text and 'STOP' in tail and not any(event.startswith('ev3:') for event in tail)
    with page.expect_response('**/api/simulation/color'):page.select_option('#sim-color','red')
    # Compatible upstream block IDs, input names and Python generator semantics.
    page.evaluate("""workspace.clear();const log=workspace.newBlock('node_logging_info');const text=workspace.newBlock('node_logging_text');text.setFieldValue('Hello factory','TEXT');log.initSvg();log.render();text.initSvg();text.render();log.getInput('MESSAGE').connection.connect(text.outputConnection);""")
    page.click('#python');assert 'self.get_logger().info(str("Hello factory"))' in page.locator('#code').inner_text()
    page.locator('#preview button').click();page.click('#run')
    page.wait_for_function("window.stationStatus.job.state === 'complete' && (window.stationStatus.events || []).includes('Hello factory')")
    # Open toolbox then physically drag a new wait block into the workspace.
    page.locator('.blocklyTreeRow').filter(has_text='Station sequence').click()
    wait=page.locator('.blocklyFlyout .blocklyDraggable').filter(has_text='wait').first
    box=wait.bounding_box();assert box
    page.mouse.move(box['x']+30,box['y']+15);page.mouse.down();page.mouse.move(650,420,steps=12);page.mouse.up()
    assert page.evaluate("workspace.getAllBlocks(false).some(b=>b.type==='station_wait')")
    stop_count=page.evaluate("(window.stationStatus.events || []).filter(e=>e==='STOP').length")
    page.click('#stop');page.wait_for_function("count => (window.stationStatus.events || []).filter(e=>e==='STOP').length > count",arg=stop_count)
    page.evaluate("workspace.clear();const block=workspace.newBlock('station_sort_cycle');block.initSvg();block.render();")
    assert page.evaluate('compile()')==[{'op':'sort_cycle'}]
    page.click('#python')
    assert 'station.command({"op":"sort_cycle"}, cancel, deadline)' in page.locator('#code').inner_text()
    page.locator('#preview button').click();page.click('#run')
    page.wait_for_function("window.stationStatus.job.state==='complete' && window.stationStatus.job.result.sort_cycles?.[0]?.completed===1")
    assert 'sort_cycle: red' in page.locator('#trace').inner_text()
    # Preview follows the compiled tree, including loops and non-taken branches.
    page.evaluate("""workspace.clear();const repeat=workspace.newBlock('station_repeat');repeat.setFieldValue(2,'COUNT');const condition=workspace.newBlock('station_color');condition.setFieldValue('blue','COLOR');const cycle=workspace.newBlock('station_sort_cycle');repeat.getInput('BODY').connection.connect(condition.previousConnection);condition.getInput('THEN').connection.connect(cycle.previousConnection);for(const block of workspace.getAllBlocks(false)){block.initSvg();block.render();}""")
    assert page.evaluate('compile()')==[{'op':'repeat','count':2,'body':[{'op':'if_color','color':'blue','then':[{'op':'sort_cycle'}],'else':[]}]}]
    page.click('#python')
    preview=page.locator('#code').inner_text()
    assert 'for _ in range(2):\n    if station.color() == "blue":\n        result = station.command({"op":"sort_cycle"}, cancel, deadline)\n    else:\n        pass' in preview
    assert 'not standalone execution code' in preview
    page.locator('#preview button').click()
    page.click('#run');page.wait_for_function("window.stationStatus.job.state==='complete' && !window.stationStatus.job.result.sort_cycles")
    assert not errors,errors
    # Screenshots capture only the real simulation, before injected readiness fixtures.
    page.screenshot(path=args.screenshot)
    fixture={'physical':True,'instructor':True,'operator_busy':False,'recovery_required':False,
             'job':{'state':'idle'},'recordings':['demo'],'streams':{
                 '/station/operator/status':{'age_s':.1,'data':{'schema':1,'fresh':True,'capabilities':{
                     'moveit':{'enabled':True,'reason':'fresh admitted owner'},
                     'restart_camera':{'enabled':False,'reason':'camera restart unit is not configured'}}}}}}
    page.click('#example')
    fixture['recording_info']={'demo':{'original_duration_s':20.,'skip_stationary_pauses_duration_s':3.}}
    page.route('**/api/status',lambda route:route.fulfill(json=fixture))
    def poll(marker):
        fixture['probe_marker']=marker
        page.wait_for_function('marker => window.stationStatus.probe_marker === marker',arg=marker)
    poll('fresh')
    assert page.evaluate("compile()[0].body[2].timing")=='original'
    assert page.evaluate("workspace.getAllBlocks(false).find(b=>b.type==='station_replay').getFieldValue('DURATION')")=="original 20.0 s → requested 40.0 s; controller may retime"
    page.evaluate("workspace.clearUndo();workspace.getAllBlocks(false).find(b=>b.type==='station_replay').setFieldValue('skip_stationary_pauses','TIMING')")
    page.wait_for_function("workspace.getAllBlocks(false).find(b=>b.type==='station_replay').getFieldValue('DURATION').includes('requested 6.0 s')")
    assert page.evaluate("compile()[0].body[2].timing")=='skip_stationary_pauses'
    page.evaluate('workspace.undo(false)')
    page.wait_for_function("compile()[0].body[2].timing==='original' && workspace.getAllBlocks(false).find(b=>b.type==='station_replay').getFieldValue('DURATION').includes('requested 40.0 s')",timeout=3000)
    page.evaluate('workspace.undo(true)')
    page.wait_for_function("compile()[0].body[2].timing==='skip_stationary_pauses' && workspace.getAllBlocks(false).find(b=>b.type==='station_replay').getFieldValue('DURATION').includes('requested 6.0 s')",timeout=3000)
    moveit=page.locator('#instructor [data-command="moveit"]')
    restart=page.locator('#instructor [data-command="restart_camera"]')
    recovery=page.locator('#instructor [data-recovery]')
    assert moveit.is_enabled()
    assert restart.is_disabled() and restart.get_attribute('title')=='camera restart unit is not configured'
    assert page.locator('#instructor [data-command="teleop"]').is_disabled() # missing capability
    assert recovery.is_disabled()
    fixture['streams']['/station/operator/status']['data']['fresh']=False;poll('stale-owner')
    assert moveit.is_disabled()
    fixture['streams']['/station/operator/status']['data']['fresh']=True
    fixture['streams']['/station/operator/status']['age_s']=2.;poll('stale')
    assert moveit.is_disabled()
    fixture['streams']['/station/operator/status']['age_s']=-.1;poll('negative-age')
    assert moveit.is_disabled()
    fixture['streams']['/station/operator/status']['age_s']=None;poll('invalid-age')
    assert moveit.is_disabled()
    fixture['streams']['/station/operator/status']['age_s']=.1
    fixture['recovery_required']=True;poll('recover')
    assert recovery.is_enabled()
    fixture['operator_busy']=True;poll('busy')
    assert moveit.is_disabled() and recovery.is_disabled()
    fixture['operator_busy']=False;fixture['job']['state']='running';poll('running')
    assert moveit.is_disabled() and recovery.is_disabled()
    fixture['job']['state']='idle';fixture['streams']={};poll('missing-status')
    assert moveit.is_disabled()
    fixture['instructor']=False;poll('student')
    assert moveit.is_disabled() and recovery.is_disabled()
    assert page.locator('#stop').is_enabled()
    page.unroute('**/api/status')
    page.wait_for_function('window.stationStatus.physical === false')
    assert not errors,errors
    # Physical status is a browser fixture only: the server remains isolated SIM, no ROS/backend calls.
    physical_page=browser.new_page();physical_requests=[]
    physical_page.on('request',lambda request:physical_requests.append((request.method,request.url)))
    physical_fixture={'physical':True,'instructor':False,'operator_busy':False,'recovery_required':False,
                      'job':{'state':'idle'},'recordings':[],'recording_info':{},'streams':{},'events':[]}
    physical_page.route('**/api/status',lambda route:route.fulfill(json=physical_fixture))
    physical_page.goto(args.url);physical_page.wait_for_function('window.stationStatus?.physical === true')
    assert [row['op'] for row in physical_page.evaluate('compile()')]==['log']
    physical_page.click('#example')
    assert [row['op'] for row in physical_page.evaluate('compile()')]==['log']
    physical_page.evaluate("workspace.clear();const block=workspace.newBlock('station_wait');block.setFieldValue(3,'SECONDS');block.initSvg();block.render();")
    physical_page.wait_for_timeout(900)
    assert physical_page.evaluate('compile()')==[{'op':'wait','seconds':3}]
    assert not any(method=='POST' for method,_ in physical_requests)
    physical_page.close()
    unknown_page=browser.new_page();allow_status=[False]
    simulation_fixture={**physical_fixture,'physical':False,'recordings':['demo']}
    def pending_status(route):
        if allow_status[0]:route.fulfill(json=simulation_fixture)
        else:route.abort()
    unknown_page.route('**/api/status',pending_status)
    unknown_page.goto(args.url);unknown_page.wait_for_function("document.getElementById('mode').textContent==='DISCONNECTED'")
    assert [row['op'] for row in unknown_page.evaluate('compile()')]==['log']
    unknown_page.click('#example')
    assert [row['op'] for row in unknown_page.evaluate('compile()')]==['log']
    unknown_page.evaluate("workspace.clear();const block=workspace.newBlock('station_wait');block.setFieldValue(7,'SECONDS');block.initSvg();block.render();")
    allow_status[0]=True
    unknown_page.wait_for_function('window.stationStatus?.physical === false')
    assert unknown_page.evaluate('compile()')==[{'op':'wait','seconds':7}]
    unknown_page.click('#example')
    assert unknown_page.evaluate('compile()[0].body[2].name')=='demo'
    allow_status[0]=False
    unknown_page.wait_for_function("document.getElementById('mode').textContent==='DISCONNECTED'")
    unknown_page.click('#example')
    assert [row['op'] for row in unknown_page.evaluate('compile()')]==['log']
    unknown_page.close()
    print(json.dumps({'browser':'passed','offline':True,'workspace_roundtrip':True,'three_starters':True,'physical_unknown_startup_log_only':True,'edits_preserved':True,'upstream_logging':True,'drag':True,'stop_listener':True,'operator_capabilities':True,'js_errors':errors}))
    browser.close()
