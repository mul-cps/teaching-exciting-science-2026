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
    assert not errors,errors
    page.screenshot(path=args.screenshot)
    print(json.dumps({'browser':'passed','offline':True,'workspace_roundtrip':True,'upstream_logging':True,'drag':True,'stop_listener':True,'js_errors':errors}))
    browser.close()
