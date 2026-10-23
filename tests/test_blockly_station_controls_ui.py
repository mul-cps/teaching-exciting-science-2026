"""Execute the real JavaScript block compiler without a browser or hardware."""
import json
from pathlib import Path
import shutil
import subprocess

import pytest

from workshop.blockly_program import validate_program

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def compiled_blocks():
    node = shutil.which('node')
    if node is None:
        pytest.skip('Node is required to exercise the JavaScript compiler')
    script = r'''
const fs=require('fs'),vm=require('vm');
const source=fs.readFileSync('web/blockly-station/app.js','utf8');
const context={registerRosBlocky(){},Blockly:{defineBlocksWithJsonArray(){}},python:{pythonGenerator:{forBlock:{}}}};
vm.createContext(context);
vm.runInContext(source.slice(0,source.indexOf('const workspace='))+
 source.slice(source.indexOf('function compileChain('),source.indexOf('let recordingInfo=')),context);
function block(data){return {type:data.type,getFieldValue:n=>data.fields?.[n],
 getNextBlock:()=>data.next?.block?block(data.next.block):null,
 getInputTargetBlock:n=>{const input=data.inputs?.[n];const child=input?.block||input?.shadow;return child?block(child):null;}};}
context.block=block;
const result=vm.runInContext(`(()=>{
 const rows=[];
 for(const definition of definitions){
  const fields=Object.fromEntries((definition.args0||[]).filter(a=>a.name&&a.type.startsWith('field_')).map(a=>[a.name,a.text??a.value??a.options?.[0]?.[1]]));
  const dropdown=(definition.args0||[]).find(a=>a.type==='field_dropdown'&&a.name==='COMMAND');
  for(const option of dropdown?.options||[null]){
   const state={type:definition.type,fields:{...fields}};
   if(option)state.fields.COMMAND=option[1];
   const program=compileChain(block(state));rows.push({type:definition.type,command:option?.[1],program});
  }
 }
 const log={type:'node_logging_info',inputs:{MESSAGE:{shadow:{type:'node_logging_text',fields:{TEXT:'Hello'}}}}};
 rows.push({type:'node_logging_info',program:compileChain(block(log))});
 return {rows,categories:toolbox.contents.map(c=>c.name),stationOptions:stationCommandOptions};
})()`,context);
result.starters=[];
for(const file of fs.readdirSync('web/blockly-station/examples').filter(f=>f.endsWith('.json'))){
 const state=JSON.parse(fs.readFileSync('web/blockly-station/examples/'+file));
 result.starters.push({file,program:context.compileChain(block(state.workspace.blocks.blocks[0]))});
}
process.stdout.write(JSON.stringify(result));
'''
    result = subprocess.run([node, '-e', script], cwd=ROOT, check=True,
                            text=True, capture_output=True)
    return json.loads(result.stdout)


def test_every_dropdown_compiles_to_a_valid_program(compiled_blocks):
    rows = compiled_blocks['rows']
    assert {row['type'] for row in rows} == {
        'station_control', 'station_arm', 'station_record_stroke', 'station_replay', 'station_approach',
        'station_ev3', 'station_ev3_service', 'station_wait', 'station_jaw', 'station_repeat',
        'station_color', 'station_stop', 'station_sort_cycle', 'station_taught_pick_place', 'node_logging_info'}
    for row in rows:
        validate_program(row['program'])
        if row['type'] == 'station_control':
            assert row['program'] == [{'op': 'station', 'command': row['command']}]


def test_all_station_operator_controls_and_composites_available(compiled_blocks):
    expected = {'emergency_reset', 'reset', 'start', 'base_release', 'base_confirm',
                'teleop', 'teleop_relative', 'teleop_absolute', 'moveit', 'damping',
                'zero_torque', 'restart_moveit', 'restart_camera', 'restart_depth',
                'restart_tags', 'restart_all', 'verify_restart', 'reinitialize',
                'verify_recovery'}
    options = compiled_blocks['stationOptions']
    assert {command for _, command in options} == expected
    assert len(options) == len(expected)
    assert ['Restart entire station + verify', 'reinitialize'] in options
    assert ['Verify recovery / unlock', 'verify_recovery'] in options
    assert compiled_blocks['categories'] == ['Messages']


def test_all_five_starters_compile_and_reset_is_a_separate_run(compiled_blocks):
    starters = {row['file']: row['program'] for row in compiled_blocks['starters']}
    assert len(starters) == 5
    for program in starters.values():
        validate_program(program)
    reset = starters['05_reset_station.json']
    assert reset[0] == {'op': 'station', 'command': 'reinitialize'}
    assert len(reset) == 2 and reset[1]['op'] == 'log'
    assert 'separate sequence' in reset[1]['message']
    html = (ROOT/'web/blockly-station/index.html').read_text()
    js = (ROOT/'web/blockly-station/app.js').read_text()
    assert '<option value="reset">5 · Reset and reinitialize</option>' in html
    assert "reset:'05_reset_station.json'" in js
    assert "$('run').disabled" not in js  # Rescue blocks must remain runnable while latched.
    assert "clicked('stop',async()=>show(await api('program/stop',{})))" in js
