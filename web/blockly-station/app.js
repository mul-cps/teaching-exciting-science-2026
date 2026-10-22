/* Offline Blockly station extension. Only strict JSON opcodes are executable. */
registerRosBlocky(Blockly,python.pythonGenerator);
const stationCommandOptions=[
 ['Restart entire station + verify','reinitialize'],['Verify recovery / unlock','verify_recovery'],
 ['Restart entire station','emergency_reset'],['Reset (torque OFF)','reset'],
 ['Saved home position','start'],['Release for manual setup','base_release'],['Record manual reference','base_confirm'],
 ['Enable leader teleop','teleop'],['Select relative teleop mapping','teleop_relative'],['Select absolute teleop mapping','teleop_absolute'],
 ['MoveIt mode','moveit'],['Gentle damping','damping'],['ZERO TORQUE / STOP','zero_torque'],
 ['Restart MoveIt','restart_moveit'],['Restart camera','restart_camera'],['Restart depth','restart_depth'],
 ['Restart tags','restart_tags'],['Restart perception','restart_all'],['Verify restart / recover','verify_restart']
];
const definitions=[
 {type:'station_control',message0:'Station %1',args0:[{type:'field_dropdown',name:'COMMAND',options:stationCommandOptions}],previousStatement:null,nextStatement:null,colour:0,tooltip:'Instructor station controls. Restart entire station + verify resets services then checks recovery. Run recovery controls in their own flat sequence; switch arm mode or move in a separate run after verification succeeds. Reset leaves torque off; it does not move to home.'},
 {type:'station_sort_cycle',message0:'Run one commissioned sorting cycle',previousStatement:null,nextStatement:null,colour:180,tooltip:'Requires the commissioned station sorting configuration, calibrated robot, live scene and destinations. Uses the native verified pickup/lift/release transaction; not a taught-stroke shortcut.'},
 {type:'station_ev3',message0:'EV3 %1',args0:[{type:'field_dropdown',name:'COMMAND',options:[['dispense one brick','eject'],['conveyor home','home'],['conveyor left','left_end'],['conveyor right','right_end'],['wake','wake'],['sleep','sleep'],['connect','connect'],['Reset inspected EV3 fault / STOP latch','reset_fault']]}],previousStatement:null,nextStatement:null,colour:180,tooltip:'After instructor inspection, reset_fault releases EV3 STOP inhibition. The feeder reference must be established separately.'},
 {type:'station_wait',message0:'wait %1 seconds',args0:[{type:'field_number',name:'SECONDS',value:1,min:0,max:10,precision:.1}],previousStatement:null,nextStatement:null,colour:210},
 {type:'station_arm',message0:'SO-ARM101 %1',args0:[{type:'field_dropdown',name:'COMMAND',options:[['saved home','home'],['MoveIt mode','moveit'],['leader teleop relative','teleop_relative'],['leader teleop absolute','teleop_absolute'],['torque OFF / station STOP','torque_off']]}],previousStatement:null,nextStatement:null,colour:30,tooltip:'Select a SO-ARM101 command. Torque OFF / station STOP also stops EV3 and disarms the arm. Saved home moves the arm; teleop lets the leader guide it. Recording only captures a manual demonstration; replay plays a saved stroke. No automatic approach to a stroke start.'},
 {type:'station_record_stroke',message0:'record follower stroke %1 for %2 seconds',args0:[{type:'field_input',name:'NAME',text:'demo'},{type:'field_number',name:'SECONDS',value:5,min:1,max:30,precision:.1}],previousStatement:null,nextStatement:null,colour:45,tooltip:'Capture follower positions during a manual leader demonstration. Hold the jaw fixed. Read-only capture applies no force and does not move the arm or approach a start pose. Replay plays the saved recording separately.'},
 {type:'station_replay',message0:'replay stroke %1 speed %2 %3',args0:[{type:'field_input',name:'NAME',text:'demo'},{type:'field_number',name:'SPEED',value:.5,min:.1,max:1,precision:.1},{type:'field_dropdown',name:'TIMING',options:[['original timing','original'],['skip stationary pauses','skip_stationary_pauses']]}],message1:'%1',args1:[{type:'field_label_serializable',name:'DURATION',text:'duration unavailable; controller may retime'}],previousStatement:null,nextStatement:null,colour:30,tooltip:'Play a previously recorded stroke. This moves the follower; recording captures a manual demonstration. No autonomous approach to the stroke start.'},
 {type:'station_jaw',message0:'jaw gap %1 mm',args0:[{type:'field_number',name:'GAP',value:30,min:0,max:200,precision:1}],previousStatement:null,nextStatement:null,colour:30},
 {type:'station_repeat',message0:'repeat %1 times %2 %3',args0:[{type:'field_number',name:'COUNT',value:2,min:1,max:5,precision:1},{type:'input_dummy'},{type:'input_statement',name:'BODY'}],previousStatement:null,nextStatement:null,colour:120},
 {type:'station_color',message0:'if sensor sees %1 %2 do %3 otherwise %4',args0:[{type:'field_dropdown',name:'COLOR',options:['red','blue','green','yellow','unknown'].map(c=>[c,c])},{type:'input_dummy'},{type:'input_statement',name:'THEN'},{type:'input_statement',name:'ELSE'}],previousStatement:null,nextStatement:null,colour:280},
 {type:'station_stop',message0:'STOP station and end program',previousStatement:null,nextStatement:null,colour:0}
];
Blockly.defineBlocksWithJsonArray(definitions);
python.pythonGenerator.forBlock.station_sort_cycle=()=> '# Commissioned station only; native RunCycles.cycles is fixed to 1.\nresult = station.command({"op": "sort_cycle"}, cancel, deadline)\n';
const blocks=types=>types.map(type=>({kind:'block',type}));
const toolbox={kind:'categoryToolbox',contents:[
 {kind:'category',name:'Station controls',colour:'0',contents:blocks(['station_control'])},
 {kind:'category',name:'SO-ARM101',colour:'30',contents:blocks(['station_arm','station_jaw'])},
 {kind:'category',name:'Recordings',colour:'45',contents:blocks(['station_record_stroke','station_replay'])},
 {kind:'category',name:'EV3 conveyor',colour:'180',contents:blocks(['station_ev3'])},
 {kind:'category',name:'Flow & decisions',colour:'210',contents:blocks(['station_wait','station_repeat','station_color','station_stop','station_sort_cycle'])},
 {kind:'category',name:'ROS Blocky · logging',colour:'#7f8c8d',contents:[{kind:'block',type:'node_logging_info',inputs:{MESSAGE:{shadow:{type:'node_logging_text'}}}}]}
]};
const workspace=Blockly.inject('workspace',{toolbox,media:'vendor/media/',trashcan:true,scrollbars:true,zoom:{controls:true,wheel:true},sounds:false});
function compileChain(block,depth=0){
 if(depth>4)throw Error('Maximum nesting is four'); const rows=[];
 while(block){
  const f=n=>block.getFieldValue(n); let row;
  switch(block.type){
   case'station_sort_cycle':row={op:'sort_cycle'};break;
   case'station_control':row={op:'station',command:f('COMMAND')};break;
   case'station_arm':row={op:'arm',command:f('COMMAND')};break;
   case'station_record_stroke':row={op:'record_stroke',name:f('NAME'),seconds:Number(f('SECONDS'))};break;
   case'station_ev3':row={op:'ev3',command:f('COMMAND')};break;
   case'station_wait':row={op:'wait',seconds:Number(f('SECONDS'))};break;
   case'station_replay':row={op:'replay',name:f('NAME'),speed:Number(f('SPEED')),timing:f('TIMING')||'original'};break;
   case'station_jaw':row={op:'jaw',gap_mm:Number(f('GAP'))};break;
   case'station_repeat':row={op:'repeat',count:Number(f('COUNT')),body:compileChain(block.getInputTargetBlock('BODY'),depth+1)};break;
   case'station_color':row={op:'if_color',color:f('COLOR'),then:compileChain(block.getInputTargetBlock('THEN'),depth+1),else:compileChain(block.getInputTargetBlock('ELSE'),depth+1)};break;
   case'station_stop':row={op:'stop'};break;
   case'node_logging_info':{
    const input=block.getInputTargetBlock('MESSAGE');if(!input||input.type!=='node_logging_text')throw Error('Connect a ROS Blocky text block');
    row={op:'log',message:input.getFieldValue('TEXT')};break;}
   default:throw Error('Unsupported block '+block.type);
  }
  rows.push(row);if(rows.length>100)throw Error('Too many blocks');block=block.getNextBlock();
 }
 return rows;
}
let recordingInfo={};
function updateReplayDurations(){
 for(const block of workspace.getAllBlocks(false)){
  if(block.type!=='station_replay')continue;
  const info=recordingInfo[block.getFieldValue('NAME')];const speed=Number(block.getFieldValue('SPEED'));
  const duration=block.getFieldValue('TIMING')==='skip_stationary_pauses'?info?.skip_stationary_pauses_duration_s:info?.original_duration_s;
  const text=Number.isFinite(duration)&&Number.isFinite(info?.original_duration_s)&&speed>0?
   `original ${info.original_duration_s.toFixed(1)} s → requested ${(duration/speed).toFixed(1)} s; controller may retime`:
   'duration unavailable; controller may retime';
  if(block.getFieldValue('DURATION')!==text){
   Blockly.Events.disable();
   try{block.setFieldValue(text,'DURATION');}finally{Blockly.Events.enable();}
  }
 }
}
workspace.addChangeListener(updateReplayDurations);
function compile(){const tops=workspace.getTopBlocks(true).filter(b=>!b.outputConnection);if(tops.length!==1)throw Error('Connect blocks into one sequence');return compileChain(tops[0]);}
const $=id=>document.getElementById(id);
async function api(path,body){const base=path==='program/stop'?'http://127.0.0.1:'+(Number(location.port)+1):'';const response=await fetch(base+'/api/'+path,{method:body===undefined?'GET':'POST',headers:{'X-Station-Token':window.STATION_TOKEN,'Content-Type':'application/json'},body:body===undefined?undefined:JSON.stringify(body)});const data=await response.json();if(!response.ok)throw Error(data.error||response.statusText);return data;}
function show(data){$('result').textContent=typeof data==='string'?data:JSON.stringify(data,null,2);}
function clicked(id,fn){$(id).onclick=async()=>{try{await fn();}catch(error){show(error.message);}};}
let knownMode=null;let startupModeResolved=false;let startupWorkspaceSnapshot=null;
const starterFiles={ev3:'01_ev3.json',teach:'02_teach_replay.json',factory:'03_color_factory.json',armteach:'04_arm_teach.json',reset:'05_reset_station.json'};
function loadWorkspace(state){
 Blockly.Events.disable();
 try{workspace.clear();Blockly.serialization.workspaces.load(state,workspace);}
 finally{Blockly.Events.enable();}
 updateReplayDurations();
}
function example(){
 if(knownMode===false){
  loadWorkspace({blocks:{languageVersion:0,blocks:[{type:'station_repeat',x:35,y:35,fields:{COUNT:2},inputs:{BODY:{block:{type:'station_ev3',fields:{COMMAND:'eject'},next:{block:{type:'station_wait',fields:{SECONDS:1},next:{block:{type:'station_replay',fields:{NAME:'demo',SPEED:.5}}}}}}}}}]}});
 }else{
  loadWorkspace({blocks:{languageVersion:0,blocks:[{type:'node_logging_info',x:35,y:35,inputs:{MESSAGE:{shadow:{type:'node_logging_text',fields:{TEXT:'Choose a starter with the instructor. Record and inspect paths before physical execution.'}}}}}]}});
 }
}
function updateStarterDownload(){
 const file=starterFiles[$('starter-choice').value];
 if(!file)throw Error('Choose a supplied starter');
 $('download-starter').href='examples/'+file;
}
$('starter-choice').onchange=updateStarterDownload;
clicked('load-starter',async()=>{
 updateStarterDownload();
 const response=await fetch($('download-starter').getAttribute('href'));
 if(!response.ok)throw Error('Starter is unavailable');
 const data=await response.json();
 if(data.schema!=='so101-blockly-workspace-v1')throw Error('Unsupported starter format');
 loadWorkspace(data.workspace);
 show('Starter loaded. Predict its actions, check placeholders and ask the instructor before physical execution.');
});
clicked('example',example);clicked('run',async()=>show(await api('program/run',{program:compile()})));clicked('stop',async()=>show(await api('program/stop',{})));
clicked('record-start',async()=>show(await api('record/start',{name:$('record-name').value})));
clicked('record-stop',async()=>show(await api('record/stop',{})));
clicked('save',()=>{const data={schema:'so101-blockly-workspace-v1',workspace:Blockly.serialization.workspaces.save(workspace)};const link=document.createElement('a');link.href=URL.createObjectURL(new Blob([JSON.stringify(data,null,2)],{type:'application/json'}));link.download='factory-blocks.json';link.click();setTimeout(()=>URL.revokeObjectURL(link.href),1000);});
$('load').onchange=async event=>{try{const file=event.target.files[0];if(!file||file.size>262144)throw Error('Choose a workspace smaller than 256 KB');const data=JSON.parse(await file.text());if(data.schema!=='so101-blockly-workspace-v1')throw Error('Unsupported workspace format');workspace.clear();Blockly.serialization.workspaces.load(data.workspace,workspace);}catch(error){show(error.message);}};
function previewProgram(rows,depth=0){
 const indent='    '.repeat(depth);let code='';
 if(!rows.length)return indent+'pass\n';
 for(const row of rows){
  if(row.op==='repeat')code+=indent+`for _ in range(${row.count}):\n`+previewProgram(row.body,depth+1);
  else if(row.op==='if_color')code+=indent+`if station.color() == ${JSON.stringify(row.color)}:\n`+previewProgram(row.then,depth+1)+indent+'else:\n'+previewProgram(row.else,depth+1);
  else if(row.op==='log')code+=indent+`self.get_logger().info(str(${JSON.stringify(row.message)}))\n`;
  else if(row.op==='wait')code+=indent+`wait_with_cancellation(${row.seconds}, cancel, deadline)\n`;
  else if(row.op==='stop')code+=indent+'station.stop()\n'+indent+`raise SystemExit("STOP block completed; program ended")\n`;
  else code+=indent+`result = station.command(${JSON.stringify(row)}, cancel, deadline)\n`;
 }
 return code;
}
clicked('python',()=>{const code='# Control-flow learning preview; not standalone execution code.\n# Actual execution uses the guarded ProgramRunner with cancellation and deadlines.\n# sort_cycle requires a commissioned station; native cycles is fixed to 1.\n'+previewProgram(compile());$('code').textContent=code;$('preview').showModal();});
const operatorCommands=['emergency_reset','reset','start','base_release','base_confirm','teleop','teleop_relative','teleop_absolute','moveit','damping','zero_torque','restart_moveit','restart_camera','restart_depth','restart_tags','restart_all','verify_restart'];
const recover=document.createElement('button');recover.dataset.recovery='true';recover.disabled=true;recover.textContent='Verify recovery / unlock Blockly';recover.onclick=async()=>{if(!confirm('Instructor: inspected and recovered station?'))return;try{show(await api('recover',{}));}catch(error){show(error.message);}};$('instructor').append(recover);
for(const command of operatorCommands){const button=document.createElement('button');button.dataset.command=command;button.textContent=command.replaceAll('_',' ');button.disabled=true;button.onclick=async()=>{if(!confirm('Instructor: '+button.textContent+'?'))return;try{show(await api('instructor',{command}));}catch(error){show(error.message);}};$('instructor').append(button);}
function updateInstructor(status){
 const available=status.physical===true&&status.instructor===true&&!status.operator_busy&&status.job?.state!=='running';
 const source=status.streams?.['/station/operator/status'];
 const fresh=Number.isFinite(source?.age_s)&&source.age_s>=0&&source.age_s<2&&source.data?.schema===1&&source.data.fresh===true;
 for(const button of $('instructor').children){
  if(button.dataset.recovery){
   button.disabled=!available||status.recovery_required!==true;
   button.title=status.recovery_required===true?'Verify inspected station recovery before unlocking Blockly':'No Blockly recovery is pending';
   continue;
  }
  const capability=source?.data?.capabilities?.[button.dataset.command];
  button.disabled=!available||!fresh||capability?.enabled!==true;
  button.title=!fresh?'Operator status is missing or stale':typeof capability?.reason==='string'?capability.reason:'Operator capability is unavailable';
 }
}
$('sim-color').onchange=async()=>{try{await api('simulation/color',{color:$('sim-color').value});}catch(error){show(error.message);}};
let jawEnabled=true;let lastJob='';async function refresh(){try{const status=await api('status');if(typeof status.physical!=='boolean')throw Error('Station mode is unavailable');if(typeof status.physical==='boolean'){knownMode=status.physical;if(!startupModeResolved){startupModeResolved=true;if(knownMode===false&&JSON.stringify(Blockly.serialization.workspaces.save(workspace))===startupWorkspaceSnapshot)example();}}recordingInfo=status.recording_info||{};updateReplayDurations();const nextJaw=!status.physical||Boolean(status.capabilities?.jaw);if(nextJaw!==jawEnabled){jawEnabled=nextJaw;toolbox.contents.flatMap(category=>category.contents||[]).find(b=>b.type==='station_jaw').disabled=!nextJaw;workspace.updateToolbox(toolbox);}$('mode').textContent=status.physical?'PHYSICAL · instructor supervised':'SIMULATION · no robot motion';$('mode').style.background=status.physical?'#a54529':'#24566b';$('recordings').textContent='Available strokes: '+status.recordings.join(', ')+(status.recording?' · recording '+status.recording:'');updateInstructor(status);$('record-start').disabled=status.recovery_required===true||status.job?.state==='running'||Boolean(status.recording);$('record-start').title=status.recovery_required===true?'Inspect and recover the station, then Verify recovery / unlock Blockly before recording':'';$('record-stop').disabled=status.job?.state==='running'||!status.recording;const encoded=JSON.stringify(status.job);if(encoded!==lastJob){show(status.job);lastJob=encoded;}if(status.record_error)show(status.record_error);if(status.recovery_required){const reason=status.job?.error||status.record_error;show('Recovery required'+(reason?': '+reason:'.')+'\nInstructor: inspect station, then load Starter 5 · Reset and reinitialize and run it. After verification succeeds, run arm mode or movement separately.');}$('sim-color-label').hidden=status.physical;$('readiness').textContent=status.physical?JSON.stringify(status.streams,null,2):'Simulation: no physical camera, perception or arm connection';const events=status.events||[];$('trace').textContent=events.length?events.join(' → '):'Run a sequence to see its actions.';if(events.length)$('brick').style.transform='translateX('+((events.length%3)-1)*35+'px)';window.stationStatus=status;}catch(error){knownMode=null;updateInstructor({});$('record-start').disabled=true;$('record-stop').disabled=true;$('mode').textContent='DISCONNECTED';show(error.message);}setTimeout(refresh,700);}
example();startupWorkspaceSnapshot=JSON.stringify(Blockly.serialization.workspaces.save(workspace));refresh();
