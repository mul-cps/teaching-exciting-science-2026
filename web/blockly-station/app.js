/* Offline Blockly station extension. Only strict JSON opcodes are executable. */
registerRosBlocky(Blockly,python.pythonGenerator);
const definitions=[
 {type:'station_ev3',message0:'EV3 %1',args0:[{type:'field_dropdown',name:'COMMAND',options:[['dispense one brick','eject'],['conveyor home','home'],['conveyor left','left_end'],['conveyor right','right_end'],['wake','wake'],['sleep','sleep'],['connect','connect']]}],previousStatement:null,nextStatement:null,colour:180},
 {type:'station_wait',message0:'wait %1 seconds',args0:[{type:'field_number',name:'SECONDS',value:1,min:0,max:10,precision:.1}],previousStatement:null,nextStatement:null,colour:210},
 {type:'station_replay',message0:'replay stroke %1 speed %2',args0:[{type:'field_input',name:'NAME',text:'demo'},{type:'field_number',name:'SPEED',value:.5,min:.1,max:1,precision:.1}],previousStatement:null,nextStatement:null,colour:30},
 {type:'station_jaw',message0:'jaw gap %1 mm',args0:[{type:'field_number',name:'GAP',value:30,min:0,max:200,precision:1}],previousStatement:null,nextStatement:null,colour:30},
 {type:'station_repeat',message0:'repeat %1 times %2 %3',args0:[{type:'field_number',name:'COUNT',value:2,min:1,max:5,precision:1},{type:'input_dummy'},{type:'input_statement',name:'BODY'}],previousStatement:null,nextStatement:null,colour:120},
 {type:'station_color',message0:'if sensor sees %1 %2 do %3 otherwise %4',args0:[{type:'field_dropdown',name:'COLOR',options:['red','blue','green','yellow','unknown'].map(c=>[c,c])},{type:'input_dummy'},{type:'input_statement',name:'THEN'},{type:'input_statement',name:'ELSE'}],previousStatement:null,nextStatement:null,colour:280},
 {type:'station_stop',message0:'STOP station and end program',previousStatement:null,nextStatement:null,colour:0}
];
Blockly.defineBlocksWithJsonArray(definitions);
const toolbox={kind:'categoryToolbox',contents:[{kind:'category',name:'Station sequence',colour:'180',contents:definitions.map(d=>({kind:'block',type:d.type}))},{kind:'category',name:'ROS Blocky · logging',colour:'#7f8c8d',contents:[{kind:'block',type:'node_logging_info',inputs:{MESSAGE:{shadow:{type:'node_logging_text'}}}}]}]};
const workspace=Blockly.inject('workspace',{toolbox,media:'vendor/media/',trashcan:true,scrollbars:true,zoom:{controls:true,wheel:true},sounds:false});
function compileChain(block,depth=0){
 if(depth>4)throw Error('Maximum nesting is four'); const rows=[];
 while(block){
  const f=n=>block.getFieldValue(n); let row;
  switch(block.type){
   case'station_ev3':row={op:'ev3',command:f('COMMAND')};break;
   case'station_wait':row={op:'wait',seconds:Number(f('SECONDS'))};break;
   case'station_replay':row={op:'replay',name:f('NAME'),speed:Number(f('SPEED'))};break;
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
function compile(){const tops=workspace.getTopBlocks(true).filter(b=>!b.outputConnection);if(tops.length!==1)throw Error('Connect blocks into one sequence');return compileChain(tops[0]);}
const $=id=>document.getElementById(id);
async function api(path,body){const base=path==='program/stop'?'http://127.0.0.1:'+(Number(location.port)+1):'';const response=await fetch(base+'/api/'+path,{method:body===undefined?'GET':'POST',headers:{'X-Station-Token':window.STATION_TOKEN,'Content-Type':'application/json'},body:body===undefined?undefined:JSON.stringify(body)});const data=await response.json();if(!response.ok)throw Error(data.error||response.statusText);return data;}
function show(data){$('result').textContent=typeof data==='string'?data:JSON.stringify(data,null,2);}
function clicked(id,fn){$(id).onclick=async()=>{try{await fn();}catch(error){show(error.message);}};}
function example(){workspace.clear();Blockly.serialization.workspaces.load({blocks:{languageVersion:0,blocks:[{type:'station_repeat',x:35,y:35,fields:{COUNT:2},inputs:{BODY:{block:{type:'station_ev3',fields:{COMMAND:'eject'},next:{block:{type:'station_wait',fields:{SECONDS:1},next:{block:{type:'station_replay',fields:{NAME:'demo',SPEED:.5}}}}}}}}}]}},workspace);}
clicked('example',example);clicked('run',async()=>show(await api('program/run',{program:compile()})));clicked('stop',async()=>show(await api('program/stop',{})));
clicked('record-start',async()=>show(await api('record/start',{name:$('record-name').value})));
clicked('record-stop',async()=>show(await api('record/stop',{})));
clicked('save',()=>{const data={schema:'so101-blockly-workspace-v1',workspace:Blockly.serialization.workspaces.save(workspace)};const link=document.createElement('a');link.href=URL.createObjectURL(new Blob([JSON.stringify(data,null,2)],{type:'application/json'}));link.download='factory-blocks.json';link.click();setTimeout(()=>URL.revokeObjectURL(link.href),1000);});
$('load').onchange=async event=>{try{const file=event.target.files[0];if(!file||file.size>262144)throw Error('Choose a workspace smaller than 256 KB');const data=JSON.parse(await file.text());if(data.schema!=='so101-blockly-workspace-v1')throw Error('Unsupported workspace format');workspace.clear();Blockly.serialization.workspaces.load(data.workspace,workspace);}catch(error){show(error.message);}};
clicked('python',()=>{python.pythonGenerator.init(workspace);let code='# ROS Blocky learning preview\n# Station automation executes through the guarded JSON adapter.\n';for(const block of workspace.getAllBlocks(false)){if(block.type==='node_logging_info')code+=python.pythonGenerator.blockToCode(block);}$('code').textContent=code;$('preview').showModal();});
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
let jawEnabled=true;let lastJob='';async function refresh(){try{const status=await api('status');const nextJaw=!status.physical||Boolean(status.capabilities?.jaw);if(nextJaw!==jawEnabled){jawEnabled=nextJaw;toolbox.contents[0].contents.find(b=>b.type==='station_jaw').disabled=!nextJaw;workspace.updateToolbox(toolbox);}$('mode').textContent=status.physical?'PHYSICAL · instructor supervised':'SIMULATION · no robot motion';$('mode').style.background=status.physical?'#a54529':'#24566b';$('recordings').textContent='Available strokes: '+status.recordings.join(', ')+(status.recording?' · recording '+status.recording:'');updateInstructor(status);const encoded=JSON.stringify(status.job);if(encoded!==lastJob){show(status.job);lastJob=encoded;}if(status.record_error)show(status.record_error);if(status.recovery_required)show('Recovery required: inspect station, use instructor recovery controls, then Verify recovery / unlock Blockly.');$('sim-color-label').hidden=status.physical;$('readiness').textContent=status.physical?JSON.stringify(status.streams,null,2):'Simulation: no physical camera, perception or arm connection';const events=status.events||[];$('trace').textContent=events.length?events.join(' → '):'Run a sequence to see its actions.';if(events.length)$('brick').style.transform='translateX('+((events.length%3)-1)*35+'px)';window.stationStatus=status;}catch(error){updateInstructor({});$('mode').textContent='DISCONNECTED';show(error.message);}setTimeout(refresh,700);}
example();refresh();
