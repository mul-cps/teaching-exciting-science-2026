#!/usr/bin/env node
/**
 * Eighteen-slide whole-class workshop deck. Native editable text/teaching
 * diagrams/code; committed photos, one clearly labelled concept illustration.
 * npm install --prefix /tmp/workshop-slides-deps pptxgenjs@4.0.1
 * PPTXGENJS_MODULE=/tmp/workshop-slides-deps/node_modules/pptxgenjs/dist/pptxgen.cjs.js node tools/build_lecture_slides.mjs
 * Requires LibreOffice, pdftoppm, ImageMagick. SOFFICE selects a portable LO.
 * LECTURE_EVIDENCE_DIR selects private previews. Assets are repository-local.
 * Approved JS fallback: preferred @oai/artifact-tool runtime was unavailable.
 */
import fs from 'node:fs/promises';
import path from 'node:path';
import os from 'node:os';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { createRequire } from 'node:module';
import { execFileSync } from 'node:child_process';
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
const require=createRequire(import.meta.url);
let exporterPath;
try {exporterPath=process.env.PPTXGENJS_MODULE?path.resolve(process.env.PPTXGENJS_MODULE):require.resolve('pptxgenjs');}
catch(error){throw new Error('Install pptxgenjs@4.0.1 and set PPTXGENJS_MODULE; see tools/lecture-slides.README.md',{cause:error});}
const {default:PptxGenJS}=await import(pathToFileURL(exporterPath).href);
const pptx=new PptxGenJS();pptx.layout='LAYOUT_WIDE';pptx.author='Exciting Science workshop';
pptx.title='Your first robot factory';pptx.subject='A complete four-hour beginner workshop';pptx.lang='en-GB';
pptx.theme={headFontFace:'Liberation Sans',bodyFontFace:'Liberation Sans',lang:'en-GB'};
const C={navy:'0C2346',paper:'F7F4EB',coral:'F15D51',cyan:'47CEE0',ink:'102A43',gray:'687687',white:'FFFFFF',stone:'DFE3E4'};
const out=path.join(root,'docs');
const evidence=process.env.LECTURE_EVIDENCE_DIR||path.join(os.tmpdir(),'workshop-lecture-preview');
await fs.mkdir(evidence,{recursive:true});
const A=n=>path.join(root,'docs/assets/lecture',n);
const scripts=(await fs.readFile(path.join(out,'lecture-notes.md'),'utf8')).split(/^## Slide \d+: /m).slice(1);
if(scripts.length!==18)throw new Error('Expected 18 speaker-note sections');
function txt(s,t,x,y,w,h,size=27,opt={}){s.addText(t,{x,y,w,h,fontFace:'Liberation Sans',fontSize:size,color:C.ink,margin:0,valign:'mid',breakLine:false,paraSpaceAfterPt:0,...opt});}
function rect(s,x,y,w,h,color){s.addShape(pptx.ShapeType.rect,{x,y,w,h,line:{color,transparency:100},fill:{color}});}
const imageDimensions=new Map();
function photo(s,n,x,y,w,h,alt,contain=false){
 const p=A(n);if(!imageDimensions.has(p))imageDimensions.set(p,execFileSync('magick',['identify','-format','%w %h',p],{encoding:'utf8'}).trim().split(' ').map(Number));
 const [iw,ih]=imageDimensions.get(p);
 if(contain){const k=Math.min(w/iw,h/ih),dw=iw*k,dh=ih*k;s.addImage({path:p,x:x+(w-dw)/2,y:y+(h-dh)/2,w:dw,h:dh,altText:alt});}
 else{s.addImage({path:p,x,y,w:iw/96,h:ih/96,sizing:{type:'cover',w,h},altText:alt});}
}
function line(s,x,y,x2,y2,color=C.coral,arrow=false,width=2.5){s.addShape(pptx.ShapeType.line,{x,y,w:x2-x,h:y2-y,line:{color,width,...(arrow?{endArrowType:'triangle'}:{})}});}
function make(n,bg=C.paper){const s=pptx.addSlide();s.background={color:bg};s.addNotes(('Slide '+n+': '+scripts[n-1]).replaceAll('**','').replaceAll('`',''));return s;}
function head(s,t,color=C.ink){txt(s,t,.65,.43,12.0,.78,40,{bold:true,color});}
function time(s,t,dark=false){txt(s,t,10.15,.42,2.5,.62,30,{bold:true,color:dark?C.cyan:C.navy,align:'right'});}
function label(s,t,x,y,w,h){txt(s,t,x,y,w,h,22,{bold:true,fill:{color:C.paper},margin:.07});}
// INTRO: 25 minutes, ten visual ideas.
{
 const s=make(1,C.navy);photo(s,'factory-concept-hero.png',0,0,13.333,7.5,'Clearly illustrated concept of a tiny sorting factory');
 txt(s,'Your first\nrobot factory',.65,1.0,5.7,2.15,52,{bold:true,color:C.white});
 txt(s,'You write the decisions.',.68,3.63,5.4,.8,29,{color:C.cyan});
 txt(s,'What must\nit know?',.68,5.0,3.8,1.2,32,{bold:true,color:C.white});
 txt(s,'Concept illustration',.68,6.85,4.5,.37,22,{color:C.white});
}
{
 const s=make(2);head(s,'Where does it go?');
 const items=[['RED','left',C.coral],['BLUE','right','08788A'],['UNKNOWN','reject',C.gray]];
 items.forEach(([a,b,c],i)=>{const x=.7+i*4.2;txt(s,a,x+.1,2.2,3.35,.7,34,{bold:true,color:c,align:'center'});line(s,x+1.78,3.12,x+1.78,3.55,c,true);line(s,x,3.7,x,5.08,c,false,5);line(s,x,5.08,x+3.55,5.08,c,false,5);line(s,x+3.55,5.08,x+3.55,3.7,c,false,5);txt(s,b,x+.15,4.0,3.25,.65,32,{color:C.navy,align:'center'});});
 txt(s,'Predict before we write the rule.',.7,5.82,11.8,.75,30,{color:C.navy,bold:true});
 txt(s,'What should happen to a green object?',.7,6.68,11.8,.45,24,{color:C.gray});
}
{
 const s=make(3);photo(s,'station-current.png',0,1.25,13.333,6.25,'Actual station camera capture with white follower arm, black leader and conveyor');
 head(s,'The equipment in our room');
 label(s,'Camera above this view',.65,1.55,3.65,.62);
 label(s,'Conveyor\nCarries objects',5.1,2.1,3.1,1.0);line(s,6.65,3.15,7.7,3.65,C.coral);
 label(s,'Robot arm + gripper\nMoves and holds',9.35,5.65,3.5,1.1);line(s,10.1,5.57,10.9,4.6,C.coral);
}
{
 const s=make(4,C.navy);head(s,'A complete cycle',C.white);
 const rows=[['Observe','Get a picture'],['Decide','Choose a destination'],['Act','Make a movement'],['Check','Did it arrive?']];
 rows.forEach(([a,b],i)=>{let y=1.65+i*1.12;txt(s,a,.8,y,4.5,.73,43,{bold:true,color:C.white});txt(s,b,6.0,y,6.0,.73,29,{color:C.cyan});if(i<3)line(s,2.0,y+.80,2.0,y+1.04,C.coral,true);});
 line(s,12.45,5.43,12.75,5.43,C.coral);line(s,12.75,5.43,12.75,1.95,C.coral);line(s,12.75,1.95,12.2,1.95,C.coral,true);
 txt(s,'Which step tells us whether it worked?',.8,6.65,11.6,.5,27,{color:C.white});
}
{
 const s=make(5);head(s,'Information and movement');
 txt(s,'Sensor',.75,1.75,5.3,.95,48,{bold:true,color:C.navy});txt(s,'measures',.75,2.9,5.3,.72,34,{color:C.gray});
 txt(s,'camera',.75,4.3,2.3,.7,30,{color:C.navy});line(s,3.15,4.66,3.65,4.66,C.cyan,true);txt(s,'image',3.9,4.3,2.1,.7,30,{bold:true,color:C.navy});
 txt(s,'Actuator',7.1,1.75,5.4,.95,48,{bold:true,color:C.coral});txt(s,'moves',7.1,2.9,5.3,.72,34,{color:C.gray});
 txt(s,'motor',7.1,4.3,2.2,.7,30,{color:C.navy});line(s,9.5,4.66,10.0,4.66,C.coral,true);txt(s,'turn',10.2,4.3,2.2,.7,30,{bold:true,color:C.navy});
 txt(s,'Which one can tell us about the color?',.75,6.36,11.8,.65,29,{color:C.navy,bold:true});
}
{
 const s=make(6,C.navy);photo(s,'digital-twin-view.png',5.55,0,7.783,7.5,'Actual supplied side view with the complete robot arm and gripper',true);
 txt(s,'You move.\nIt follows.',.65,.8,4.7,1.75,46,{bold:true,color:C.white});
 txt(s,'Leader: your handle\nFollower: robot arm\nTwin: live screen model',.65,3.0,4.7,2.1,25,{color:C.white,lineSpacingMultiple:1.32});
 txt(s,'A joint bends or rotates.\nThe gripper opens and closes.',.65,5.75,4.75,1.1,23,{color:C.cyan});
 label(s,'Actual station screen',9.2,6.84,3.65,.45);
}
{
 const s=make(7);head(s,'What does the camera give us?');
 const xs=[.72,5.05,9.35];txt(s,'Picture',xs[0],1.66,3.0,.6,27,{bold:true});txt(s,'Pixels',xs[1],1.66,3.0,.6,27,{bold:true});txt(s,'Label',xs[2],1.66,3.0,.6,27,{bold:true});
 photo(s,'brick-red.png',xs[0],2.57,2.9,2.9,'Actual red brick crop');photo(s,'brick-pixels.png',xs[1],2.57,2.9,2.9,'Deliberately reduced pixel enlargement of that crop');
 line(s,3.91,4.03,4.62,4.03,C.coral,true);line(s,8.23,4.03,8.94,4.03,C.coral,true);txt(s,'“red”',9.35,3.3,3.3,1.18,48,{bold:true,color:C.coral});
 txt(s,'Pixels are tiny squares of color.',.72,6.4,11.8,.65,29,{color:C.navy});
}
{
 const s=make(8);rect(s,6.667,0,6.667,7.5,C.navy);
 txt(s,'Write a rule',.7,.7,5.5,.85,40,{bold:true,color:C.navy});txt(s,'Teach examples',7.34,.7,5.35,.85,40,{bold:true,color:C.white});
 txt(s,'red',.75,2.75,2.0,.7,36,{bold:true,color:C.coral});line(s,2.85,3.12,3.45,3.12,C.coral,true);txt(s,'left',3.8,2.75,2.0,.7,36,{color:C.navy});
 txt(s,'blue',.75,4.0,2.0,.7,36,{bold:true,color:'08788A'});line(s,2.85,4.37,3.45,4.37,C.navy,true);txt(s,'right',3.8,4.0,2.0,.7,36,{color:C.navy});
 txt(s,'red examples\nblue examples\nempty background',7.34,2.25,5.2,2.85,31,{color:C.white,lineSpacingMultiple:1.26});
 txt(s,'We write the choice.',.75,6.25,5.5,.6,27,{bold:true,color:C.navy});txt(s,'We show the patterns.',7.34,6.25,5.3,.6,27,{bold:true,color:C.cyan});
}
{
 const s=make(9);head(s,'A rule the computer can run');
 txt(s,'A name stores\na value.',.7,1.7,4.5,1.2,36,{bold:true,color:C.navy});
 txt(s,'COLOR stores text.\nQuotes mean text.\nif asks a question.',.7,3.42,4.6,1.65,25,{color:C.gray,lineSpacingMultiple:1.2});
 txt(s,'COLOR = "red"\nif COLOR == "red":\n    print("left")\nelif COLOR == "blue":\n    print("right")\nelse:\n    print("reject")',5.64,1.62,7.05,4.5,27,{fontFace:'Noto Sans Mono',color:C.navy,lineSpacingMultiple:1.12});
 txt(s,'Change red to blue. What will appear?',.7,6.62,11.9,.52,28,{color:C.navy,bold:true});
}
{
 const s=make(10);rect(s,6.667,0,6.667,7.5,C.coral);
 txt(s,'No matching color',.7,.8,5.45,.8,32,{bold:true,color:C.navy});
 txt(s,'REJECT',.7,2.5,5.45,1.2,61,{bold:true,color:C.coral});
 txt(s,'Unknown or missing input\nuses the reject destination.',.7,4.3,5.4,1.3,27,{color:C.navy});
 txt(s,'Unexpected movement',7.34,.8,5.4,.8,31,{bold:true,color:C.navy});
 txt(s,'STOP',7.34,2.5,5.4,1.2,75,{bold:true,color:C.navy});
 txt(s,'Ask the instructor\nbefore another physical action.',7.34,4.3,5.35,1.3,27,{color:C.navy});
 txt(s,'Check the result.',.7,6.65,5.4,.5,27,{bold:true,color:C.navy});
}
// ACTIVITY CHECKPOINTS: their durations total the remaining 215 minutes.
{
 const s=make(11,C.navy);time(s,'10 minutes',true);txt(s,'Build a path',.7,1.12,6.5,1.2,48,{bold:true,color:C.white});
 txt(s,'Predict. Build. Run.',.7,2.8,6.3,.75,32,{color:C.cyan});txt(s,'Levels 1–3 first.\n4–5 if time remains.',.7,4.0,6.2,1.3,28,{color:C.white});
 photo(s,'blockly-view.png',7.3,1.6,5.35,4.65,'Actual Blockly Maze browser view; students start at level 1',true);
 txt(s,'Finish: explain one correction.',.7,6.55,11.8,.55,27,{color:C.white});
}
{
 const s=make(12);time(s,'30 minutes');txt(s,'Change one thing',.7,1.1,10.3,1.12,47,{bold:true,color:C.navy});
 txt(s,'Predict the output.\nRun the cell.\nChange the value.',.75,2.83,5.65,2.1,30,{color:C.navy,lineSpacingMultiple:1.3});
 txt(s,'COLOR = "blue"',7.03,2.8,5.65,.8,34,{fontFace:'Noto Sans Mono',color:C.navy});txt(s,'?',8.8,3.75,2.0,1.45,86,{bold:true,color:C.coral,align:'center'});
 txt(s,'Finish: a result you predicted correctly.',.75,6.54,11.8,.55,28,{bold:true,color:C.navy});
}
{
 const s=make(13,C.navy);time(s,'20 minutes',true);txt(s,'Teach three answers',.7,1.1,11.5,1.1,47,{bold:true,color:C.white});
 txt(s,'red',.75,2.87,3.2,.8,42,{bold:true,color:C.coral});txt(s,'blue',4.82,2.87,3.2,.8,42,{bold:true,color:C.cyan});txt(s,'background',8.58,2.87,4.1,.8,34,{bold:true,color:C.white});
 txt(s,'Examples',.75,4.55,2.5,.65,29,{color:C.white});line(s,3.6,4.88,4.3,4.88,C.coral,true);txt(s,'Train',4.82,4.55,2.0,.65,29,{color:C.white});line(s,7.24,4.88,7.98,4.88,C.coral,true);txt(s,'A new view',8.58,4.55,4.1,.65,29,{color:C.white});
 txt(s,'Finish: explain one mistake or limitation.',.75,6.55,11.9,.55,27,{color:C.white});
}
{
 const s=make(14,C.navy);txt(s,'A real break',.75,1.3,6.5,1.1,48,{bold:true,color:C.white});txt(s,'10',7.55,1.7,4.3,2.8,142,{bold:true,color:C.cyan,align:'center'});
 txt(s,'Return at minute 95',.75,3.35,6.6,.82,31,{color:C.white});txt(s,'minutes',7.55,4.77,4.3,.75,35,{color:C.cyan,align:'center'});
 txt(s,'Leave station controls with the instructor.',.75,6.58,11.8,.5,24,{color:C.white});
}
{
 const s=make(15);time(s,'45 minutes');txt(s,'Three stations',.7,1.0,9.2,1.1,48,{bold:true,color:C.navy});
 const rows=[['1','Robot + digital twin','Compare a real joint with the screen.',C.coral],['2','Camera + conveyor','Find one reliable reading.', '08788A'],['3','Virtual robot','Predict one joint movement.',C.navy]];
 rows.forEach(([n,t,b,c],i)=>{let y=2.4+i*1.27;txt(s,n,.75,y,.7,.85,48,{bold:true,color:c});txt(s,t,1.85,y,10.8,.6,30,{bold:true,color:C.navy});txt(s,b,1.85,y+.64,10.8,.48,24,{color:C.gray});});
 txt(s,'Every 15 minutes, move to the next station.',.75,6.63,11.8,.48,25,{color:C.navy});
}
{
 const s=make(16,C.navy);time(s,'50 minutes',true);txt(s,'Make the whole\nfactory work',.7,1.0,8.7,1.8,48,{bold:true,color:C.white});
 txt(s,'5',10.08,2.7,2.5,1.85,110,{bold:true,color:C.cyan,align:'center'});
 txt(s,'red, blue, red, unknown, blue',.75,3.65,9.0,.75,29,{color:C.white});
 txt(s,'2 left + 2 right + 1 reject',.75,4.88,9.2,.75,33,{bold:true,color:C.cyan});
 txt(s,'Test in simulation first.\nFinish: five recorded outcomes.',.75,6.04,11.8,1.0,27,{color:C.white});
}
{
 const s=make(17);time(s,'35 minutes');txt(s,'Choose an upgrade',.7,1.05,10.8,1.1,47,{bold:true,color:C.navy});
 txt(s,'Swap red and blue destinations.',.75,2.72,11.9,.8,31,{bold:true,color:C.navy});txt(s,'Count the results.',.75,3.8,11.9,.8,31,{bold:true,color:C.navy});
 txt(s,'Stretch: a new color or a bad reading.',.75,5.0,11.9,.7,27,{color:C.gray});txt(s,'Finish: show a test that caught a mistake.',.75,6.55,11.9,.55,27,{color:C.navy,bold:true});
}
{
 const s=make(18,C.cyan);time(s,'15 minutes');txt(s,'Show your factory',.7,1.05,11.7,1.15,50,{bold:true,color:C.navy});
 txt(s,'Rule.\nPrediction.\nResult.',.75,2.83,5.8,2.6,43,{bold:true,color:C.navy,lineSpacingMultiple:1.1});
 txt(s,'Show one change.\nExplain one limitation.\nSwap who speaks.',7.1,3.0,5.6,2.15,29,{color:C.navy,lineSpacingMultiple:1.25});
 txt(s,'What would you improve next?',.75,6.55,11.9,.6,31,{bold:true,color:C.navy});
}
if(pptx._slides.length!==18)throw new Error('Expected 18 slides');
const pptxPath=path.join(out,'workshop-slides.pptx');await pptx.writeFile({fileName:pptxPath});
// Remove only this exporter version's ghost shared-master declarations.
const JSZip=createRequire(exporterPath)('jszip');const zip=await JSZip.loadAsync(await fs.readFile(pptxPath));
const types=await zip.file('[Content_Types].xml').async('string');zip.file('[Content_Types].xml',types.replace(/<Override\b[^>]*PartName="(\/ppt\/slideMasters\/slideMaster\d+\.xml)"[^>]*\/>/g,(d,p)=>zip.file(p.slice(1))?d:''));
await fs.writeFile(pptxPath,await zip.generateAsync({type:'nodebuffer',compression:'DEFLATE'}));
const soffice=process.env.SOFFICE||'libreoffice';
execFileSync(soffice,[`-env:UserInstallation=${pathToFileURL(path.join(evidence,'lo-profile')).href}`,'--headless','--convert-to','pdf','--outdir',evidence,pptxPath],{stdio:'inherit'});
await fs.copyFile(path.join(evidence,'workshop-slides.pdf'),path.join(out,'workshop-slides.pdf'));
execFileSync('pdftoppm',['-png','-r','120',path.join(out,'workshop-slides.pdf'),path.join(evidence,'class-slide')],{stdio:'inherit'});
execFileSync('magick',['montage',...Array.from({length:18},(_,i)=>path.join(evidence,`class-slide-${String(i+1).padStart(2,'0')}.png`)),'-thumbnail','640x360','-tile','3x6','-geometry','+12+12','-background',`#${C.paper}`,path.join(out,'presentation-preview.png')],{stdio:'inherit'});
await fs.writeFile(path.join(evidence,'build-receipt.json'),JSON.stringify({slideCount:18,introMinutes:[3,2,3,3,2,3,2,2,3,2],activityMinutes:[10,30,20,10,45,50,35,15],totalMinutes:240,pptx:pptxPath,pdf:path.join(out,'workshop-slides.pdf'),authoring:'PptxGenJS 4.0.1 approved fallback',fonts:['Liberation Sans','Noto Sans Mono'],conceptIllustrationSlides:[1]},null,2));
console.log(`Built 18-slide whole-class deck: ${pptxPath}`);
