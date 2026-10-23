#!/usr/bin/env node
/** Eight introductory slides followed by sixteen editable practical mission cards. Authored text/notes/assets: curriculum/workshop.json. */
import fs from 'node:fs/promises';
import path from 'node:path';
import os from 'node:os';
import {fileURLToPath,pathToFileURL} from 'node:url';
import {createRequire} from 'node:module';
import {execFileSync} from 'node:child_process';
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
const c=JSON.parse(await fs.readFile(path.join(root,'curriculum/workshop.json'),'utf8'));
if(c.slides.length!==8||c.slides.reduce((n,s)=>n+s.minutes,0)!==20)throw Error('Eight intro slides / 20 minutes required');
const require=createRequire(import.meta.url);
const modulePath=process.env.PPTXGENJS_MODULE||'/tmp/workshop-slides-deps/node_modules/pptxgenjs/dist/pptxgen.cjs.js';
const {default:PptxGenJS}=await import(pathToFileURL(modulePath).href);
const pptx=new PptxGenJS();pptx.layout='LAYOUT_WIDE';pptx.author='Exciting Science workshop';pptx.title=c.title;pptx.lang='en-GB';
pptx.theme={headFontFace:'Liberation Sans',bodyFontFace:'Liberation Sans',lang:'en-GB'};
const C={ink:'17354B',teal:'197F80',paper:'F7FAFB',pale:'E4EFF0',white:'FFFFFF',red:'C34C42'};
const evidence=process.env.LECTURE_EVIDENCE_DIR||'/tmp/workshop-lecture-preview';await fs.mkdir(evidence,{recursive:true});
function text(s,t,x,y,w,h,size=28,opts={}){s.addText(t,{x,y,w,h,fontFace:'Liberation Sans',fontSize:size,color:C.ink,margin:0,breakLine:false,valign:'mid',...opts});}
function rect(s,x,y,w,h,color){s.addShape(pptx.ShapeType.rect,{x,y,w,h,line:{color,transparency:100},fill:{color}});}
function image(s,name,x,y,w,h){const p=path.join(root,'docs/assets/lecture',name);const dims=execFileSync('magick',['identify','-format','%w %h',p],{encoding:'utf8'}).split(' ').map(Number);const scale=Math.min(w/dims[0],h/dims[1]);const iw=dims[0]*scale,ih=dims[1]*scale;s.addImage({path:p,x:x+(w-iw)/2,y:y+(h-ih)/2,w:iw,h:ih,altText:name});}
for(const [index,spec] of c.slides.entries()){
 const s=pptx.addSlide();s.background={color:C.paper};s.addNotes(`${spec.minutes} minutes\n${spec.notes}`);
 text(s,spec.title,.6,.36,11.8,.75,36,{bold:true});
 text(s,`${index+1}/8  ·  ${spec.minutes} min`,10.7,7.02,2.0,.25,12,{align:'right',color:C.teal});
 if(index===0){image(s,spec.asset,.6,1.35,8.1,4.8);spec.lines.forEach((t,i)=>text(s,t,9.05,1.8+i*1.1,3.55,.85,25,{bold:i===0}));text(s,'Actual station photograph · one fixed pickup and one fixed destination',.6,6.45,11.8,.4,20);}
 else if(index===1){
  image(s,spec.asset,.65,1.55,5.4,4.95);
  spec.lines.forEach((t,i)=>{const y=1.65+i*1.12;rect(s,7.0,y,5.55,.72,C.pale);text(s,t,7.25,y+.08,5.05,.55,28,{bold:i===0});if(i<3)s.addShape(pptx.ShapeType.line,{x:9.75,y:y+.75,w:0,h:.30,line:{color:C.teal,width:2,endArrowType:'triangle'}});});
 }
 else if(index===3){
  spec.lines.forEach((t,i)=>{let y=1.55+i*.94;rect(s,.8,y,10.8,.65,i%2?C.pale:'EDF3F5');text(s,`${i+1}   ${t}`,1.05,y+.05,10.3,.5,28,{bold:i===1});if(i<4)s.addShape(pptx.ShapeType.line,{x:6.1,y:y+.67,w:0,h:.24,line:{color:C.teal,width:2,endArrowType:'triangle'}});});
 }
 else if(index===6){rect(s,.65,1.65,12,3.85,C.ink);text(s,spec.lines.join('\n'),1,2.02,11.3,3.05,34,{fontFace:'Noto Sans Mono',color:C.white,breakLine:false});text(s,'Predict the output, then change one value.',.7,6.1,11.7,.6,28,{bold:true,color:C.teal});}
 else if(index===5){image(s,spec.asset,.6,1.45,9.1,5.1);spec.lines.forEach((t,i)=>text(s,t,10,1.9+i*1.25,2.7,.95,25,{bold:i===2}));}
 else{image(s,spec.asset,.65,1.55,6.3,4.95);spec.lines.forEach((t,i)=>text(s,t,7.4,1.75+i*(4.7/spec.lines.length),5.15,.9,28,{bold:i===0,color:index===4&&i===1?C.red:C.ink}));}
}
// Two distance-readable native slide cards per mission; full text stays in print/notebook.
for(const [missionIndex,mission] of c.missions.entries()){
 const card=mission.slide_card;
 const notes=[`${mission.minutes} minute activity. These two cards share that one activity duration.`,
   ...['worked_example','predict','change','run','observe','explain','success','offline_route','extension'].map(k=>`${k}: ${mission[k]}`),
   ...mission.hints,`Instructor: ${mission.instructor_notes}`].join('\n');
 const pages=[
  [['Worked example',card.worked_example],['Predict',card.predict],['Change',card.change],['Run',card.run]],
  [['Observe',card.observe],['Explain',card.explain],['Success',card.success],['Hints',card.hints.join('\n')]]
 ];
 for(const [part,rows] of pages.entries()){
  const slide=pptx.addSlide();slide.background={color:C.paper};slide.addNotes(notes);
  text(slide,mission.title,.65,.32,12,.72,32,{bold:true});
  text(slide,`Mission ${missionIndex+1} · ${mission.minutes} minutes total · ${part===0?'Predict and run':'Check and explain'}`,.7,1.12,11.9,.42,22,{color:C.teal,bold:true});
  rows.forEach(([label,body],i)=>{
   const y=1.85+i*1.10;
   text(slide,label,.72,y,2.10,.75,22,{bold:true,color:C.teal});
   text(slide,body,2.98,y,9.55,.89,24,{valign:'top',breakLine:false});
  });
  if(part===1)text(slide,`Optional: ${card.extension}`,.72,6.50,11.9,.68,22,{color:C.ink});
 }
}
const slideCount=c.slides.length+2*c.missions.length;
if(pptx._slides.length!==slideCount)throw Error('Full deck slide count differs from source');
const out=path.join(root,'docs');const target=path.join(out,'workshop-slides.pptx');await pptx.writeFile({fileName:target});
const JSZip=createRequire(modulePath)('jszip');const zip=await JSZip.loadAsync(await fs.readFile(target));
const types=await zip.file('[Content_Types].xml').async('string');zip.file('[Content_Types].xml',types.replace(/<Override\b[^>]*PartName="(\/ppt\/slideMasters\/slideMaster\d+\.xml)"[^>]*\/>/g,(d,p)=>zip.file(p.slice(1))?d:''));
await fs.writeFile(target,await zip.generateAsync({type:'nodebuffer',compression:'DEFLATE'}));
execFileSync(process.env.SOFFICE||'soffice',[`-env:UserInstallation=${pathToFileURL(path.join(evidence,'lo-profile')).href}`,'--headless','--convert-to','pdf','--outdir',evidence,target],{stdio:'inherit'});
await fs.copyFile(path.join(evidence,'workshop-slides.pdf'),path.join(out,'workshop-slides.pdf'));
execFileSync('pdftoppm',['-png','-r','110',path.join(out,'workshop-slides.pdf'),path.join(evidence,'slide')]);
const pages=Array.from({length:slideCount},(_,i)=>path.join(evidence,`slide-${String(i+1).padStart(2,'0')}.png`));
execFileSync('magick',['montage',...pages,'-thumbnail','640x360','-tile','3x8','-geometry','+10+10',path.join(out,'presentation-preview.png')]);
await fs.writeFile(path.join(evidence,'build-receipt.json'),JSON.stringify({slideCount,introSlideCount:8,missionSlideCount:2*c.missions.length,missionIds:c.missions.map(m=>m.id),introMinutes:c.slides.map(s=>s.minutes),totalMinutes:c.sessions.reduce((n,s)=>n+s.minutes,0),source:'curriculum/workshop.json',authoring:'PptxGenJS 4.0.1 native editable text and shapes',assets:c.slides.map(s=>s.asset)},null,2));
console.log(`Built ${slideCount} editable slides: eight introductory slides (20 minutes) plus ${2*c.missions.length} practical mission cards.`);
