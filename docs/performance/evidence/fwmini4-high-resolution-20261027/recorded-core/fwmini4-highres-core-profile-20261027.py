import pathlib,json,subprocess,time,hashlib
import pyrealsense2 as rs
root=pathlib.Path('/home/bjoern/.local/share/exciting-science/validation/highres-20261027')
binary=root/'install/so101_station_vulkan/lib/so101_station_vulkan/benchmark_rgbd'
summary=[]
for label,cw,ch in [('720p',1280,720),('1080p',1920,1080)]:
 out=root/('recorded-'+label);out.mkdir(exist_ok=False)
 pipe=rs.pipeline();config=rs.config();config.enable_device('243322071783');config.enable_stream(rs.stream.color,cw,ch,rs.format.rgb8,30);config.enable_stream(rs.stream.depth,1280,720,rs.format.z16,30)
 try:
  profile=pipe.start(config)
  for _ in range(30): frames=pipe.wait_for_frames(5000)
  original=pipe.wait_for_frames(5000);aligned=rs.align(rs.stream.color).process(original);color=aligned.get_color_frame();depth=aligned.get_depth_frame();k=color.profile.as_video_stream_profile().get_intrinsics()
  assert (color.get_width(),color.get_height(),depth.get_width(),depth.get_height())==(cw,ch,cw,ch)
  def plane(frame,bpp):
   data=bytes(frame.get_data());stride=frame.get_stride_in_bytes();row=cw*bpp
   return b''.join(data[y*stride:y*stride+row] for y in range(ch))
  rgb,z16=plane(color,3),plane(depth,2);assert len(rgb)==cw*ch*3 and len(z16)==cw*ch*2
  (out/'rgb.bin').write_bytes(rgb);(out/'depth.bin').write_bytes(z16)
  scale=profile.get_device().first_depth_sensor().get_depth_scale()
  meta={'profile':label,'width':cw,'height':ch,'fx':k.fx,'fy':k.fy,'cx':k.ppx,'cy':k.ppy,'depth_scale':scale,'source_frame':color.get_frame_number(),'depth_source_frame':depth.get_frame_number(),'rgb_sha256':hashlib.sha256(rgb).hexdigest(),'depth_sha256':hashlib.sha256(z16).hexdigest(),'metric_scope':'frozen SDK aligned Core only; excludes acquisition and alignment'}
  (out/'metadata.json').write_text(json.dumps(meta,indent=2)+'\n')
 finally:pipe.stop()
 for mode,frames,warmup,timestamps in [('compare',2,1,0),('cpu',20,5,0),('vulkan',20,5,1)]:
  command=['/usr/bin/timeout','--signal=TERM','--kill-after=5s','90s',str(binary),'--backend',mode,'--width',str(cw),'--height',str(ch),'--rgb',str(out/'rgb.bin'),'--depth',str(out/'depth.bin'),'--depth-scale',str(scale),'--fx',str(k.fx),'--fy',str(k.fy),'--cx',str(k.ppx),'--cy',str(k.ppy),'--frames',str(frames),'--warmup',str(warmup),'--timestamps',str(timestamps),'--output',str(out/(mode+'.json'))]
  result=subprocess.run(command,capture_output=True,text=True);(out/(mode+'.log')).write_text(result.stdout+result.stderr)
  if result.returncode:raise RuntimeError(f'{label} {mode} failed: '+result.stderr)
  data=json.loads((out/(mode+'.json')).read_text());summary.append({'profile':label,'mode':mode,'result':data});print(json.dumps(summary[-1]),flush=True)
(root/'recorded-core-profile-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
