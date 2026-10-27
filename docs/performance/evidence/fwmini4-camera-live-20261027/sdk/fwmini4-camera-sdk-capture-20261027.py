from pathlib import Path
import pyrealsense2 as rs,time,json,statistics,sys
out=Path('/tmp/fwmini4-camera-sdk-640x480-20261027.json');assert not out.exists()
ctx=rs.context();devices=[d for d in ctx.query_devices() if '435' in d.get_info(rs.camera_info.name)];assert len(devices)==1
serial=devices[0].get_info(rs.camera_info.serial_number)
cfg=rs.config();cfg.enable_device(serial);cfg.enable_stream(rs.stream.color,640,480,rs.format.rgb8,30);cfg.enable_stream(rs.stream.depth,640,480,rs.format.z16,30)
pipe=rs.pipeline(ctx);started=False
try:
 profile=pipe.start(cfg);started=True
 for _ in range(30):pipe.wait_for_frames(5000)
 rows=[];cpu0=time.process_time();start=time.monotonic();deadline=start+10
 while time.monotonic()<deadline:
  frames=pipe.wait_for_frames(5000);receipt=time.monotonic();color=frames.get_color_frame();depth=frames.get_depth_frame()
  if color and depth:rows.append((color.get_frame_number(),depth.get_frame_number(),receipt,color.get_timestamp(),depth.get_timestamp()))
 elapsed=time.monotonic()-start;cpu=time.process_time()-cpu0
 unique=len(set((a,b) for a,b,*_ in rows));color_unique=len(set(x[0] for x in rows));depth_unique=len(set(x[1] for x in rows))
 def percentile(values,p):
  s=sorted(values);x=(len(s)-1)*p;lo=int(x);hi=min(lo+1,len(s)-1);return s[lo]+(s[hi]-s[lo])*(x-lo)
 intervals=[1000*(b[2]-a[2]) for a,b in zip(rows,rows[1:])]
 result={'scope':'SDK acquisition only; excludes alignment, Core, geometry, ROS and control','physical_camera':True,'width':640,'height':480,'requested_fps':30,'usb':devices[0].get_info(rs.camera_info.usb_type_descriptor),'observed_pairs':len(rows),'unique_pairs':unique,'unique_color':color_unique,'unique_depth':depth_unique,'elapsed_s':elapsed,'unique_pairs_per_s':unique/elapsed,'process_cpu_s':cpu,'cpu_ms_per_unique_pair':1000*cpu/unique,'host_interval_ms_p50':statistics.median(intervals),'host_interval_ms_p95':percentile(intervals,.95),'first_pair':rows[0][:2],'last_pair':rows[-1][:2],'depth_scale_m':profile.get_device().first_depth_sensor().get_depth_scale(),'full_station_qualified':False}
 out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2),flush=True)
finally:
 if started:pipe.stop()
