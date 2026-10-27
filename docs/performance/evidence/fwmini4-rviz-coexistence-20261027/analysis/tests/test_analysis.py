import importlib.util,json,tempfile,unittest,subprocess,sys
from pathlib import Path
spec=importlib.util.spec_from_file_location('analysis','/tmp/rviz-coexistence-analysis-20261027.py');a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)


def runner(profile='vga',mode='vulkan'):
    return dict(logical_cpus=4,passed=True,profile_requested=profile,mode=mode,profile_matches=True,actual_extents={'vga':[640,480,640,480],'720p':[1280,720,1280,720],'1080p':[1920,1080,1280,720]}[profile])


def collector(begin,backend='vulkan',profile='vga'):
    alignment='raw_vulkan' if backend=='vulkan' else 'vendor_cpu'
    fields={'profile':profile,**dict(zip(('color_width','color_height','depth_width','depth_height'),map(str,runner(profile,backend)['actual_extents']))),'backend':backend,'alignment':alignment,'vulkan_allocation_count':'35' if backend=='vulkan' else '0'}
    measurements=[{'receipt_monotonic':begin+i*.2,'collector_received_age_ms':v,'fields':{'source_frame':str(x),'depth_source_frame':str(x+10)}} for i,(x,v) in enumerate([(10,1),(11,2),(13,3),(14,20)])]
    return dict(errors=[],mode_requested=backend,status_samples=[{'text':'starting','receipt_monotonic':begin-1,'fields':{}},{'text':'experimental; ...','receipt_monotonic':begin,'fields':fields}],measurements=measurements,steady_seconds=2.0,unique_measured_outputs=4,output_hz=2.0,container_cpu_ms_proc_stat=800,container_cpu_ms_per_unique_output=200,collector_received_age_p95_ms=3,container_pid=99,passed=True,execution_eligible=False,full_station_qualified=False)


def meta(drawable,w,h,renderer='Intel GPU'):
    return dict(event='renderer',display=1,drawable=drawable,monotonic_ns=1000000000,vendor='Intel',renderer=renderer,version='4.6',width=w,height=h,direct=True,direct_known=True,metadata_complete=True)


def interval(drawable,start,end,count,w,h):
    return dict(event='interval',display=1,drawable=drawable,start_ns=int(start*1e9),end_ns=int(end*1e9),elapsed_ns=int((end-start)*1e9),interval_swaps=count,total_swaps=100,width=w,height=h)


def sample(t,ticks,host, start=7):
    return dict(monotonic=t,clock_ticks_per_second=100,host_cpu_ticks=host,sampling_cost_seconds=.002,processes={'rviz':{'pid':8,'cpu_ticks':ticks,'starttime_ticks':start,'rss_pages':100,'drm_clients':{'12':{'drm-client-id':'12','drm-driver':'i915','drm-engine-render':str(ticks*100)+' ns'}}}})


class Tests(unittest.TestCase):
    def test_shared_profile_logs_use_distinct_windows_no_sum(self):
        logs=[meta(10,1000,700),meta(20,100,100),interval(10,9.5,10.5,500,1000,700),interval(10,10,11,30,1000,700),interval(10,11,12,20,1000,700),interval(20,10,12,120,100,100),interval(10,20,22,80,1000,700)]
        first=a.glx_summary(logs,10,12);second=a.glx_summary(logs,20,22)
        self.assertEqual(first['primary_viewport']['weighted_swap_hz'],25)
        self.assertEqual(first['primary_viewport']['covered_seconds'],2)
        self.assertEqual(first['primary_viewport']['interval_count'],2)
        self.assertEqual(first['viewports'][1]['weighted_swap_hz'],60)
        self.assertEqual(second['primary_viewport']['weighted_swap_hz'],40)
        self.assertEqual(first['primary_viewport']['min_interval_swap_hz'],20)
        self.assertEqual(first['primary_viewport']['max_interval_swap_hz'],30)
    def test_cpu_counter_math_and_actual_window(self):
        rows=[sample(9.95,100,[100,0,0,300,10,0,0,0]),sample(12.05,142,[200,0,0,500,50,0,0,0])]
        v=a.observer_summary(rows,10,12,99)
        self.assertAlmostEqual(v['window']['begin_offset_seconds'],-.05)
        self.assertAlmostEqual(v['window']['end_offset_seconds'],.05)
        self.assertAlmostEqual(v['processes']['rviz']['CPU_ms'],420)
        self.assertAlmostEqual(v['processes']['rviz']['total_four_core_CPU_percent'],5)
        self.assertEqual(v['processes']['rviz']['RSS_end_pages'],100)
        self.assertEqual(v['processes']['rviz']['DRM_clients']['12']['engine_delta_ns']['render'],4200)
        self.assertAlmostEqual(v['host_busy_percent'],100*100/340)
        self.assertFalse(v['observer_process_CPU_included_in_target_sum'])
        self.assertEqual(v['sampling_cost_seconds']['max'],.002)
    def test_pid_change_and_counter_regression_are_not_cpu_zero(self):
        rows=[sample(10,100,[100,0,0,300,0,0,0,0]),sample(12,99,[200,0,0,500,0,0,0,0],start=9)]
        v=a.observer_summary(rows,10,12,99)
        self.assertTrue(v['errors']);self.assertIsNone(v['processes']['rviz']['CPU_ms'])
        self.assertIsNone(v['external_targets_total_four_core_CPU_percent'])
    def test_empty_and_software_glx_are_errors_not_zero(self):
        self.assertIsNone(a.glx_summary([],10,12)['primary_viewport'])
        self.assertTrue(a.glx_summary([],10,12)['errors'])
        rows=[meta(1,100,100,'llvmpipe (LLVM)'),interval(1,10,11,30,100,100)]
        self.assertTrue(a.glx_summary(rows,10,12)['errors'])
        rows[0]['metadata_complete']=False
        self.assertIn('incomplete',str(a.glx_summary(rows,10,12)['errors']))
    def test_collector_math_and_expected_first_status(self):
        d=collector(10);v=a.perception_summary(d,runner(),'vulkan','vga')
        self.assertEqual(v['window']['begin_monotonic_approx'],10)
        self.assertEqual(v['metrics']['unique_output_hz'],2)
        self.assertEqual(v['metrics']['total_four_core_CPU_percent'],10)
        self.assertEqual(v['metrics']['CPU_ms_per_output'],200)
        self.assertEqual(v['metrics']['receipt_age_p95_ms'],3)
        self.assertEqual(v['source_ID_windows']['source_frame']['missing_between_first_last'],1)
        self.assertTrue(v['allocation_safety']['stable'])
        d['status_samples'][1]['fields']['alignment']='vendor_cpu'
        self.assertTrue(a.perception_summary(d,{},'vulkan','vga')['errors'])
    def test_cli_shared_logs_six_runs(self):
        with tempfile.TemporaryDirectory(dir='/tmp/rviz-analysis-tests-20261027') as tmp:
            root=Path(tmp);(root/'logs').mkdir()
            for profile in ('vga','720p','1080p'):
                for lane,backend,begin in [('vulkan','vulkan',10),('system-cpu','cpu',20)]:
                    p=root/lane/'live'/profile;p.mkdir(parents=True)
                    (p/(backend+'.json')).write_text(json.dumps(collector(begin,backend,profile)))
                    (p/(backend+'.runner.json')).write_text(json.dumps(runner(profile,backend)))
                logs=[meta(10,1000,700),interval(10,10,11,30,1000,700),interval(10,20,22,80,1000,700)]
                (root/'logs'/('rviz-'+profile+'.log')).write_text('RViz ordinary log\n'+'\n'.join(json.dumps(x) for x in logs))
                rows=[sample(t,n,[n,0,0,n*3,0,0,0,0]) for t,n in [(10,100),(12,140),(20,200),(22,240)]]
                (root/'logs'/('processes-'+profile+'.jsonl')).write_text('\n'.join(json.dumps(x) for x in rows))
            v=a.analyze(root);self.assertEqual(len(v['runs']),6)
            self.assertFalse(v['qualified']);self.assertTrue(v['RViz_uses_actual_camera_RGB'])
            self.assertFalse(v['matched_camera_pixels_between_modes'])
            self.assertIn('RViz actually displays camera RGB; robot poses/world cloud/brick markers are synthetic',v['limits'])
            self.assertEqual(v['runs']['vga/vulkan']['GLX']['primary_viewport']['weighted_swap_hz'],30)
            self.assertEqual(v['runs']['vga/cpu']['GLX']['primary_viewport']['weighted_swap_hz'],40)
            self.assertTrue(v['runs']['vga/vulkan']['approximate_combined_CPU']['different_accounting_windows'])
            output=root/'analysis.json'
            command=[sys.executable,'/tmp/rviz-coexistence-analysis-20261027.py','--base',str(root),'--output',str(output)]
            subprocess.run(command,check=True)
            self.assertTrue(json.loads(output.read_text())['analysis_valid'])
            (root/'system-cpu/live/vga/cpu.json').unlink()
            p=subprocess.run(command,capture_output=True,text=True)
            self.assertEqual(p.returncode,1)
            self.assertEqual(len(json.loads(output.read_text())['runs']),6)

class ErrorTests(unittest.TestCase):
    def test_profile_mode_and_extents_reject_mislabeled_inputs(self):
        for key,value in [('profile_requested','720p'),('mode','cpu'),('profile_matches',False),('profile_matches',1),('actual_extents',[1280,720,1280,720])]:
            with self.subTest(key=key,value=value):
                r=runner();r[key]=value
                self.assertTrue(a.perception_summary(collector(10),r,'vulkan','vga')['errors'])
    def test_collector_errors_and_mode_are_not_safety_pass(self):
        for key,value in [('errors',['hidden failure']),('mode_requested','cpu')]:
            with self.subTest(key=key):
                d=collector(10);d[key]=value
                self.assertTrue(a.perception_summary(d,runner(),'vulkan','vga')['errors'])
    def test_measurement_receipts_must_be_inside_exact_inferred_bounds(self):
        for t in [9.999999,12.000001]:
            with self.subTest(t=t):
                d=collector(10);d['measurements'][0]['receipt_monotonic']=t
                self.assertTrue(a.perception_summary(d,runner(),'vulkan','vga')['errors'])
        d=collector(10);d['measurements'][0]['receipt_monotonic']=10;d['measurements'][-1]['receipt_monotonic']=12
        self.assertFalse(a.perception_summary(d,runner(),'vulkan','vga')['errors'])
    def test_status_profile_and_extent_changes_are_rejected(self):
        for key,value in [('profile','720p'),('color_width','1280')]:
            with self.subTest(key=key):
                d=collector(10);d['status_samples'][1]['fields'][key]=value
                self.assertTrue(a.perception_summary(d,runner(),'vulkan','vga')['errors'])
    def test_interval_zero_elapsed_and_overlap_rejected(self):
        rows=[meta(1,100,100),interval(1,10,11,0,100,100)]
        self.assertTrue(a.glx_summary(rows,10,12)['errors'])
        rows=[meta(1,100,100),interval(1,10,11.5,30,100,100),interval(1,11,12,30,100,100)]
        self.assertIn('overlapping',str(a.glx_summary(rows,10,12)['errors']))
    def test_missing_metadata_not_borrowed_from_other_drawable(self):
        rows=[meta(1,100,100),interval(2,10,11,30,200,200)]
        v=a.glx_summary(rows,10,12)
        self.assertIsNone(v['primary_viewport']['renderer_record'])
        self.assertTrue(v['errors'])
    def test_missing_process_row_invalidates_sum(self):
        rows=[sample(10,100,[100,0,0,300,0,0,0,0]),sample(11,110,[110,0,0,310,0,0,0,0]),sample(12,120,[120,0,0,320,0,0,0,0])]
        rows[1]['processes']['rviz']={'pid':8,'missing':True}
        self.assertIsNone(a.observer_summary(rows,10,12,99)['external_targets_total_four_core_CPU_percent'])
    def test_camera_pid_and_alias_not_double_counted(self):
        rows=[sample(10,100,[100,0,0,300,0,0,0,0]),sample(12,140,[200,0,0,500,0,0,0,0])]
        for row in rows:row['processes']['alias']=dict(row['processes']['rviz'])
        v=a.observer_summary(rows,10,12,99)
        self.assertAlmostEqual(v['external_targets_total_four_core_CPU_percent'],5)
        self.assertEqual(a.observer_summary(rows,10,12,8)['external_targets_total_four_core_CPU_percent'],0)
    def test_no_cpu_window_is_invalid_not_zero(self):
        v=a.observer_summary([sample(1,1,[1]*8)],10,12,99)
        self.assertTrue(v['errors']);self.assertIsNone(v['external_targets_total_four_core_CPU_percent'])
    def test_allocation_growth_and_summary_mismatch_are_errors(self):
        d=collector(10)
        d['status_samples'].append({'receipt_monotonic':11,'text':'experimental; ...','fields':{'backend':'vulkan','alignment':'raw_vulkan','vulkan_allocation_count':'36'}})
        d['output_hz']=3
        v=a.perception_summary(d,runner(),'vulkan','vga')
        self.assertFalse(v['allocation_safety']['stable']);self.assertGreaterEqual(len(v['errors']),2)
    def test_engine_client_disappearance_not_zero_utilization(self):
        first=sample(10,100,[1]*8)['processes']['rviz'];last=sample(12,140,[2]*8)['processes']['rviz'];last['drm_clients']={}
        result,warnings=a.engines(first,last)
        self.assertEqual(result,{});self.assertTrue(warnings)

if __name__=='__main__':unittest.main()
