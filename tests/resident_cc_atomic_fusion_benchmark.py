"""CPU proof-model cost only; never GPU or production CPU latency evidence."""
import json
import time
from test_resident_cc_atomic_fusion import fused_solve
from test_resident_cc_copyback_model import scene
from resident_cc_copyback_model import graph, solve

result={'scope':'CPU-only graph proof model; not production CPU/GPU latency','cases':[]}
for name in ('flat','holes','checker','snake'):
    colors,active=scene(name)
    forest,edges,boundary=graph(colors,active)
    start=time.perf_counter_ns();old=solve(forest,edges,boundary,copyback=True)
    old_ms=(time.perf_counter_ns()-start)/1e6
    start=time.perf_counter_ns();new=fused_solve(forest,edges,boundary,'shuffled')
    fused_ms=(time.perf_counter_ns()-start)/1e6
    assert old['roots']==new['roots'] and old['fault']==new['fault']==0
    result['cases'].append({'case':name,'pixels':colors.size,'active_pixels':int(active.sum()),'bulk_copy_model_ms':old_ms,'fused_model_ms':fused_ms,'roots_exact':True})
result['full640_command_counts']={'rounds':32,'dispatches':71,'barriers':74,'parent_copy_commands':0,'parent_copy_payload_bytes':0,'removed_parent_copy_payload_bytes':39321600,'removed_parent_copy_read_plus_write_bytes':78643200,'atomic_flatten_attempts_max':32*640*480,'new_device_or_host_buffer_bytes':0,'upload_bytes_unchanged':1536000,'compact_download_bytes_unchanged':163856}
print(json.dumps(result,indent=2))
