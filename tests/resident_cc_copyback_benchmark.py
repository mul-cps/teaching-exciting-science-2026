"""CPU-only paired forest model; numbers cannot select a GPU backend."""
import json
import time
import numpy as np
from resident_cc_copyback_model import graph, solve, reference
from test_resident_cc_copyback_model import scene

result = {'scope': 'CPU graph/forest model only; excludes GPU/production CPU latency', 'cases': []}
for name in ('flat', 'snake', 'checker', 'rings', 'split'):
    colors, active = scene(name, 128, 96)
    forest, edges, boundary = graph(colors, active)
    expected = reference(colors, active)
    case = {'case': name, 'width': 128, 'height': 96, 'active_pixels': int(np.count_nonzero(active))}
    for copyback in (False, True):
        start = time.perf_counter()
        actual = solve(forest, edges, boundary, copyback=copyback)
        ms = (time.perf_counter()-start)*1000
        assert actual['fault'] == 0 and actual['roots'] == expected
        case['copyback' if copyback else 'original'] = {key: value for key, value in actual.items() if key != 'roots'} | {'model_ms': ms}
    result['cases'].append(case)
count = 640*480
result['full640_work_counts'] = {'pixels': count, 'rounds': 32, 'dispatches': 71, 'barriers_retaining_existing': 138, 'copy_commands': 32, 'copy_payload_bytes': 32*count*4, 'copy_read_plus_write_bytes': 2*32*count*4, 'new_device_or_host_buffer_bytes': 0, 'upload_bytes_unchanged': 1536000, 'compact_download_bytes_unchanged': 163856}
print(json.dumps(result))
