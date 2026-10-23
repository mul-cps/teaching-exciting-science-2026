"""Host CPU model cost/proof counts; explicitly not GPU latency or production CPU."""
import importlib.util
import json
from pathlib import Path
import time
import numpy as np

spec = importlib.util.spec_from_file_location('tile_model', Path(__file__).with_name('test_resident_vulkan_cc_model.py'))
model = importlib.util.module_from_spec(spec)
spec.loader.exec_module(model)
result = {'scope': 'CPU-only immutable phase model and framework work counts; excludes GPU execution', 'cases': []}
for case in ('flat', 'partial_snake', 'rings'):
    w, h = (65, 49) if case == 'partial_snake' else (128, 96)
    y, x = np.indices((h, w)); colors = np.ones((h, w), np.uint32); candidate = np.ones((h, w), bool)
    if case == 'rings':
        candidate = np.minimum.reduce([x, y, w-1-x, h-1-y]) % 4 == 0
    elif case == 'partial_snake':
        candidate[:] = False
        for row in range(0, h, 2):
            candidate[row] = True
            if row + 1 < h:
                candidate[row+1, w-1 if row//2 % 2 == 0 else 0] = True
    start = time.perf_counter(); actual, fault, rounds = model.tiled_candidate(colors, candidate); ms = (time.perf_counter()-start)*1000
    np.testing.assert_array_equal(actual, model.independent_labels(colors, candidate))
    result['cases'].append({'case': case, 'width': w, 'height': h, 'model_ms': ms, 'tile_rounds_with_confirmation': rounds, 'fault': fault})
w, h = 640, 480
all_edges = cross_edges = lane_visits = 0
for y in range(h):
    for x in range(w):
        selected = not ((x & 15) != 0 and (x & 15) != 15 and (y & 15) != 15)
        lane_visits += selected
        for dy, dx in ((0, 1), (1, -1), (1, 0), (1, 1)):
            nx, ny = x+dx, y+dy
            if 0 <= nx < w and ny < h:
                all_edges += 1
                cross_edges += (nx//16, ny//16) != (x//16, y//16)
result['full640_work_counts'] = {'undirected_8neighbor_edges': all_edges, 'cross_tile_edges': cross_edges, 'selected_boundary_lanes': lane_visits, 'pixels': w*h, 'dispatches': 71, 'barriers': 74, 'tile_shared_bytes': 3*256*4+4, 'max_tile_rounds': 256, 'max_tile_barriers': 1+3*256}
print(json.dumps(result))
