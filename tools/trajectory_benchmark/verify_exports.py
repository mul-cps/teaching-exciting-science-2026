#!/usr/bin/env python3
"""Check frozen benchmark exports and the measured A/B sample identity."""
import hashlib
import json
from pathlib import Path
import sys

root=Path(sys.argv[1]);report=json.loads((root/'report.json').read_text())
assert report['status']=='OFFLINE_ONLY_NOT_OWNER_ADMISSIBLE_NOT_COLLISION_APPROVED'
for index,case in enumerate(report['cases']):
    directory=root/f'case-{index:02d}'
    for name,arm in case['arms'].items():
        if 'error' in arm:raise ValueError(f'{name}: {arm["error"]}')
        data=(directory/f'{name[0]}-samples.json').read_bytes()
        assert hashlib.sha256(data).hexdigest()==arm['samples_sha256']
        samples=json.loads(data)
        assert len(samples)==arm['sample_count']
        assert all(abs(sample['time']-i*.02)<1e-10 for i,sample in enumerate(samples))
        assert arm['endpoint_error_linf_rad']<1e-10
    assert case['arms']['A_current_bridge']['samples_sha256']==case['arms']['B_moveit_then_current_bridge']['samples_sha256']
    assert case['arms']['A_current_bridge']['finite_difference_limit_pass']
    assert not case['arms']['C_direct_derivative_hermite']['finite_difference_limit_pass']
print(f'PASS: {len(report["cases"])} frozen paths; exact 50Hz exports, hashes, A/B identity, baseline limits and candidate rejection')
