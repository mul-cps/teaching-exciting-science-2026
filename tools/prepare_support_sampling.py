"""Immutable offline reference controls for 0 < G <= 4800; no frame fallback."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import struct
import tempfile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CAP, TRIALS, MINIMUM, SEED = 2000, 120, 30, 17


def _integer(value, low, high):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)):
        raise TypeError('expected integer')
    if not low <= value <= high:
        raise ValueError('integer outside admitted reference range')
    return int(value)


def _header(g):
    g = _integer(g, 1, 4800)
    m = min(g, CAP)
    sc, pc = g+2, m+2
    sn, pn = max(g-CAP, 0)*CAP, max(m-29, 0)*360
    po = 64+4*sc
    si = po+4*pc
    pi = si+4*sn
    words = (0x31545053, 1, g, CAP, TRIALS, MINIMUM, sc, pc, sn, pn,
             64, po, si, pi, pi+4*pn, 0)
    if any(not 0 <= x <= 0xffffffff for x in words):
        raise ValueError('u32 size overflow')
    return words


def _offsets(g):
    sel = np.maximum(np.arange(g+2, dtype=np.int64)-2001, 0)*CAP
    prop = np.maximum(np.arange(min(g, CAP)+2, dtype=np.int64)-30, 0)*360
    return sel.astype('<u4'), prop.astype('<u4')


def _sources():
    names = ('ros2/src/so101_station_objects/so101_station_objects/objects.py',
             'workshop/digital_twin/grasp_geometry.py', 'tools/native_support_oracle.py',
             'tools/prepare_support_sampling.py')
    return {n: hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in names}


def build_sampling_artifact(output_path, max_population):
    """Create binary then external manifest; refuse existing outputs."""
    h = _header(max_population)
    path = Path(output_path)
    manifest_path = path.with_suffix(path.suffix+'.json')
    if path.exists() or manifest_path.exists():
        raise FileExistsError(path)
    so, po = _offsets(h[2])
    selection = np.empty(h[8], dtype='<u4')
    proposals = np.empty(h[9], dtype='<u4')
    for p in range(CAP+1, h[2]+1):
        selection[so[p]:so[p+1]] = np.random.default_rng(SEED).choice(p, CAP, replace=False)
    for m in range(MINIMUM, min(h[2], CAP)+1):
        rng = np.random.default_rng(SEED)
        valid = np.arange(m)
        row = proposals[po[m]:po[m+1]].reshape(TRIALS, 3)
        for trial in range(TRIALS):
            row[trial] = rng.choice(valid, 3, replace=False)
    payload = struct.pack('<16I', *h)+so.tobytes()+po.tobytes()+selection.tobytes()+proposals.tobytes()
    manifest = dict(schema='support-sampling-v1', max_population=h[2], cap=CAP,
                    trials=TRIALS, minimum=MINIMUM, seed=SEED,
                    numpy=np.__version__, bit_generator=type(np.random.default_rng(SEED).bit_generator).__name__,
                    sources=_sources(), payload_bytes=4*(h[8]+h[9]),
                    offsets_bytes=4*(h[6]+h[7]), artifact_bytes=h[14],
                    artifact_sha256=hashlib.sha256(payload).hexdigest())
    pending = []
    published = False
    try:
        for destination, content in ((path, payload), (manifest_path, (json.dumps(manifest, indent=2, sort_keys=True)+'\n').encode())):
            with tempfile.NamedTemporaryFile(dir=path.parent, suffix='.pending', delete=False) as stream:
                pending.append(Path(stream.name))
                stream.write(content)
                stream.flush()
                os.fsync(stream.fileno())
        # Manifest is the completion marker, always published last.
        # Same-filesystem hard links publish atomically without replacing peers.
        # The binary link claims this destination pair until completion/cleanup;
        # other builders cannot publish over that claim.
        os.link(pending[0], path)
        published = True
        os.link(pending[1], manifest_path)
    except BaseException:
        if published:
            try:
                # Keep the temporary inode until cleanup can verify ownership.
                # Even an externally replaced destination must be preserved.
                if path.samefile(pending[0]):
                    path.unlink()
            except FileNotFoundError:
                pass
        raise
    finally:
        for temp in pending:
            temp.unlink(missing_ok=True)
    return manifest


def read_sampling_artifact(path):
    """Validate digest and full ABI; immutable bytes own all decoded views."""
    path = Path(path)
    manifest = json.loads(path.with_suffix(path.suffix+'.json').read_text())
    data = path.read_bytes()
    if len(data) < 64:
        raise ValueError('truncated header')
    h = struct.unpack('<16I', data[:64])
    expected = _header(h[2])
    if h != expected or len(data) != h[14]:
        raise ValueError('header/size mismatch')
    if manifest.get('artifact_sha256') != hashlib.sha256(data).hexdigest():
        raise ValueError('artifact digest mismatch')
    required = dict(schema='support-sampling-v1', max_population=h[2], cap=CAP,
                    trials=TRIALS, minimum=MINIMUM, seed=SEED,
                    payload_bytes=4*(h[8]+h[9]), offsets_bytes=4*(h[6]+h[7]),
                    artifact_bytes=h[14])
    if any(type(manifest.get(k)) is not type(v) or manifest[k] != v for k, v in required.items()):
        raise ValueError('manifest configuration mismatch')
    table = dict(max_population=h[2], manifest=manifest)
    for name, count, offset in zip(('selection_offsets', 'proposal_offsets', 'selection', 'proposals'), h[6:10], h[10:14]):
        table[name] = np.frombuffer(data, dtype='<u4', count=count, offset=offset)
    so, po = _offsets(h[2])
    if not np.array_equal(so, table['selection_offsets']) or not np.array_equal(po, table['proposal_offsets']):
        raise ValueError('offsets mismatch')
    for p in range(CAP+1, h[2]+1):
        row = selection_indices(table, p)
        if np.any(row >= p) or len(np.unique(row)) != CAP:
            raise ValueError('invalid selection')
    for m in range(MINIMUM, min(h[2], CAP)+1):
        row = proposal_indices(table, m)
        if np.any(row >= m) or np.any(row[:, 0] == row[:, 1]) or np.any(row[:, 0] == row[:, 2]) or np.any(row[:, 1] == row[:, 2]):
            raise ValueError('invalid triplet')
    return table


def selection_indices(table, population):
    """Offline inspection only; identity rows have no stored indices."""
    p = _integer(population, 0, table['max_population'])
    if p <= CAP:
        return np.frombuffer(np.arange(p, dtype='<u4').tobytes(), dtype='<u4')
    offsets = table['selection_offsets']
    return table['selection'][offsets[p]:offsets[p+1]]


def proposal_indices(table, population):
    """Offline inspection only, preserving trial and within-triplet order."""
    m = _integer(population, 0, min(table['max_population'], CAP))
    offsets = table['proposal_offsets']
    return table['proposals'][offsets[m]:offsets[m+1]].reshape(-1, 3)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--max-population', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(build_sampling_artifact(args.output, args.max_population), sort_keys=True))


if __name__ == '__main__':
    main()
