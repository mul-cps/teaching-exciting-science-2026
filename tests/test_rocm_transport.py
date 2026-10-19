"""CPU-only fixed-slot/ownership/packing gates, never imports a GPU runtime."""
import json
import mmap
import os
import socket
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from workshop.perception.rocm_transport import (
    ExposureLedger, LatestSlots, SharedSlots, SlotLayout, decode_packet,
    encode_packet, pack_cloud,
)
from workshop.perception.rocm_worker import serve


def meta(depth=1_000_000_000, color=1_000_001_000):
    return dict(depth_stamp_ns=depth, color_stamp_ns=color, source_generation='a'*64)


def test_depth_and_rgb_exposures_each_deduplicated_not_receipt_counts():
    ledger = ExposureLedger('a'*64)
    assert ledger.admit(meta(), 1_100_000_000)
    assert not ledger.admit(meta(color=1_010_000_000), 1_100_000_000)
    assert not ledger.admit(meta(depth=1_010_000_000), 1_100_000_000)
    assert ledger.admit(meta(1_030_000_000, 1_030_001_000), 1_100_000_000)


@pytest.mark.parametrize('record,now', [
    (meta(1_000_000_000, 1_050_000_000), 1_100_000_000),
    (meta(), 999_999_999), (meta(), 1_500_001_001),
    (dict(meta(), source_generation='b'*64), 1_100_000_000),
    (dict(meta(), depth_stamp_ns=True), 1_100_000_000),
])
def test_bad_generation_future_stale_or_pair_rejected(record, now):
    with pytest.raises(ValueError): ExposureLedger('a'*64).admit(record, now)


def test_rollback_is_not_a_new_generation_or_silent_replay():
    ledger = ExposureLedger('a'*64); ledger.admit(meta(), 1_100_000_000)
    with pytest.raises(ValueError, match='rollback'):
        ledger.admit(meta(999_999_999, 1_000_000_001), 1_100_000_000)


def test_one_active_one_replaceable_pending_and_sequence_ownership():
    slots = LatestSlots(); first = slots.offer(meta()); active = slots.begin()
    assert active == first
    pending = slots.offer(meta(2_000_000_000, 2_000_000_000))
    latest = slots.offer(meta(3_000_000_000, 3_000_000_000))
    assert latest['slot'] == pending['slot'] != active['slot']
    assert slots.begin() is None
    with pytest.raises(ValueError): slots.complete(dict(active, sequence=active['sequence']+1))
    slots.complete(active); assert slots.begin() == latest
    slots.invalidate(); assert slots.begin() is None and slots.active is None


def test_fixed_shared_slots_copy_ownership_and_exact_packed_cloud():
    layout = SlotLayout((2, 2), (3, 2)); shared = SharedSlots.create(layout)
    try:
        depth = np.array([[1, 2], [0, 4]], np.uint16)
        rgb = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
        shared.write_input(0, depth, rgb); original = depth.copy(); depth[:] = 99
        d, c = shared.input_views(0)
        np.testing.assert_array_equal(d, original); np.testing.assert_array_equal(c, rgb)
        assert not d.flags.writeable and not c.flags.writeable
        xyz = np.arange(12, dtype=np.float32).reshape(2, 2, 3)
        packed = np.array([[0x112233, 0xabcdef], [0, 0x765432]], np.uint32)
        valid = np.array([[1, 1], [0, 1]], np.uint8)
        cloud = pack_cloud(xyz, packed, valid)
        assert len(cloud) == 60
        expected = b''.join(xyz[y, x].astype('<f4').tobytes()+bytes(4)+int(packed[y, x]).to_bytes(4, 'little')
                            for y, x in ((0, 0), (0, 1), (1, 1)))
        assert cloud == expected
        aligned = np.arange(6, dtype=np.uint16).reshape(2, 3)
        shared.write_output(0, aligned, cloud)
        a, b = shared.owned_output(0, len(cloud))
        ros_aligned, ros_cloud = shared.owned_ros_arrays(0, len(cloud))
        shared.write_output(0, np.zeros_like(aligned), bytes(60))
        assert a == aligned.astype('<u2').tobytes() and b == cloud
        assert ros_aligned.tobytes() == a and ros_cloud.tobytes() == b
        assert ros_aligned.typecode == ros_cloud.typecode == 'B'
        with pytest.raises(ValueError): shared.owned_output(0, layout.cloud_capacity+20)
        for count in (True, -1, layout.cloud_capacity+20, 1):
            with pytest.raises(ValueError): shared.owned_ros_arrays(0, count)
    finally: shared.close()


@pytest.mark.parametrize('value', [{'x': float('nan')}, {'x': 'x'*65536}, [], {'x': object()}])
def test_ipc_is_finite_bounded_dict_json_only(value):
    with pytest.raises(ValueError): encode_packet(value)


def test_ipc_strict_json_and_nonfinite_decode():
    assert decode_packet(encode_packet({'kind': 'ready', 'value': 3})) == {'kind': 'ready', 'value': 3}
    for raw in (b'{"x":NaN}', b'{"x":1,"x":2}', b'[]', b'{}'*40000):
        with pytest.raises(ValueError): decode_packet(raw)


def test_layout_cannot_allocate_from_unbounded_dimensions_or_bool():
    for depth in ((True, 2), (1281, 720), (1280, 721), (-1, 2)):
        with pytest.raises(ValueError): SlotLayout(depth, (1920, 1080))
    layout = SlotLayout((1280, 720), (1920, 1080))
    assert layout.input_capacity == 8_064_000
    assert layout.output_capacity == 22_579_200


def configuration():
    return dict(schema='realsense-cpu-projection-oracle-v1', qualified=False,
        align_name='Align', pointcloud_name='Pointcloud', occlusion_option=2., depth_scale_m=.001,
        depth_size=[2, 2], color_size=[3, 2], depth_K=[4., 4., 1., 1.], color_K=[4., 4., 1., 1.],
        rotation_column_major=[1., 0., 0., 0., 1., 0., 0., 0., 1.], translation_m=[.02, 0., 0.])


def test_actual_private_socket_shared_slot_protocol_cpu_factory_only():
    from workshop.perception.rocm_transport import source_hashes
    layout = SlotLayout((2, 2), (3, 2)); shared = SharedSlots.create(layout)
    parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    parent.settimeout(1.)
    calls = []
    class FakeBackend:
        hardware = {'test_only_cpu_factory': True}
        def process(self, depth, rgb):
            calls.append(depth.copy())
            xyz = np.zeros((2, 2, 3), np.float32); xyz[..., 2] = depth.astype(np.float32)*.001
            return SimpleNamespace(aligned_z16=np.ones((2, 3), np.uint16), xyz=xyz,
                rgb_packed=np.full((2, 2), 0x112233, np.uint32), valid=(depth > 0).astype(np.uint8), complete_ms=.1)
    errors = []
    def run():
        try: serve(child, shared, configuration(), lambda c: FakeBackend())
        except Exception as error: errors.append(error)
    thread = threading.Thread(target=run); thread.start()
    try:
        ready = decode_packet(parent.recv(65537))
        assert ready['kind'] == 'ready' and ready['source_sha256'] == source_hashes()
        assert len(calls) == 1 and not calls[0].any()  # startup, never source evidence
        depth = np.array([[100, 200], [0, 400]], np.uint16)
        shared.write_input(1, depth, np.zeros((2, 3, 3), np.uint8))
        parent.send(encode_packet(dict(kind='job', slot=1, sequence=3, metadata=meta())))
        reply = decode_packet(parent.recv(65537))
        assert reply['kind'] == 'result' and reply['metadata'] == meta() and reply['sequence'] == 3
        aligned, cloud = shared.owned_output(1, reply['cloud_bytes'])
        assert reply['cloud_bytes'] == 60 and len(aligned) == 12
        parent.send(encode_packet({'kind': 'stop'})); thread.join(1.)
        assert not errors and not thread.is_alive()
    finally:
        parent.close(); child.close(); shared.close()


def test_static_hot_paths_have_no_gpu_import_filehash_or_per_point_loop():
    import ast
    from pathlib import Path
    root = Path(__file__).resolve().parents[1]
    transport = ast.parse((root/'workshop/perception/rocm_transport.py').read_text())
    pack = next(n for n in ast.walk(transport) if isinstance(n, ast.FunctionDef) and n.name == 'pack_cloud')
    assert not any(isinstance(n, (ast.For, ast.While, ast.ListComp, ast.DictComp, ast.SetComp, ast.GeneratorExp)) for n in ast.walk(pack))
    client = ast.parse((root/'workshop/perception/rocm_client.py').read_text())
    for method in ('offer', 'poll', '_dispatch'):
        body = next(n for n in ast.walk(client) if isinstance(n, ast.FunctionDef) and n.name == method)
        calls = [ast.unparse(n.func) for n in ast.walk(body) if isinstance(n, ast.Call)]
        assert not any(x.endswith(('.wait', '.read_bytes', '.sha256', '.sleep', '.item', '.cpu', '.synchronize')) for x in calls)
    for filename in ('rocm_transport.py', 'rocm_client.py'):
        source = (root/'workshop/perception'/filename).read_text()
        assert 'import torch' not in source and 'import triton' not in source

