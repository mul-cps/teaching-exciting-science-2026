from types import SimpleNamespace
import struct
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2/src/so101_station_map"))

from so101_station_map.surface_map import (SurfaceVoxelMap, cloud_xyz_chunks,
                                            tracking_status_is_fresh)


def cloud(points, *, height, width, row_padding=0, stale_height1_row=False):
    point_step = 12
    row_step = width * point_step + row_padding
    data = bytearray(height * row_step)
    for row in range(height):
        for col in range(width):
            xyz = points[row * width + col]
            struct.pack_into("<fff", data, row * row_step + col * point_step, *xyz)
    if stale_height1_row:
        row_step += 64
    fields = [SimpleNamespace(name=name, offset=offset, datatype=7, count=1)
              for name, offset in zip(("x", "y", "z"), (0, 4, 8))]
    return SimpleNamespace(height=height, width=width, point_step=point_step,
                           row_step=row_step, data=bytes(data), fields=fields,
                           is_bigendian=False)


def test_chunk_decoder_handles_row_padding_and_compact_unordered_cloud():
    organized = cloud([(1, 2, 3), (4, 5, 6)], height=1, width=2)
    assert np.array_equal(np.concatenate(list(cloud_xyz_chunks(organized))),
                          [[1, 2, 3], [4, 5, 6]])
    compact = cloud([(1, 2, 3), (4, 5, 6)], height=1, width=2,
                    stale_height1_row=True)
    assert np.array_equal(np.concatenate(list(cloud_xyz_chunks(compact))),
                          [[1, 2, 3], [4, 5, 6]])

    padded = cloud([(1, 2, 3), (4, 5, 6)], height=2, width=1, row_padding=8)
    chunks = list(cloud_xyz_chunks(padded, max_chunk_points=1))
    assert len(chunks) == 2
    assert np.array_equal(np.concatenate(chunks), [[1, 2, 3], [4, 5, 6]])


def test_large_unordered_cloud_is_decoded_in_point_bounded_blocks():
    count = 640 * 480
    values = np.zeros((count, 3), dtype="<f4")
    values[:, 0] = np.linspace(-0.5, 0.5, count, dtype=np.float32)
    values[:, 2] = 0.7
    msg = SimpleNamespace(
        height=1, width=count, point_step=12, row_step=640 * 12,
        data=values.tobytes(),
        fields=[SimpleNamespace(name=name, offset=offset, datatype=7, count=1)
                for name, offset in zip(("x", "y", "z"), (0, 4, 8))],
        is_bigendian=False)
    block_count = 0
    decoded = 0
    largest = 0
    for block in cloud_xyz_chunks(msg):
        block_count += 1
        decoded += len(block)
        largest = max(largest, len(block))
    assert decoded == count
    assert largest <= 8192
    assert block_count == 38


def test_mapping_requires_recent_tracking_for_the_cloud_timestamp():
    assert tracking_status_is_fresh("tracking", 10.0, 10.2, 10.1, 0.5)
    assert not tracking_status_is_fresh("stale", 10.0, 10.2, 10.1, 0.5)
    assert not tracking_status_is_fresh("tracking", 10.0, 10.6, 10.1, 0.5)
    assert not tracking_status_is_fresh("tracking", 10.0, 10.2, 10.6, 0.5)
    assert not tracking_status_is_fresh("tracking", 10.3, 10.2, 10.3, 0.5)


def test_chunk_decoder_rejects_malformed_layout_and_oversized_cloud():
    malformed = cloud([(0, 0, 1)], height=1, width=1)
    malformed.row_step = 1
    malformed.data += b"\x00"
    with pytest.raises(ValueError, match="Malformed"):
        list(cloud_xyz_chunks(malformed))
    oversized = cloud([(0, 0, 1)], height=1, width=1)
    oversized.height, oversized.width = 481, 640
    with pytest.raises(ValueError, match="bound"):
        list(cloud_xyz_chunks(oversized))


def test_surface_voxels_crop_cap_and_expire_without_free_space_inference():
    voxel_map = SurfaceVoxelMap(resolution=0.01, ttl_s=0.5, max_voxels=2,
                                bounds=((-0.1, 0.1), (-0.1, 0.1), (0, 0.2)))
    points = np.array([
        [0.001, 0.001, 0.101],  # one observed surface cell
        [0.002, 0.001, 0.101],  # same voxel
        [0.051, 0.001, 0.101],
        [-0.051, 0.001, 0.101],
        [0.5, 0, 0.1],  # cropped outside workspace
        [np.nan, 0, 0.1],
    ])
    accepted = voxel_map.integrate(points, now=1.0)
    assert accepted == 4
    assert voxel_map.last_input_points == len(points)
    assert voxel_map.last_accepted_points == 4
    assert len(voxel_map.cells) == 2
    assert np.allclose(voxel_map.centers(), [[0.005, 0.005, 0.105],
                                             [0.055, 0.005, 0.105]])
    assert voxel_map.clear_expired(1.49) == 0
    assert voxel_map.clear_expired(1.51) == 2
    assert voxel_map.centers().shape == (0, 3)
