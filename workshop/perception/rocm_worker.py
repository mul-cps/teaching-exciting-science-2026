"""Private fixed-slot worker; direct entry requires explicit GPU enable.

Only this process imports Torch. It has no ROS, SDK camera or control interface.
"""
import argparse
import hashlib
import importlib
import os
from pathlib import Path
import socket
import sys
import time
import types

import numpy as np

if not __package__:
    # Installed pure sources may live under a ROS3.12 site-packages directory.
    # Register ONLY this pinned source directory as a private package; adding
    # its parent to sys.path would expose foreign compiled ROS extensions.
    __package__ = '_so101_private_rocm'
    package = types.ModuleType(__package__)
    package.__package__ = __package__
    package.__path__ = [str(Path(__file__).resolve().parent)]
    sys.modules[__package__] = package
from .rocm_projection import calibration_from_oracle
from .rocm_transport import (MAX_PACKET, SharedSlots, SlotLayout, decode_packet,
                             encode_packet, pack_cloud, source_hashes, WirePacker)


def serve(connection, slots, configuration, backend_factory, wire_packer=None):
    """Internal factory injection is for CPU protocol tests, never a wire field."""
    calibration = calibration_from_oracle(configuration)
    if (calibration.depth_size != slots.layout.depth_size
            or calibration.color_size != slots.layout.color_size):
        raise ValueError('worker admitted calibration differs from fixed slots')
    start = time.monotonic()
    backend = backend_factory(calibration)
    # GPU/JIT setup finishes before any source acquisition is admitted. These
    # zeros are explicitly a startup diagnostic, never source-camera evidence.
    backend.process(np.zeros((calibration.depth_size[1], calibration.depth_size[0]), np.uint16),
                    np.zeros((calibration.color_size[1], calibration.color_size[0], 3), np.uint8))
    connection.sendall(encode_packet(dict(kind='ready', protocol=1, hardware=backend.hardware,
        source_sha256=source_hashes(), startup_ms=(time.monotonic()-start)*1000, qualified=False)))
    previous = 0
    while True:
        raw = connection.recv(MAX_PACKET+1)
        if not raw: return
        job = decode_packet(raw)
        if job == {'kind': 'stop'}: return
        if (set(job) != {'kind', 'slot', 'sequence', 'metadata'} or job['kind'] != 'job'
                or type(job['sequence']) is not int or job['sequence'] <= previous
                or type(job['metadata']) is not dict):
            raise ValueError('invalid monotonic worker job')
        SharedSlots._slot(job['slot']); previous = job['sequence']
        started = time.monotonic()
        depth, color = slots.input_views(job['slot'])
        try:
            result = backend.process(depth, color)
        finally:
            del depth, color
        if wire_packer is None:
            # CPU protocol tests only. Production main always supplies the
            # separately qualified native packer; there is no runtime fallback.
            cloud = pack_cloud(result.xyz, result.rgb_packed, result.valid)
            slots.write_output(job['slot'], result.aligned_z16, cloud); cloud_bytes = len(cloud)
        else:
            slots.write_aligned(job['slot'], result.aligned_z16)
            cloud_bytes = wire_packer.pack_into(result.xyz, result.rgb_packed, result.valid,
                                               slots.cloud_output_view(job['slot']))
        connection.sendall(encode_packet(dict(kind='result', slot=job['slot'], sequence=job['sequence'],
            metadata=job['metadata'], cloud_bytes=cloud_bytes, backend_ms=result.complete_ms,
            worker_ms=(time.monotonic()-started)*1000, qualified=False)))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--enabled', action='store_true')
    parser.add_argument('--socket-fd', type=int, required=True)
    parser.add_argument('--input-fd', type=int, required=True)
    parser.add_argument('--output-fd', type=int, required=True)
    parser.add_argument('--compatibility-library', type=Path, required=True)
    parser.add_argument('--compatibility-sha256', required=True)
    args = parser.parse_args(argv)
    if args.enabled is not True: parser.error('private GPU worker requires explicit --enabled')
    connection = socket.socket(fileno=args.socket_fd)
    slots = None
    try:
        configuration = decode_packet(connection.recv(MAX_PACKET+1))
        if (set(configuration) != {'kind', 'protocol', 'calibration'} or configuration['kind'] != 'configure'
                or type(configuration['protocol']) is not int or configuration['protocol'] != 1):
            raise ValueError('invalid worker startup configuration')
        if (not args.compatibility_library.is_file()
                or hashlib.sha256(args.compatibility_library.read_bytes()).hexdigest() != args.compatibility_sha256):
            raise ValueError('compatibility shared-library source pin mismatch')
        admitted = calibration_from_oracle(configuration['calibration'])
        slots = SharedSlots(SlotLayout(admitted.depth_size, admitted.color_size), args.input_fd, args.output_fd)
        wire_packer = WirePacker(args.compatibility_library)  # before any GPU resource
        # Deliberately after --enabled, finite calibration and library hash.
        backend_type = importlib.import_module(__package__+'.rocm_projection_backend').ROCmProjection
        serve(connection, slots, configuration['calibration'],
              lambda c: backend_type(c, args.compatibility_library, enabled=True), wire_packer=wire_packer)
        return 0
    except Exception as error:
        try: connection.sendall(encode_packet(dict(kind='error', error=(type(error).__name__+': '+str(error))[:1024], qualified=False)))
        except (OSError, ValueError): pass
        return 3
    finally:
        if slots is not None: slots.close()
        connection.close()


if __name__ == '__main__': raise SystemExit(main())
