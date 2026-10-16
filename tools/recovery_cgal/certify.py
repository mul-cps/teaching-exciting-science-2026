#!/usr/bin/env python3
"""Bounded offline exact-mesh evidence. Never a production motion permission."""
from __future__ import annotations
import argparse
import contextlib
import ctypes
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import resource
import selectors
import signal
import stat
import struct
import subprocess
import time

import numpy as np

REVIEWED_SOURCE_SHA256 = '2ba8cc76e23ae144da2ec1a0a6224fadcd8ca24c1400de2a40ffbf7c09adfef4'
CGAL_VERSION = '6.2.1'
MAX_FACES, MAX_CELLS = 250000, 100000
MAX_MESH_BYTES, MAX_CELLS_BYTES = 84+50*MAX_FACES, 16*1024**2
MAX_BINARY_BYTES, MAX_PAYLOAD_BYTES = 16*1024**2, 80*1024**2
MAX_STDOUT_BYTES, MAX_STDERR_BYTES = 12*1024**2, 65536


class Rejected(Exception):
    pass


def sha(data):
    return hashlib.sha256(data).hexdigest()


def check_deadline(deadline):
    if time.monotonic() >= deadline:
        raise Rejected('operation_timeout')


def bounded_read(path, maximum, deadline):
    """Regular local files only; check elapsed time after every bounded read.

    OS/filesystem calls cannot be forcibly preempted in-process. They never
    produce a positive result after expiry; no actuator loop may call this.
    """
    check_deadline(deadline)
    descriptor=os.open(path,os.O_RDONLY|os.O_NONBLOCK)
    with os.fdopen(descriptor, 'rb', buffering=0) as source:
        info=os.fstat(source.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_size > maximum:
            raise Rejected('input_byte_limit_or_nonregular_file')
        data=bytearray()
        while True:
            part=source.read(min(65536,maximum+1-len(data)))
            check_deadline(deadline)
            if not part:
                return bytes(data)
            data.extend(part)
            if len(data)>maximum:
                raise Rejected('input_byte_limit_or_nonregular_file')


@contextlib.contextmanager
def sealed_snapshot(data, name):
    """Linux immutable bytes, including on conda Python missing memfd APIs.

    Constants are Linux UAPI: MFD_CLOEXEC|ALLOW_SEALING|EXEC=19;
    F_ADD_SEALS=1033, F_GET_SEALS=1034, WRITE|GROW|SHRINK|SEAL|EXEC=47.
    Unsupported platforms/kernels fail closed. No private mutable pathname.
    """
    libc=ctypes.CDLL(None,use_errno=True)
    create=libc.memfd_create
    create.argtypes=(ctypes.c_char_p,ctypes.c_uint)
    create.restype=ctypes.c_int
    fd=create(name.encode('ascii'),19)
    if fd<0:
        raise Rejected('immutable_snapshot_unavailable')
    try:
        view=memoryview(data)
        while view:
            written=os.write(fd,view[:65536])
            view=view[written:]
        os.fchmod(fd,0o500)
        fcntl.fcntl(fd,getattr(fcntl,'F_ADD_SEALS',1033),47)
        if fcntl.fcntl(fd,getattr(fcntl,'F_GET_SEALS',1034)) & 47 != 47:
            raise Rejected('immutable_snapshot_unavailable')
        os.lseek(fd,0,os.SEEK_SET)
        yield fd
    finally:
        os.close(fd)


def finite(value, *, positive=False):
    if isinstance(value,(bool,np.bool_)) or not isinstance(value,(int,float,np.integer,np.floating)):
        raise ValueError('finite numeric value required')
    try:
        value=float(value)
    except (OverflowError,ValueError) as error:
        raise ValueError('finite numeric value required') from error
    if not math.isfinite(value) or (positive and value<=0):
        raise ValueError('finite numeric value required')
    return value


def unknown(reason, count):
    return {'status':'unknown','reason':reason,
            'cells':[{'certified':False,'reason':reason} for _ in range(count)],
            'production_recovery_qualified':False,'hardware_access':False}


def trusted_backend(binary, manifest, expected_sha256, deadline):
    if (not isinstance(expected_sha256,str) or len(expected_sha256)!=64 or
            any(c not in '0123456789abcdef'for c in expected_sha256)):
        raise Rejected('trusted_build_pin_required')
    manifest_bytes=bounded_read(manifest,65536,deadline)
    if sha(manifest_bytes)!=expected_sha256:
        raise Rejected('untrusted_build_manifest')
    try:
        build=json.loads(manifest_bytes)
        if (build['source_sha256']!=REVIEWED_SOURCE_SHA256 or build['cgal_version']!=CGAL_VERSION or
                build['returncode']!=0 or type(build['returncode'])is not int):
            raise Rejected('unreviewed_source_or_version')
        data=bounded_read(binary,MAX_BINARY_BYTES,deadline)
        if sha(data)!=build['binary_sha256'] or not data.startswith(b'\x7fELF'):
            raise Rejected('untrusted_backend_bytes')
    except (ValueError,KeyError,TypeError,RecursionError) as error:
        raise Rejected('invalid_build_manifest')from error
    return data,sha(manifest_bytes)


def bounded_child(fd,payload,deadline,memory_mb):
    check_deadline(deadline)
    def limits():
        resource.setrlimit(resource.RLIMIT_AS,(memory_mb*1024**2,)*2)
        cpu=max(1,math.ceil(deadline-time.monotonic()))
        resource.setrlimit(resource.RLIMIT_CPU,(cpu,cpu))
        resource.setrlimit(resource.RLIMIT_CORE,(0,0))
        os.nice(10)
    child=subprocess.Popen([f'/proc/self/fd/{fd}'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,
                           stderr=subprocess.PIPE,pass_fds=(fd,),start_new_session=True,
                           preexec_fn=limits,env={'PATH':'/usr/bin:/bin','LANG':'C','LC_ALL':'C'})
    output,errors=bytearray(),bytearray()
    offset=0
    try:
        with selectors.DefaultSelector()as selected:
            for stream,event,name in ((child.stdin,selectors.EVENT_WRITE,'stdin'),
                                      (child.stdout,selectors.EVENT_READ,'stdout'),
                                      (child.stderr,selectors.EVENT_READ,'stderr')):
                os.set_blocking(stream.fileno(),False)
                selected.register(stream,event,name)
            while selected.get_map():
                check_deadline(deadline)
                for key,_ in selected.select(min(.05,max(0,deadline-time.monotonic()))):
                    if key.data=='stdin':
                        try:
                            offset+=os.write(key.fd,payload[offset:offset+65536])
                        except BrokenPipeError:
                            offset=len(payload)
                        if offset==len(payload):
                            selected.unregister(key.fileobj); key.fileobj.close()
                    else:
                        data=os.read(key.fd,65536)
                        if not data:
                            selected.unregister(key.fileobj)
                            continue
                        target,maximum=(output,MAX_STDOUT_BYTES)if key.data=='stdout'else(errors,MAX_STDERR_BYTES)
                        if len(target)+len(data)>maximum:
                            raise Rejected('backend_output_limit')
                        target.extend(data)
            child.wait(timeout=max(.001,deadline-time.monotonic()))
        check_deadline(deadline)
        if child.returncode!=0 or errors:
            raise Rejected('backend_failed_or_stderr')
        return bytes(output)
    finally:
        # Also kill descendants which kept pipe FDs open after the leader exited.
        try:
            os.killpg(child.pid,signal.SIGKILL)
        except ProcessLookupError:
            pass
        child.wait()
        for stream in (child.stdin,child.stdout,child.stderr):
            stream.close()


def validate_response(data, count, face_count):
    try:
        result=json.loads(data)
        status=result['status']
        required={'status','reason','components','cgal_version','cells'}
        if status=='proved':
            required.add('backend_elapsed_s')
        if (set(result)!=required or result['cgal_version']!=CGAL_VERSION or
                type(result['components'])is not int or not 0<=result['components']<=face_count or
                not isinstance(result['cells'],list) or len(result['cells'])!=count):
            raise ValueError('schema')
        if status=='proved':
            if result['reason']!='exact_mesh_and_cell_checks' or result['components']<1 or finite(result['backend_elapsed_s'])<0:
                raise ValueError('proof')
            reasons={'strict_full_cell_interior','boundary_clearance_unproven','not_material_interior'}
        elif status=='rejected':
            if result['reason']not in {'invalid_manifold_or_winding','degenerate_triangle','invalid_or_open_mesh',
                    'self_intersection','zero_volume_component','invalid_volume_orientation'}:
                raise ValueError('rejection')
            reasons={'mesh_not_proven'}
        elif status=='unknown'and result['reason']=='invalid_input_or_backend_exception':
            reasons={'mesh_not_proven'}
        else:
            raise ValueError('status')
        for cell in result['cells']:
            if (set(cell)!={'certified','reason'} or type(cell['certified'])is not bool or
                    cell['reason']not in reasons or
                    cell['certified']!=(status=='proved'and cell['reason']=='strict_full_cell_interior')):
                raise ValueError('cell')
        return result
    except (ValueError,TypeError,KeyError,OverflowError,RecursionError)as error:
        raise Rejected('invalid_backend_response')from error


def certify_triangles(triangles,cells,*,binary,build_manifest=None,trusted_build_sha256=None,
                      timeout_s=30.,memory_mb=4096,scale=(1.,1.,1.),_started=None):
    started=time.monotonic()if _started is None else _started
    timeout_s=finite(timeout_s,positive=True)
    if timeout_s>120:
        raise ValueError('maximum operation timeout120 seconds')
    deadline=started+timeout_s
    if type(memory_mb)is not int or not 1<=memory_mb<=8192:
        raise ValueError('integer memory_mb1..8192 required')
    if not isinstance(triangles,np.ndarray) or triangles.dtype not in (np.dtype('float32'),np.dtype('float64')):
        raise ValueError('float32/float64 triangle array required')
    if triangles.ndim!=3 or triangles.shape[1:]!=(3,3) or not 0<len(triangles)<=MAX_FACES:
        raise ValueError('bounded nonempty Nx3x3 triangles required')
    if not isinstance(cells,(list,tuple))or len(cells)>MAX_CELLS:
        raise ValueError('bounded cell list required')
    if len(scale)!=3:
        raise ValueError('XYZ scale required')
    scale=tuple(finite(v,positive=True)for v in scale)
    rows=[]
    for c in cells:
        if (not isinstance(c,dict)or set(c)!={'center','pitch','boundary_uncertainty'}or
                not isinstance(c['center'],(list,tuple,np.ndarray))or len(c['center'])!=3):
            raise ValueError('exact cell schema required')
        center=[finite(v)for v in c['center']]
        pitch,uncertainty=finite(c['pitch'],positive=True),finite(c['boundary_uncertainty'])
        if uncertainty<0:
            raise ValueError('nonnegative uncertainty required')
        rows.append([*center,pitch,uncertainty])
    provenance={'scale_hex':[v.hex()for v in scale], 'coordinate_units':'scaled source coordinates; physical accuracy unqualified',
                'timeout_s':timeout_s,'memory_mb':memory_mb}
    try:
        check_deadline(deadline)
        triangles=np.array(triangles,dtype=np.float64,copy=True)
        if not np.isfinite(triangles).all():
            raise ValueError('finite triangles required')
        lines=bytearray()
        def append(line):
            check_deadline(deadline)
            data=(line+'\n').encode('ascii')
            if len(lines)+len(data)>MAX_PAYLOAD_BYTES:
                raise Rejected('payload_byte_limit')
            lines.extend(data)
        append('recovery-cgal-v1');append(f'{len(triangles)} {len(rows)}')
        append(' '.join(v.hex()for v in scale))
        for triangle in triangles:
            append(' '.join(float(v).hex()for v in triangle.ravel()))
        for row in rows:
            append(' '.join(float(v).hex()for v in row))
        payload=bytes(lines)
        provenance.update(payload_sha256=sha(payload),triangle_float64_le_sha256=sha(triangles.astype('<f8').tobytes()),triangle_positions_preserved=True)
        binary=Path(binary)
        backend,manifest_sha=trusted_backend(binary,build_manifest or binary.with_name('build.json'),trusted_build_sha256,deadline)
        provenance.update(backend_sha256=sha(backend),trusted_build_sha256=manifest_sha,reviewed_source_sha256=REVIEWED_SOURCE_SHA256,cgal_version=CGAL_VERSION)
        with sealed_snapshot(backend,'recovery-cgal-backend')as fd:
            response=bounded_child(fd,payload,deadline,memory_mb)
        result=validate_response(response,len(rows),len(triangles))
        check_deadline(deadline)
    except Rejected as error:
        result=unknown(str(error),len(rows))
    except (OSError,subprocess.SubprocessError,AttributeError)as error:
        result=unknown('backend_or_snapshot_unavailable',len(rows))
    result.update(provenance=provenance,elapsed_s=time.monotonic()-started,
                  production_recovery_qualified=False,hardware_access=False)
    return result


def certify_file(mesh_path,cells,**kwargs):
    started=time.monotonic()
    timeout=finite(kwargs.get('timeout_s',30.),positive=True)
    try:
        path=Path(mesh_path).resolve()
        source=bounded_read(path,MAX_MESH_BYTES,started+min(timeout,120.))
        if len(source)<84:
            raise Rejected('unsupported_or_invalid_binary_stl')
        count=struct.unpack_from('<I',source,80)[0]
        if not 0<count<=MAX_FACES or len(source)!=84+50*count:
            raise Rejected('unsupported_or_invalid_binary_stl')
        dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])
        triangles=np.frombuffer(source,dtype=dtype,offset=84,count=count)['vertices'].astype(np.float64)
        result=certify_triangles(triangles,cells,_started=started,**kwargs)
        result['provenance'].update(source_path=str(path),source_sha256=sha(source),faces=count,faces_removed_or_repaired=0)
        return result
    except Rejected as error:
        return unknown(str(error),len(cells)if isinstance(cells,(list,tuple))and len(cells)<=MAX_CELLS else 0)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mesh',type=Path,required=True);p.add_argument('--binary',type=Path,required=True)
    p.add_argument('--build-manifest',type=Path)
    p.add_argument('--trusted-build-sha256',required=True)
    p.add_argument('--cells',type=Path);p.add_argument('--scale',type=float,nargs=3,default=(1.,1.,1.))
    p.add_argument('--timeout-s',type=float,default=30.);p.add_argument('--memory-mb',type=int,default=4096)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();started=time.monotonic()
    try:
        timeout=finite(args.timeout_s,positive=True)
        if timeout>120:
            raise ValueError('timeout_limit')
        cells=json.loads(bounded_read(args.cells,MAX_CELLS_BYTES,started+timeout))if args.cells else []
        result=certify_file(args.mesh,cells,binary=args.binary,build_manifest=args.build_manifest,
                trusted_build_sha256=args.trusted_build_sha256,scale=args.scale,
                timeout_s=max(.000001,timeout-(time.monotonic()-started)),memory_mb=args.memory_mb)
        check_deadline(started+timeout)
    except (Rejected,ValueError,OSError,TypeError,KeyError,OverflowError,RecursionError)as error:
        result=unknown(str(error)if isinstance(error,Rejected)else type(error).__name__,0)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items()if k not in {'cells','provenance'}}))
    return 0 if result['status']=='proved'else 2


if __name__=='__main__':
    raise SystemExit(main())
