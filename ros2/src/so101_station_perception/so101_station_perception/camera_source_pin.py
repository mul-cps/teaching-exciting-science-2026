"""Metadata-only camera pins. No robot, world, fitting, images or actuation."""
from collections import deque
from copy import deepcopy
import json
import os
from pathlib import Path
import tempfile

from .derived_lineage import (LineageBuffer, SCHEMA, canonical_gid, finite,
    native_snapshot, require, statement_pin, validate_camera, validate_graph, validate_statement)
from .aligned_cohort import stamp_ns


class CameraPins:
    def __init__(self):
        self.ledger=LineageBuffer();self.infos={kind:deque(maxlen=4) for kind in ('depth','color')}
        self.advancing_pairs=0;self.epoch=object()

    def _open(self):
        if self.ledger.invalid_reason:raise ValueError(self.ledger.invalid_reason)

    def invalidate(self,reason):
        for messages in self.infos.values():messages.clear()
        self.ledger.invalidate(reason)

    def lineage(self,raw,gid,*,now_ns,now_mono):
        self._open()
        if self.ledger.add(raw,gid,now_ns=now_ns,now_mono=now_mono):self.advancing_pairs+=1

    def info(self,kind,message,gid,*,now_ns,now_mono):
        self._open();require(kind in self.infos,'unsupported camera info kind')
        stamp=stamp_ns(message);require(0<=now_ns-stamp<=500_000_000,'CameraInfo future/stale')
        maximum=(1280,720) if kind=='depth' else (1920,1080)
        require(type(message.header.frame_id) is str and 0<len(message.header.frame_id)<=128 and
            all(type(n) is int and 0<n<=limit for n,limit in zip((message.width,message.height),maximum)) and
            len(message.k)==9 and len(message.d)==5 and getattr(message,'distortion_model',None)=='plumb_bob',
            'unsupported bounded native CameraInfo profile')
        try:self.ledger.observe_gid(kind+'_info',gid)
        except ValueError as error:self.invalidate(str(error))
        gid=canonical_gid(gid)
        if any(item[1]==gid and stamp_ns(item[0])==stamp for item in self.infos[kind]):return
        self.infos[kind].append((message,gid,now_mono))

    def select(self,*,now_ns,now_mono):
        self._open();require(self.advancing_pairs>=2,'two advancing original source pairs required')
        require(bool(self.ledger.entries),'fresh per-cohort lineage unavailable')
        value,gid,receipt=self.ledger.entries[-1]
        validate_statement(value,now_ns=now_ns,now_mono=now_mono,receipt_mono=receipt)
        selected={'epoch':self.epoch,'statement':value,'publisher_gid':gid,'received_monotonic':receipt,'infos':{}}
        for kind in ('depth','color'):
            records=[item for item in self.infos[kind] if stamp_ns(item[0])==value['original'][kind+'_info_stamp_ns'] and item[1]==value['source'][kind+'_info_publisher_gid']]
            require(bool(records),'exact native CameraInfo cohort unavailable')
            record=records[-1];require(0<=now_mono-record[2]<=.250,'own CameraInfo receipt lease expired/future')
            selected['infos'][kind]=record
        return selected

    def verify(self,selected,*,lookup,color_from_depth,identity,now_ns,now_mono):
        self._open();require(selected['epoch'] is self.epoch,'selected cohort predates capture')
        value=selected['statement']
        require(statement_pin(value)==self.ledger.pin,'selected source pin changed')
        require(bool(self.ledger.entries) and self.ledger.entries[-1][0]['sequence']==value['sequence'],'newer lineage observed; no old-status fallback')
        self.check_leases(selected,identity,now_ns=now_ns,now_mono=now_mono)
        try:
            snapshot=validate_graph(value,lookup,lineage_gid=selected['publisher_gid'])
            require(snapshot==identity['source_snapshot'] and identity['serial_number']==value['source']['serial_number'],'native generation/serial changed during independent DeviceInfo')
            validate_camera(value,selected['infos']['depth'][0],selected['infos']['color'][0],color_from_depth)
        except ValueError as error:self.invalidate(str(error))
        selected['verified_optical_tf']=[[float(x) for x in row] for row in color_from_depth]
        return snapshot

    def check_leases(self,selected,identity,*,now_ns,now_mono):
        self._open()
        request,confirmed=identity['request_started_monotonic'],identity['confirmed_monotonic']
        require(all(finite(t) for t in (request,confirmed,now_mono)) and 0<=request<=confirmed<=now_mono<request+1. and confirmed-request<=.4,'own DeviceInfo lease expired/invalid')
        require(type(identity['confirmed_ros_ns']) is int and 0<=now_ns-identity['confirmed_ros_ns']<=1_000_000_000,'own DeviceInfo ROS clock future/stale')
        validate_statement(selected['statement'],now_ns=now_ns,now_mono=now_mono,receipt_mono=selected['received_monotonic'])
        for record in selected['infos'].values():require(0<=now_mono-record[2]<=.250,'own CameraInfo receipt lease expired/future')


def artifact(selected,identity,*,source_hashes):
    value=selected['statement']
    info={kind:{'stamp_ns':stamp_ns(record[0]),'frame_id':record[0].header.frame_id,
        'width':record[0].width,'height':record[0].height,'k':[float(x) for x in record[0].k],
        'd':[float(x) for x in record[0].d],'distortion_model':record[0].distortion_model,'publisher_gid':record[1],'received_monotonic':record[2]}
        for kind,record in selected['infos'].items()}
    return {'schema':'so101-camera-source-v1','qualified':False,'execution_enabled':False,
        'metadata_only':True,'independent_pixels_observed':False,
        'derived_source':{'schema':SCHEMA,'qualified':False,'local_provenance_not_cryptographic':True,'pin':statement_pin(value)},
        'camera_evidence':{'lineage':{'statement':deepcopy(value),'received_monotonic':selected['received_monotonic'],'publisher_gid':selected['publisher_gid']},
            'native_info':info,'color_from_depth':selected.get('verified_optical_tf'),
            'independent_device_info':deepcopy(identity),'source_sha256':dict(source_hashes)}}


def publish_exclusive(documents,*,check):
    """Stage complete JSON, recheck leases, publish via exclusive hard links.

    Consumers reload only after successful command completion. On failure remove
    only links created by this call, never preexisting operator files.
    """
    staged=[];published=[]
    try:
        for destination,value in documents.items():
            destination=Path(destination)
            if destination.exists():raise FileExistsError(destination)
            destination.parent.mkdir(parents=True,exist_ok=True)
            raw=(json.dumps(value,sort_keys=True,indent=2,allow_nan=False)+'\n').encode()
            require(len(raw)<=262144,'camera source artifact exceeds bounded JSON size')
            with tempfile.NamedTemporaryFile(dir=destination.parent,prefix='.'+destination.name+'.',delete=False) as stream:
                temporary=Path(stream.name);staged.append((temporary,destination));stream.write(raw);stream.flush();os.fsync(stream.fileno())
        check()
        for temporary,destination in staged:
            os.link(temporary,destination);published.append((temporary,destination))
        check()
    except BaseException:
        for temporary,destination in reversed(published):
            try:
                if os.path.samestat(temporary.stat(),destination.stat()):destination.unlink()
            except FileNotFoundError:pass
        raise
    finally:
        for temporary,_ in staged:temporary.unlink(missing_ok=True)
