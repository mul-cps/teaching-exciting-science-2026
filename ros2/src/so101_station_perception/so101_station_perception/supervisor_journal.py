"""Cold transition-only quota journal and exclusive supervisor lifetime lock."""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import tempfile
import time
from .derived_lineage import require


class SupervisorJournal:
    def __init__(self,directory,baseline_sha):
        self.directory=Path(directory).resolve()
        require(self.directory.is_dir(),'existing trusted supervisor journal directory required')
        self.path=self.directory/'perception-supervisor.json'
        self.lock=(self.directory/'perception-supervisor.lock').open('a+b')
        try:fcntl.flock(self.lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BaseException:self.lock.close();raise RuntimeError('another automatic perception supervisor owns journal')
        self.baseline_sha=baseline_sha
        try:
            self.boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
            if self.path.exists():
                with self.path.open('rb') as stream:raw=stream.read(65537)
                require(len(raw)<=65536,'supervisor journal exceeds64KiB')
                def pairs(items):
                    result={}
                    for k,v in items:require(k not in result,'duplicate journal key');result[k]=v
                    return result
                self.loaded=json.loads(raw,object_pairs_hook=pairs)
                require(self.loaded.get('baseline_sha256')==baseline_sha,'journal baseline differs; explicit bootstrap required')
                require(self.loaded.get('boot_id')==self.boot,'journal boot changed; explicit bootstrap required')
                history=self.loaded['policy'];now=time.monotonic();utc=time.time()
                require(type(history.get('last_mono')) in (int,float) and now>=history['last_mono'] and utc>=history['last_utc'] and
                    abs((utc-history['last_utc'])-(now-history['last_mono']))<=2.,'persisted wall/monotonic clock discontinuity')
            else:self.loaded={}
        except BaseException:self.close();raise

    def write(self,policy,**context):
        value=dict(schema='so101-perception-supervisor-persistence-v1',baseline_sha256=self.baseline_sha,
                   boot_id=self.boot,policy=policy,**context)
        raw=json.dumps(value,allow_nan=False,sort_keys=True,separators=(',',':')).encode()
        require(len(raw)<=65536,'supervisor journal exceeds64KiB')
        descriptor,path=tempfile.mkstemp(prefix='.supervisor-',dir=self.directory)
        try:
            with os.fdopen(descriptor,'wb') as stream:stream.write(raw);stream.flush();os.fsync(stream.fileno())
            os.replace(path,self.path)
            directory=os.open(self.directory,os.O_RDONLY|os.O_DIRECTORY)
            try:os.fsync(directory)
            finally:os.close(directory)
            require(self.path.read_bytes()==raw,'supervisor journal readback differs')
            return hashlib.sha256(raw).hexdigest()
        finally:
            try:os.unlink(path)
            except FileNotFoundError:pass

    def close(self):
        if not self.lock.closed:self.lock.close()
