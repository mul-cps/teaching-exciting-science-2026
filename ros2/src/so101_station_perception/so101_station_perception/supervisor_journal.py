"""Cold transition-only quota journal and exclusive supervisor lifetime lock."""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import tempfile
import time
from .derived_lineage import require,hex_value
from .supervisor_policy import SupervisorPolicy


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
                require(type(self.loaded) is dict and set(self.loaded)=={'schema','baseline_sha256','boot_id','policy','consumer_epochs','adopted'} and
                    self.loaded['schema']=='so101-perception-supervisor-persistence-v1','incomplete/unknown supervisor journal')
                require(type(self.loaded['consumer_epochs']) is dict and
                    set(self.loaded['consumer_epochs']).issubset({'/so101_charuco_world','/so101_colored_objects'}) and
                    all(hex_value(v,32) for v in self.loaded['consumer_epochs'].values()),'invalid persisted consumer epochs')
                adopted=self.loaded['adopted']
                require(adopted is None or type(adopted) is dict and set(adopted)=={'job_id','artifact_sha256','producer_generation'} and
                    hex_value(adopted['job_id'],32) and hex_value(adopted['artifact_sha256'],64) and hex_value(adopted['producer_generation'],32),
                    'invalid persisted attributable artifact')
                require(type(self.loaded['policy']) is dict,'persisted policy object required')
                SupervisorPolicy(self.loaded['policy'])  # validate authority BEFORE any defaulting/clock access
                require(self.loaded.get('baseline_sha256')==baseline_sha,'journal baseline differs; explicit bootstrap required')
                require(self.loaded.get('boot_id')==self.boot,'journal boot changed; explicit bootstrap required')
                history=self.loaded['policy'];now=time.monotonic();utc=time.time()
                require(type(history.get('last_mono')) in (int,float) and now>=history['last_mono'] and utc>=history['last_utc'] and
                    abs((utc-history['last_utc'])-(now-history['last_mono']))<=2.,'persisted wall/monotonic clock discontinuity')
            else:self.loaded={}
        except BaseException:self.close();raise

    def write(self,policy,**context):
        value=dict(schema='so101-perception-supervisor-persistence-v1',baseline_sha256=self.baseline_sha,
                   boot_id=self.boot,policy=policy,consumer_epochs=context.get('consumer_epochs',{}),adopted=context.get('adopted'))
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
