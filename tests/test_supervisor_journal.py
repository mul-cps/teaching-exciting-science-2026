from pathlib import Path
import json
import sys
import time
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros2/src/so101_station_perception'))
from so101_station_perception.supervisor_journal import SupervisorJournal
from so101_station_perception.supervisor_policy import SupervisorPolicy


def test_exclusive_journal_preserves_caps_across_restart_and_no_budget_rewind(tmp_path):
    journal=SupervisorJournal(tmp_path,'a'*64);p=SupervisorPolicy();p.clocks(time.monotonic(),time.time())
    p.attempts.append(p.last_utc);journal.write(p.journal());journal.close()
    second=SupervisorJournal(tmp_path,'a'*64)
    try:
        restored=SupervisorPolicy(second.loaded['policy']);restored.clocks(time.monotonic(),time.time())
        assert len(restored.attempts)==1
        with pytest.raises(RuntimeError):SupervisorJournal(tmp_path,'a'*64)
        restored.reset_fault(time.monotonic(),terminal_proven=True)
        assert len(restored.attempts)==1
    finally:second.close()


def test_unknown_inflight_is_durable_before_rpc_and_restarts_latched(tmp_path):
    journal=SupervisorJournal(tmp_path,'a'*64);p=SupervisorPolicy();now=time.monotonic();utc=time.time();p.clocks(now,utc)
    p.enabled=True;p.state='verifying_native';p.fault={'fault_id':'b'*32};p.reserve(now,utc)
    journal.write(p.journal());journal.close();second=SupervisorJournal(tmp_path,'a'*64)
    try:
        restored=SupervisorPolicy(second.loaded['policy']);assert restored.job and restored.latched and len(restored.attempts)==1
    finally:second.close()


@pytest.mark.parametrize('field,value',[('boot_id','changed'),('baseline_sha256','c'*64)])
def test_changed_boot_or_baseline_requires_explicit_bootstrap(tmp_path,field,value):
    journal=SupervisorJournal(tmp_path,'a'*64);p=SupervisorPolicy();p.clocks(time.monotonic(),time.time());journal.write(p.journal());journal.close()
    path=tmp_path/'perception-supervisor.json';v=json.loads(path.read_text());v[field]=value;path.write_text(json.dumps(v))
    with pytest.raises(ValueError):SupervisorJournal(tmp_path,'a'*64)


def test_future_journal_clock_refuses_restart(tmp_path):
    journal=SupervisorJournal(tmp_path,'a'*64);p=SupervisorPolicy();p.clocks(time.monotonic(),time.time());journal.write(p.journal());journal.close()
    path=tmp_path/'perception-supervisor.json';v=json.loads(path.read_text());v['policy']['last_utc']+=10.;path.write_text(json.dumps(v))
    with pytest.raises(ValueError):SupervisorJournal(tmp_path,'a'*64)
