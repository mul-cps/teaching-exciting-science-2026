"""Refresh owner snapshots often enough without relaxing cached sample admission."""
import ast
from pathlib import Path
import threading
import pytest
from workshop.blockly_ros import ROSBackend

SOURCE=Path(__file__).resolve().parents[1]/'workshop/blockly_ros.py'


def test_owner_refresh_period_is_50ms_and_single_timer():
    tree=ast.parse(SOURCE.read_text())
    calls=[node for node in ast.walk(tree) if isinstance(node,ast.Call)
           and isinstance(node.func,ast.Attribute) and node.func.attr=='create_timer'
           and len(node.args)>1 and ast.unparse(node.args[1])=='self._refresh_owner']
    assert len(calls)==1
    assert ast.literal_eval(calls[0].args[0])==.05


def owner(now,measurement_age=.2):
    return {'schema':1,'kind':'follower_status','sampled_monotonic':now,
            'fresh':True,'position_age_s':measurement_age,'torque_age_s':measurement_age,
            'arm_id':'fake','calibration_sha256':'a'*64,'owner_started_monotonic':1.,
            'owner_source_sha256':{'owner':'b'*64}}


def test_refresh_period_leaves_measurement_freshness_margin(monkeypatch):
    clock=[100.];monkeypatch.setattr('workshop.blockly_ros.time.monotonic',lambda:clock[0])
    backend=ROSBackend.__new__(ROSBackend);backend.lock=threading.Lock()
    backend.owner_cache=(100.,owner(100.))
    backend.latest={'positions':[0.]*5,'jaw':0.,'stamp':100.,'received':100.}
    backend.trigger=lambda *args,**kwargs:pytest.fail('sample must remain cache-only')
    clock[0]=100.05
    assert backend.sample()['identity']['arm_id']=='fake'
    clock[0]=100.11
    with pytest.raises(RuntimeError,match='owner measurements are stale'):backend.sample()


@pytest.mark.parametrize('field',['position_age_s','torque_age_s'])
def test_measurement_300ms_bound_is_not_relaxed(monkeypatch,field):
    monkeypatch.setattr('workshop.blockly_ros.time.monotonic',lambda:100.)
    status=owner(100.,.01);status[field]=.301
    with pytest.raises(RuntimeError,match='owner measurements are stale'):ROSBackend._identity(status)


def test_snapshot_300ms_bound_is_not_relaxed(monkeypatch):
    monkeypatch.setattr('workshop.blockly_ros.time.monotonic',lambda:100.301)
    with pytest.raises(RuntimeError,match='source snapshot is stale'):ROSBackend._identity(owner(100.,0.))
