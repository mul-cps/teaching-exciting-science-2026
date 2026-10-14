"""Guards against accidentally rebuilding a CAD solver in the warm path."""
import ast
import inspect
import textwrap

import numpy as np
import pytest

from workshop.digital_twin.ik import SO101IK


def test_warm_methods_do_not_build_models():
    for name in ("forward_raw", "solve_pinch", "forward_provenance"):
        source = inspect.getsource(getattr(SO101IK, name))
        tree = ast.parse(textwrap.dedent(source))
        calls = [n.func.attr for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)]
        assert not {"parse", "TemporaryDirectory", "write", "read_bytes"}.intersection(calls)


def test_warm_cache_never_invokes_factory_or_provider_again(monkeypatch):
    pytest.importorskip("placo")
    pytest.importorskip("lerobot.model.kinematics")
    from lerobot.model.kinematics import RobotKinematics
    calls = {"factory": 0, "provider": 0}
    cal = {name: {"range_min": 1200, "range_max": 2900} for name in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")}
    raw = {name: 2050 for name in cal}
    def factory(*args, **kwargs):
        calls["factory"] += 1
        return RobotKinematics(*args, **kwargs)
    def provider(percent):
        calls["provider"] += 1
        return np.eye(4)
    with SO101IK(cal, provider, backend_factory=factory) as ik:
        target = ik.forward_raw(raw, opening_raw=2050)
        def forbidden(*args, **kwargs):
            raise AssertionError("Warm path attempted XML parsing or temporary model build")
        monkeypatch.setattr("workshop.digital_twin.ik.ET.fromstring", forbidden)
        monkeypatch.setattr("workshop.digital_twin.ik.ET.parse", forbidden)
        monkeypatch.setattr("workshop.digital_twin.ik.tempfile.TemporaryDirectory", forbidden)
        for delta in (-1, 1, 0, -1):
            measured = dict(raw, gripper=2050 + delta)
            ik.forward_raw(measured, opening_raw=2050)
            assert ik.solve_pinch(target, raw).success
        assert calls == {"factory": 1, "provider": 1}
