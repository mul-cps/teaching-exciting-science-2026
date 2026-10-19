"""Bounded scheduling independently of ROS image decoding."""
import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
HELPER = ROOT / 'ros2/src/so101_station_perception/so101_station_perception/latest_cohort.py'


def scheduler(hz=5.):
    assert HELPER.exists(), 'latest-cohort scheduling is not implemented'
    spec = importlib.util.spec_from_file_location('latest_cohort', HELPER)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module.LatestCohort(hz)


def test_latest_wins_without_busy_overlap_or_catchup():
    s = scheduler()
    assert s.offer('first', (1, 1), 10.)
    assert s.take(10.) == ('first', 10.)
    assert s.offer('older', (2, 2), 10.01)
    assert s.offer('latest', (3, 3), 10.02)
    assert s.take(11.) is None  # still active
    s.complete()
    assert s.take(11.) == ('latest', 10.02)
    s.complete()
    s.offer('next', (4, 4), 11.)
    assert s.take(11.01) is None  # no catch-up from original 10s start
    assert s.take(11.201) == ('next', 11.)
    assert s.metrics()['overwritten'] == 1


def test_both_exposure_stamps_unique_and_ordered_even_after_drop():
    s = scheduler()
    assert s.offer('first', (10, 20), 0.)
    assert not s.offer('duplicate rgb', (10, 21), .1)
    assert not s.offer('duplicate depth', (11, 20), .1)
    assert not s.offer('old', (9, 19), .1)
    assert s.take(.2) == ('first', 0.)
    s.complete()
    assert not s.offer('replay', (10, 20), .3)
    assert s.offer('fresh', (11, 21), .3)
    s.close()
    assert not s.offer('after close', (12, 22), .4)
    assert s.take(2.) is None


def test_jittered_five_hz_does_not_discard_early_frames():
    s = scheduler();selected=[]
    arrivals=[0., .195, .402, .597, .801, .996]
    for tick in range(122):
        now=tick*.01
        for i,t in enumerate(arrivals):
            if now-.01 < t <= now or tick==0 and t==0:
                s.offer(i,(i+1,i+1),t)
        item=s.take(now)
        if item is not None:selected.append(item[0]);s.complete()
    assert selected==list(range(6))


@pytest.mark.parametrize('rate', [float('nan'), float('inf'), 0., -1., True])
def test_rate_requires_finite_positive_value(rate):
    with pytest.raises(ValueError):scheduler(rate)


def test_five_hz_cap_and_shutdown_discards_pending():
    s=scheduler(100.)
    assert s.period==.2
    s.offer('a',(1,1),0.);assert s.take(0.)==('a',0.)
    s.offer('b',(2,2),.1);s.close();s.complete()
    assert s.take(1.) is None


def test_intake_and_scheduler_have_no_decoding_io_or_per_pixel_work():
    import ast
    tree=ast.parse(HELPER.read_text())
    assert not any(isinstance(n,(ast.For,ast.While,ast.ListComp,ast.SetComp,ast.DictComp,ast.GeneratorExp)) for n in ast.walk(tree))
    source=(HELPER.parent/'marker_pipeline.py').read_text();tree=ast.parse(source)
    intake=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='_images')
    calls={n.func.attr for n in ast.walk(intake) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)}
    assert calls=={'offer','monotonic','get_clock','now'}
    assert 'callback_group=self.intake_group' in source
    assert 'self.create_timer(.01, self._drain)' in source
    assert 'self.scheduler.complete(stale=stale)' in source


def test_entrypoint_defaults_domain76_and_rejects_domain0_before_ros_init():
    import ast
    source=(HELPER.parent/'marker_pipeline.py').read_text();tree=ast.parse(source)
    nodes=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='configure_ros_domain']
    assert len(nodes)==1,'entrypoint domain guard is missing'
    namespace={};exec(compile(ast.Module(body=nodes,type_ignores=[]),'<domain guard>','exec'),namespace)
    guard=namespace['configure_ros_domain'];env={};guard(env)
    assert env=={'ROS_DOMAIN_ID':'76','ROS_LOCALHOST_ONLY':'1'}
    env={'ROS_DOMAIN_ID':'177'};guard(env);assert env['ROS_DOMAIN_ID']=='177'
    for domain in ('0','-1','233','abc','76.5',''):
        with pytest.raises(ValueError):guard({'ROS_DOMAIN_ID':domain})
    main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    calls=[n for n in ast.walk(main) if isinstance(n,ast.Call)]
    guard_call=next(n for n in calls if isinstance(n.func,ast.Name) and n.func.id=='configure_ros_domain')
    init_call=next(n for n in calls if isinstance(n.func,ast.Attribute) and n.func.attr=='init')
    assert guard_call.lineno<init_call.lineno
