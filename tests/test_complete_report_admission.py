"""CPU completion admission: real tracker and publisher, no ROS initialization."""
import ast
import copy
import json
import sys
import time
from pathlib import Path
from types import MethodType, SimpleNamespace as NS

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2/src/so101_station_objects'))
from so101_station_objects import objects as implementation

SOURCE = ROOT / 'ros2/src/so101_station_objects/so101_station_objects/objects.py'


def methods():
    tree = ast.parse(SOURCE.read_text())
    selected = [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and
                n.name in {'_complete_report', '_consume_analysis', '_publish_report', '_publish_unavailable'}]
    # Load the actual pure stamp helpers without importing their ROS module.
    helpers = ast.parse((ROOT / 'ros2/src/so101_station_perception/so101_station_perception/tag_observations.py').read_text())
    selected += [n for n in helpers.body if isinstance(n, ast.FunctionDef) and n.name in {'stamp_is_fresh', 'stamp_seconds'}]
    for fn in selected:
        fn.body = [n for n in fn.body if not (isinstance(n, ast.ImportFrom) and
                   n.module == 'so101_station_perception.tag_observations')]
    namespace = dict(vars(implementation))
    namespace.update(copy=copy, json=json, time=time, np=np,
                     Time=NS(from_msg=lambda stamp: stamp), TransformException=RuntimeError)
    class Message:
        DELETEALL = 3
        def __init__(self):
            self.header = NS(stamp=None, frame_id='')
            self.markers, self.poses, self.detections = [], [], []
    namespace.update({name: Message for name in ('Detection3DArray', 'MarkerArray', 'Marker', 'PoseArray', 'String')})
    exec(compile(ast.fix_missing_locations(ast.Module(body=selected, type_ignores=[])), str(SOURCE), 'exec'), namespace)
    return {n.name: namespace[n.name] for n in selected}


def observation(x=0., color='red'):
    return {'color': color, 'position': np.array([x, .1, .02])}


def report(objects=None, cached=False):
    return {'objects': [] if objects is None else objects,
            'support_plane': {'normal': np.array([0., 0., 1.]), 'd': 0.,
                              'plane_source': 'cached_measured_support_plane' if cached else 'measured',
                              'support_points': 20, 'support_inliers': 20,
                              'inlier_mask': np.ones(4, bool), 'residuals_m': np.zeros(4)},
            'observation_depth_range_m': [.1, 1.2]}


def node(*, real_publisher=False):
    functions = methods()
    assert '_complete_report' in functions, 'common completion sink is missing'
    events, published, unavailable = [], [], []
    n = NS(tracker=implementation.StableObjectTracker(acquisition_min_frames=3),
           last_support_plane=None, last_analysis_result_stamp=None,
           require_registration_tracking=False, board_frame='charuco_board',
           max_observation_depth=1.2, last_marker_update_mono=time.monotonic(),
           last_marker_update_stamp=None, marker_clear_sent=True,
           get_clock=lambda: NS(now=lambda: NS(nanoseconds=int(n.now * 1e9))), now=10.1,
           derived=NS(check=lambda token: events.append('check')))
    for name in ('_complete_report', '_publish_unavailable'):
        setattr(n, name, MethodType(functions[name], n))
    publisher = NS(publish=published.append)
    for name in ('detections_pub', 'markers_pub', 'grasps_pub', 'contacts_pub', 'status_pub'):
        setattr(n, name, publisher)
    n._publish_empty = lambda stamp, **kw: unavailable.append(stamp)
    if real_publisher:
        n._publish_report = MethodType(functions['_publish_report'], n)
    else:
        def publish(payload, stamp, derived_token=None, *, commit=None):
            if derived_token is not None:
                n.derived.check(derived_token)
            assert commit is not None
            commit()
            published.append((payload, stamp))
            return True
        n._publish_report = publish
    n.events, n.published, n.unavailable = events, published, unavailable
    return n


def job(stamp=10., token=None):
    return {'stamp': stamp, 'ros_stamp': NS(sec=int(stamp), nanosec=round((stamp % 1) * 1e9)),
            'derived_token': token}


def test_common_sink_success_measured_cached_empty_and_independent_acquisitions():
    n = node()
    for index, stamp in enumerate((10., 10.1, 10.2)):
        n.now = stamp + .01
        original = report([observation()])
        original_objects, original_plane = original['objects'], original['support_plane']
        n._complete_report(job(stamp), original)
        payload, original_stamp = n.published[-1]
        assert original_stamp.sec == 10 and original_stamp.nanosec == index * 100_000_000
        assert len(payload['objects']) == int(index == 2)
        assert original['objects'] is original_objects and original['support_plane'] is original_plane
        assert 'acquisition' not in original and 'stamp' not in original
    assert payload['objects'][0]['track_id'] == 1
    assert payload['objects'][0]['acquisition_observations'] == 3
    cache = n.last_support_plane
    n.now = 10.31
    n._complete_report(job(10.3), report([observation()], cached=True))
    assert n.last_support_plane is cache
    n.now = 10.41
    n._complete_report(job(10.4), report())
    assert n.published[-1][0]['objects'] == [] and n.last_analysis_result_stamp == 10.4


@pytest.mark.parametrize('expiry', ['attachment', 'materialization'])
def test_late_expiry_does_not_advance_cache_ids_or_existing_nested_tracks(expiry):
    n = node(real_publisher=expiry == 'materialization')
    for stamp in (9.5, 9.6, 9.7):
        n.tracker.update([observation()], stamp)
    n.tracker.update([observation(), observation(.3, 'blue')], 9.8)
    n.tracker.owned.add(1)
    n.tracker.attachment_records[1] = {'phase': 'locked', 'world_frame': 'world', 'world_position': [0., .1, .02]}
    old = n.tracker
    tracks = old.tracks
    nested = {key: (value, dict(value), copy.deepcopy(value)) for key, value in tracks.items()}
    owned, attachments = old.owned, old.attachment_records
    cache = n.last_support_plane = {'old': True}
    n.last_analysis_result_stamp = 9.8
    next_id = old.next_id
    revoked = [False]
    def lookup(*args):
        if expiry == 'attachment':
            revoked[0] = True
        return NS(transform=NS(translation=NS(x=0., y=0., z=0.), rotation=NS(x=0., y=0., z=0., w=1.)))
    n.tf_buffer = NS(lookup_transform=lookup)
    def check(token):
        if revoked[0]:
            raise ValueError('expired at ' + expiry)
    n.derived.check = check
    if expiry == 'materialization':
        # Empty tracked output avoids ROS message stubs for object fields. The
        # actual publisher's JSON serialization is the materialization boundary.
        class DuringMaterialization(dict):
            def items(self):
                revoked[0] = True
                return super().items()
        payload = report([observation(.5, 'green')])
        payload['validation_diagnostics'] = DuringMaterialization(value=1)
    else:
        payload = report([observation(.02), observation(.5, 'green')])
    n._complete_report(job(token=object()), payload)
    assert n.tracker is old and n.last_support_plane is cache
    assert n.last_analysis_result_stamp == 9.8 and old.next_id == next_id
    assert old.owned is owned and old.attachment_records is attachments
    assert old.owned == {1} and attachments[1]['world_position'] == [0., .1, .02]
    assert list(old.tracks) == [1]  # Existing unavailable policy breaks blue probation.
    assert old.last_acquisition_stamp == 10.  # Trusted failure barrier, not accepted observation.
    for key, (identity, aliases, before) in nested.items():
        assert tracks[key] is identity
        for field, value in before.items():
            assert identity[field] is aliases[field]
            if isinstance(value, np.ndarray):
                np.testing.assert_array_equal(identity[field], value)
            else:
                assert identity[field] == value
    # One unavailable status is allowed; no fresh detection/marker/grasp/contact.
    assert len(n.published) == 1 and json.loads(n.published[0].data)['state'] == 'unavailable'
    assert not n.unavailable
    # A rejected new candidate cannot consume an ID or contribute evidence.
    accepted = []
    def admit(payload, stamp, *, commit, **kwargs):
        commit()
        accepted.append(payload)
        return True
    n._publish_report = admit
    for stamp in (10.1, 10.2, 10.3):
        n.now = stamp + .01
        n._complete_report(job(stamp), report([observation(.5, 'green')]))
    assert [len(value['objects']) for value in accepted] == [0, 0, 1]
    assert accepted[-1]['objects'][0]['track_id'] == next_id
    assert accepted[-1]['objects'][0]['acquisition_observations'] == 3


def test_real_publisher_commit_once_after_materialization_before_all_five_publishes():
    n = node(real_publisher=True)
    order = []
    n.derived.check = lambda token: order.append('admission')
    for name in ('detections_pub', 'markers_pub', 'grasps_pub', 'contacts_pub', 'status_pub'):
        setattr(n, name, NS(publish=lambda msg, name=name: order.append(name)))
    payload = report()
    payload['stamp'] = 10.
    payload['support_plane'] = {}
    assert n._publish_report(payload, job()['ros_stamp'], object(), commit=lambda: order.append('commit')) is True
    assert order == ['admission', 'commit', 'detections_pub', 'markers_pub', 'grasps_pub', 'contacts_pub', 'status_pub']
    n.derived.check = lambda token: (_ for _ in ()).throw(ValueError('expired'))
    order.clear()
    assert n._publish_report(payload, job()['ros_stamp'], object(), commit=lambda: order.append('commit')) is False
    assert order == ['status_pub']


def encode_baseline(value):
    """Use the retained oracle's lossless array/scalar tags, never new expectations."""
    if isinstance(value, np.ndarray):
        return {'__ndarray__': True, 'dtype': str(value.dtype), 'shape': list(value.shape), 'data': value.tolist()}
    if isinstance(value, np.generic):
        return {'__numpy_scalar__': str(value.dtype), 'data': value.item()}
    if isinstance(value, dict):
        return {str(key): encode_baseline(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [encode_baseline(item) for item in value]
    if isinstance(value, set):
        return {'__type__': 'builtins.set', 'repr': repr(value)}
    return value


def decode_baseline(value):
    if isinstance(value, list):
        return [decode_baseline(item) for item in value]
    if not isinstance(value, dict):
        return value
    if value.get('__ndarray__') is True:
        return np.array(value['data'], dtype=value['dtype']).reshape(value['shape'])
    if '__numpy_scalar__' in value:
        return np.dtype(value['__numpy_scalar__']).type(value['data'])
    if value.get('__type__') == 'builtins.set':
        assert value['repr'] == 'set()'
        return set()
    return {key: decode_baseline(item) for key, item in value.items()}


def test_retained_five_cpu_reports_match_frozen_consumed_reports_and_state():
    fixture = json.loads((Path(__file__).parent / 'fixtures/complete_report_admission/cpu_baseline.json').read_text())
    assert fixture['provenance']['source_commit'] == '443cd8f82ae19dbb1468088dbf33d9bed6851aaa'
    assert fixture['provenance']['projection']['omitted_report_fields'] == []
    assert [case['label'] for case in fixture['cases']] == [
        'complete_1', 'complete_2', 'complete_3', 'cached_plane', 'rejected_empty']
    n = node()
    before = decode_baseline(fixture['initial_state'])
    n.tracker.__dict__.update(before['tracker'])
    n.last_support_plane = before['last_support_plane']
    n.last_analysis_result_stamp = before['last_analysis_result_stamp']
    for case in fixture['cases']:
        offered = decode_baseline(case['analyzer_report'])
        expected = case['consumed_report']
        stamp = expected['stamp']
        original_stamp = NS(**case['original_ros_stamp'])
        n.now = stamp + .01
        completion = {'stamp': stamp, 'ros_stamp': original_stamp}
        n._complete_report(completion, offered)
        assert n.published[-1][1] is original_stamp
        assert encode_baseline(n.published[-1][0]) == expected
        after = case['accepted_state']
        assert encode_baseline(n.tracker.__dict__) == after['tracker']
        assert encode_baseline(n.last_support_plane) == after['last_support_plane']
        assert n.last_analysis_result_stamp == after['last_analysis_result_stamp']


@pytest.mark.parametrize('objects', [[], [observation()], [observation(-.01), observation(.01)],
                                     [observation(.3, 'green')]])
def test_shallow_tracker_stage_preserves_old_nested_mapping_arrays_and_ownership(objects):
    # Exercise expiry, association, ambiguity and new acquisition independently;
    # the accepted tracker AST guard prevents unreviewed nested-write changes.
    state = implementation.StableObjectTracker(acquisition_min_frames=3)
    for stamp in (8., 8.1, 8.2):
        state.update([observation()], stamp)
    state.update([observation(), observation(.3, 'blue')], 8.3)
    state.owned.add(1)
    state.attachment_records[1] = {'phase': 'locked', 'world_position': np.array([0., .1, .02])}
    mapping, owned, records = state.tracks, state.owned, state.attachment_records
    values = {key: (track, dict(track), copy.deepcopy(track)) for key, track in mapping.items()}
    original_shell = dict(state.__dict__)
    staged = copy.copy(state)
    staged.update(objects, 10., expected_owned_positions={1: np.array([0., .1, .02])})
    assert staged.tracks is not mapping
    assert state.tracks is mapping and state.owned is owned and state.attachment_records is records
    assert state.__dict__ == original_shell
    for key, (track, aliases, saved) in values.items():
        assert mapping[key] is track
        for field, value in saved.items():
            assert track[field] is aliases[field]
            if isinstance(value, np.ndarray):
                np.testing.assert_array_equal(track[field], value)
            else:
                assert track[field] == value
