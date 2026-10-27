"""Guard optional admission before observable state and unchanged direct paths."""
import ast
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
PERCEPTION=ROOT/'ros2/src/so101_station_perception/so101_station_perception'


def function(path,name):
    tree=ast.parse(path.read_text())
    return next(node for node in ast.walk(tree) if isinstance(node,ast.FunctionDef) and node.name==name)


def test_board_and_tags_have_optional_final_admission():
    for file,name,commit in [('charuco_world.py','_pair','self.filtered_pose = stabilize_pose'),
                             ('tag_observations.py','_images','self.publisher.publish(output)')]:
        fn=function(PERCEPTION/file,name)
        assert 'admission_check' in [arg.arg for arg in fn.args.args]
        text=ast.get_source_segment((PERCEPTION/file).read_text(),fn)
        assert text.index('admission_check()')<text.index(commit)


def test_both_nodes_default_off_and_objects_check_before_tracker():
    for path in [PERCEPTION/'marker_pipeline.py',ROOT/'ros2/src/so101_station_objects/so101_station_objects/objects.py']:
        text=path.read_text();assert '"allow_derived_source", False' in text or '"allow_derived_source": False' in text
        assert 'DerivedAligned' in text and 'derived_token' in text
    path=ROOT/'ros2/src/so101_station_objects/so101_station_objects/objects.py'
    receipt=ast.get_source_segment(path.read_text(),function(path,'_consume_analysis'))
    assert 'self._complete_report(job, report, error)' in receipt
    text=ast.get_source_segment(path.read_text(),function(path,'_complete_report'))
    assert text.index('self.derived.check(')<text.index('staged_tracker.update(')
    assert text.index('lookup_transform(')<text.rindex('self.derived.check(')<text.index('staged_tracker.update(')


def test_adapter_intake_has_no_pixel_decode_copy_or_extra_images():
    path=PERCEPTION/'derived_aligned.py';text=path.read_text()
    for name in ('_intake','drain'):
        fn=function(path,name);source=ast.get_source_segment(text,fn)
        assert not any(token in source for token in ('imgmsg_to_cv2','.copy(', 'np.asarray','bytes(message.data)','list(message.data)'))
    from collections import deque
    tree = ast.parse((PERCEPTION/'aligned_cohort.py').read_text())
    initializer = next(node.value for node in ast.walk(tree)
                       if isinstance(node, ast.Assign) and any(
                           isinstance(target, ast.Attribute)
                           and isinstance(target.value, ast.Name)
                           and target.value.id == 'self' and target.attr == 'streams'
                           for target in node.targets))
    streams = eval(compile(ast.Expression(initializer), '<streams initializer>', 'eval'),
                   {'deque': deque, 'STREAMS': ('color', 'aligned', 'color_info', 'depth_info')})
    assert {key: stream.maxlen for key, stream in streams.items()} == {
        'color': 4, 'aligned': 4, 'color_info': 16, 'depth_info': 16}
    assert 'maxlen=8' in (PERCEPTION/'derived_lineage.py').read_text()
    assert "('color',TOPICS['color'],Image),('aligned',ALIGNED_TOPIC,Image)" in text


def test_launch_default_off_and_recovery_inputs_not_remapped():
    station=(ROOT/'ros2/src/so101_station_bringup/launch/station_live.launch.py').read_text()
    for scope in ('marker','objects'):
        assert f"'{scope}_allow_derived_source': 'false'" in station
        assert f"'{scope}_derived_source_cohort': ''" in station
    assert "'recovery_depth_topic': '/camera/aligned_depth_to_color/image_raw'" in station
    assert "'recovery_camera_info_topic': '/camera/color/camera_info'" in station
    for package,leaf in [('so101_station_objects','objects'),('so101_station_perception','marker_pipeline')]:
        text=(ROOT/f'ros2/src/{package}/launch/{leaf}.launch.py').read_text()
        assert 'allow_derived_source' in text and 'derived_source_cohort' in text


def test_object_final_admission_follows_all_materialization_and_unavailable_unchanged():
    path=ROOT/'ros2/src/so101_station_objects/so101_station_objects/objects.py'
    text=ast.get_source_segment(path.read_text(),function(path,'_publish_report'))
    assert text.index('json.dumps(status')<text.index('self.derived.check(derived_token)')<text.index('commit()')<text.index('self.detections_pub.publish(')
    unavailable=ast.get_source_segment(path.read_text(),function(path,'_publish_unavailable'))
    assert 'derived_token' not in unavailable and 'publish(detections)' not in unavailable
