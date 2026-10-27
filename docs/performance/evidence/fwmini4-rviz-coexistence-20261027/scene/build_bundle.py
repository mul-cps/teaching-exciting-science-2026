#!/usr/bin/env python3
"""Build temporary configuration/assets; pure CPU, never starts ROS or OpenGL."""
from pathlib import Path
import argparse
import os
import shutil
import xml.etree.ElementTree as ET
import yaml
import xacro

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--station-root', required=True)
    parser.add_argument('--description-source', required=True)
    args = parser.parse_args()
    station, original = Path(args.station_root), Path(args.description_source)
    package = ROOT / 'share/so101_description'
    shutil.copytree(original, package, dirs_exist_ok=True)
    index = ROOT / 'share/ament_index/resource_index/packages'
    index.mkdir(parents=True, exist_ok=True)
    (index / 'so101_description').touch()
    os.environ['AMENT_PREFIX_PATH'] = str(ROOT) + os.pathsep + os.environ.get('AMENT_PREFIX_PATH', '')
    description = xacro.process_file(str(package/'urdf/so101_arm.urdf.xacro'), mappings={
        'variant': 'follower', 'use_ros2_control': 'false', 'mesh_dir': (package/'meshes').as_uri()}).toxml()
    assert ET.fromstring(description).find('ros2_control') is None
    (ROOT/'robot.urdf').write_text(description)
    (ROOT/'rsp.yaml').write_text(yaml.safe_dump({'/**': {'ros__parameters': {'robot_description': description, 'publish_frequency': 30.0}}}, sort_keys=False))
    config = yaml.safe_load((station/'ros2/src/so101_station_perception/config/station_live.rviz').read_text())
    config['Panels'] = [p for p in config['Panels'] if p['Class'] in {'rviz_common/Displays', 'rviz_common/Views'}]
    manager = config['Visualization Manager']
    keep = {'Grid', 'Selected robot and board frames', 'Follower robot model', 'Native RGB-D surface', 'Current camera RGB', 'Colored brick tracks'}
    manager['Displays'] = [d for d in manager['Displays'] if d['Name'] in keep]
    for display in manager['Displays']:
        if display['Name'] == 'Native RGB-D surface':
            display['Name'] = 'SYNTHETIC 20k world surface'
            display['Topic']['Value'] = '/rviz_coexistence_scene/synthetic_world_points'
        elif display['Name'] == 'Colored brick tracks':
            display['Name'] = 'SYNTHETIC colored bricks'
            display.pop('Namespaces', None)
            display['Topic']['Value'] = '/rviz_coexistence_scene/synthetic_bricks'
        elif display['Name'] == 'Current camera RGB':
            display['Name'] = 'Resident camera RGB (actual)'
            display['Topic']['Value'] = '/station/resident/frame'
        elif display['Class'].endswith('/TF'):
            display['Frames'] = {'All Enabled': False, 'base_link': {'Value': True}, 'gripper_frame_link': {'Value': True}, 'charuco_board': {'Value': True}}
    manager['Global Options'].update({'Fixed Frame': 'charuco_board', 'Frame Rate': 30})
    manager['Name'] = 'resident_camera_RViz_coexistence'
    # QByteArray saveState contains only widget/dock geometry. This uses plain Qt
    # Widgets with the offscreen platform: no RenderPanel/OpenGL/GPU context.
    os.environ['QT_QPA_PLATFORM'] = 'offscreen'
    from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QDockWidget
    from PyQt5.QtCore import Qt
    app = QApplication([])
    window = QMainWindow()
    window.setCentralWidget(QWidget())
    window.resize(1600, 900)
    displays = QDockWidget('Displays', window)
    displays.setObjectName('Displays')
    displays.setWidget(QWidget())
    window.addDockWidget(Qt.LeftDockWidgetArea, displays)
    image = QDockWidget('Resident camera RGB (actual)', window)
    image.setObjectName('Resident camera RGB (actual)')
    image.setWidget(QWidget())
    window.addDockWidget(Qt.RightDockWidgetArea, image)
    views = QDockWidget('Views', window)
    views.setObjectName('Views')
    views.setWidget(QWidget())
    window.addDockWidget(Qt.RightDockWidgetArea, views)
    window.splitDockWidget(image, views, Qt.Vertical)
    window.show()
    app.processEvents()
    window.resizeDocks([image, views], [360, 500], Qt.Vertical)
    window.resizeDocks([displays, image], [260, 640], Qt.Horizontal)
    app.processEvents()
    assert not image.isFloating()
    manager['Views']['Current'].update({'Distance': 1.15, 'Pitch': .35, 'Yaw': .4})
    config['Window Geometry'] = {'Width': 1600, 'Height': 900, 'X': 0, 'Y': 0,
                                'Displays': {'collapsed': False}, 'Views': {'collapsed': False},
                                'Resident camera RGB (actual)': {'collapsed': False},
                                'QMainWindow State': bytes(window.saveState()).hex()}
    (ROOT/'rviz-coexistence.rviz').write_text(yaml.safe_dump(config, sort_keys=False))
    (ROOT/'layout_requested.yaml').write_text(yaml.safe_dump({'main_window': [1600, 900], 'rgb_dock': {'area': 'right', 'floating': False, 'requested_width': 640, 'requested_height': 360, 'views_panel_below_rgb': True}, 'actual_runtime_drawable_extents_require_verification': True}))
    window.close()
    app.quit()


if __name__ == '__main__':
    main()
