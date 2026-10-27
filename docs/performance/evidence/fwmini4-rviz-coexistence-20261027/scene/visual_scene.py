#!/usr/bin/env python3
"""Synthetic visual geometry only. RGB comes from the resident DDS Image adapter.

All message objects and point/position payload buffers are constructed once.
Rclpy/RMW serialization and transport may allocate internally; no zero-total-heap
claim is made. The CPU benchmark separately measures native serialization.
"""
from array import array
import argparse
import json
import math
import os
import numpy as np
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState, PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray

POINT_COUNT = 20000
CLOUD_TOPIC = '/rviz_coexistence_scene/synthetic_world_points'
MARKER_TOPIC = '/rviz_coexistence_scene/synthetic_bricks'
JOINT_TOPIC = '/follower/joint_states'


def set_stamp(message, nanoseconds):
    seconds, nanos = divmod(nanoseconds, 1000000000)
    message.header.stamp.sec = seconds
    message.header.stamp.nanosec = nanos


class SceneMessages:
    def __init__(self):
        # Construction only: one vectorized 200x100 colored surface, no Python
        # loop over points. This immutable payload is republished at 10 Hz.
        points = np.empty(POINT_COUNT, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
        x, y = np.meshgrid(np.linspace(-.3, .3, 200, dtype=np.float32), np.linspace(-.2, .2, 100, dtype=np.float32))
        points['x'], points['y'] = x.ravel(), y.ravel()
        points['z'] = .02 + .025 * np.sin(15 * points['x']) * np.cos(15 * points['y'])
        red = np.asarray((points['x'] + .3) / .6 * 255, dtype=np.uint32)
        green = np.asarray((points['y'] + .2) / .4 * 255, dtype=np.uint32)
        points['rgb'] = red << 16 | green << 8 | np.uint32(100)
        self.cloud = PointCloud2()
        self.cloud.header.frame_id = 'charuco_board'
        self.cloud.height, self.cloud.width = 1, POINT_COUNT
        self.cloud.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                             for i, name in enumerate(('x', 'y', 'z', 'rgb'))]
        self.cloud.is_bigendian, self.cloud.is_dense = False, True
        self.cloud.point_step, self.cloud.row_step = 16, POINT_COUNT * 16
        self.cloud.data = array('B', points.tobytes())
        self.joints = JointState()
        self.joints.name = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
        self.joints.position = array('d', [0., -.5, .8, -.4, 0., .3])
        self.markers = MarkerArray()
        markers = []
        for i, color in enumerate(((1., .1, .1), (.1, 1., .1), (.1, .3, 1.))):
            marker = Marker()
            marker.header.frame_id, marker.ns, marker.id = 'charuco_board', 'SYNTHETIC_visual_load_bricks', i
            marker.type, marker.action = Marker.CUBE, Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = -.1 + i*.1, .1, .06
            marker.pose.orientation.w = 1.
            marker.scale.x, marker.scale.y, marker.scale.z = .06, .04, .06
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = *color, 1.
            markers.append(marker)
        label = Marker()
        label.header.frame_id, label.ns, label.id = 'charuco_board', 'SYNTHETIC_visual_load_label', 0
        label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        label.pose.position.z, label.pose.orientation.w, label.scale.z = .33, 1., .025
        label.color.r, label.color.g, label.color.b, label.color.a = 1., .9, .2, 1.
        label.text = 'SYNTHETIC robot/world load; resident RGB is actual camera'
        markers.append(label)
        self.markers.markers = markers
        self.board_to_base = TransformStamped()
        self.board_to_base.header.frame_id, self.board_to_base.child_frame_id = 'charuco_board', 'base_link'
        self.board_to_base.transform.translation.x = -.15
        self.board_to_base.transform.translation.z = .025
        self.board_to_base.transform.rotation.w = 1.

    def update_joint(self, nanoseconds, tick):
        set_stamp(self.joints, nanoseconds)
        # Visualization-only animation in an isolated domain; no command topic.
        self.joints.position[0] = .15 * math.sin(tick * (2 * math.pi / 300))

    def update_world(self, nanoseconds):
        set_stamp(self.cloud, nanoseconds)
        for marker in self.markers.markers:
            set_stamp(marker, nanoseconds)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=('vga', '720p', '1080p'), default='vga', help='Evidence label; actual resident camera profile is selected independently')
    args, ros_args = parser.parse_known_args()
    if os.environ.get('ROS_DOMAIN_ID') != '179':
        raise RuntimeError('visual-only load requires the isolated ROS_DOMAIN_ID=179')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from tf2_ros import StaticTransformBroadcaster

    class VisualScene(Node):
        def __init__(self):
            super().__init__('synthetic_visual_scene', namespace='/rviz_coexistence_scene')
            self.messages, self.tick = SceneMessages(), 0
            self.cloud_publisher = self.create_publisher(PointCloud2, CLOUD_TOPIC, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.marker_publisher = self.create_publisher(MarkerArray, MARKER_TOPIC, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.joint_publisher = self.create_publisher(JointState, JOINT_TOPIC, 1)
            self.static_tf = StaticTransformBroadcaster(self)
            set_stamp(self.messages.board_to_base, self.get_clock().now().nanoseconds)
            self.static_tf.sendTransform(self.messages.board_to_base)
            self.create_timer(1/30, self.on_joint)
            self.create_timer(.1, self.on_world)
            print(json.dumps({'scope': 'SYNTHETIC robot/world load only; actual RGB from /station/resident/frame', 'profile_label': args.profile, 'world_points': POINT_COUNT, 'cloud_hz': 10, 'joint_hz': 30, 'rgb_publisher': False, 'usb_or_control_owner': False}), flush=True)

        def on_joint(self):
            self.messages.update_joint(self.get_clock().now().nanoseconds, self.tick)
            self.tick += 1
            self.joint_publisher.publish(self.messages.joints)

        def on_world(self):
            self.messages.update_world(self.get_clock().now().nanoseconds)
            self.cloud_publisher.publish(self.messages.cloud)
            self.marker_publisher.publish(self.messages.markers)

    rclpy.init(args=ros_args)
    node = VisualScene()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
