#!/usr/bin/env python3
"""Subscribe to the existing ROS camera owner; never opens or launches a camera."""
import argparse
from collections import deque
import json
from pathlib import Path
import time

import cv2
import numpy as np


def main():
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener
    from scipy.spatial.transform import Rotation
    from rclpy.time import Time
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=12.)
    args = parser.parse_args()
    if not 0 < args.seconds <= 30:
        parser.error('capture duration must be positive and at most30seconds')
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Node('colored_validation_readonly_capture')
    bridge, buffers = CvBridge(), {name: deque(maxlen=40) for name in ('color','aligned','raw','info')}
    tf = Buffer()
    listener = TransformListener(tf, node)
    subscriptions, statuses, frames = [], [], []
    stamp = lambda message: message.header.stamp.sec+message.header.stamp.nanosec*1e-9
    topics = {'color': '/camera/color/image_raw', 'aligned': '/camera/aligned_depth_to_color/image_raw',
              'raw': '/camera/depth/image_rect_raw', 'info': '/camera/color/camera_info'}
    for name, topic in topics.items():
        subscriptions.append(node.create_subscription(CameraInfo if name == 'info' else Image,
            topic, lambda message, name=name: buffers[name].append(message), qos_profile_sensor_data))
    def status(message):
        try:
            statuses.append(json.loads(message.data))
        except ValueError:
            pass
    subscriptions.append(node.create_subscription(String, '/so101/grasp_status', status, 10))
    started, last_stamp = time.monotonic(), -float('inf')
    try:
        while time.monotonic()-started < args.seconds:
            rclpy.spin_once(node, timeout_sec=.05)
            if not buffers['color'] or any(not values for values in buffers.values()):
                continue
            color = buffers['color'][-1]
            target = stamp(color)
            if target-last_stamp < .8:
                continue
            pair = {name: min(values, key=lambda value: abs(stamp(value)-target)) for name, values in buffers.items()}
            if max(abs(stamp(value)-target) for value in pair.values()) > .002:
                continue
            if pair['aligned'].header.frame_id != color.header.frame_id or pair['info'].header.frame_id != color.header.frame_id:
                continue
            directory = args.output/f'frame-{len(frames):03d}'
            directory.mkdir()
            rgb = bridge.imgmsg_to_cv2(color, desired_encoding='rgb8')
            aligned = bridge.imgmsg_to_cv2(pair['aligned'], desired_encoding='passthrough')
            raw = bridge.imgmsg_to_cv2(pair['raw'], desired_encoding='passthrough')
            if not cv2.imwrite(str(directory/'color.png'), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
                raise RuntimeError('RGB artifact write failed')
            np.save(directory/'depth_aligned.npy', aligned)
            np.save(directory/'depth_raw.npy', raw)
            info = pair['info']
            metadata = {'stamp': target, 'stamps': {name: stamp(value) for name,value in pair.items()},
                'frames': {name: value.header.frame_id for name,value in pair.items()},
                'encodings': {name: value.encoding for name,value in pair.items() if name != 'info'},
                'k': list(info.k), 'd': list(info.d), 'distortion_model': info.distortion_model,
                'width': int(info.width), 'height': int(info.height), 'board_from_camera': None}
            try:
                transform = tf.lookup_transform('charuco_board', color.header.frame_id, Time.from_msg(color.header.stamp)).transform
                q, t = transform.rotation, transform.translation
                matrix = np.eye(4)
                matrix[:3,:3] = Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
                matrix[:3,3] = [t.x,t.y,t.z]
                metadata['board_from_camera'] = matrix.tolist()
            except Exception as error:
                metadata['tf_error'] = str(error)
            (directory/'info.json').write_text(json.dumps(metadata, indent=2))
            frames.append({'directory': directory.name, **metadata})
            last_stamp = target
    finally:
        node.destroy_node()
        rclpy.shutdown()
        (args.output/'manifest.json').write_text(json.dumps({'state': 'readonly_ros_subscriber',
            'matched_tolerance_s': .002, 'frames': frames, 'status_count': len(statuses)}, indent=2))
        (args.output/'statuses.json').write_text(json.dumps(statuses, indent=2))
    print(json.dumps({'frames':len(frames),'statuses':len(statuses),'output':str(args.output)}))


if __name__ == '__main__':
    main()
