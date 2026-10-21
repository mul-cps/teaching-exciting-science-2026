"""Bounded topic-only ROS observer. Runs inside existing runtime, never opens USB."""
import argparse
import json
import math
import time

JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')
MAX_AGE = 1.0
MAX_JOINT_VALUES = 16
MAX_JOINT_NAME_CHARS = 64
MAX_STATUS_CHARS = 8192
MAX_REASON_CHARS = 256


class Cache:
    def __init__(self):
        self.items = {}

    def remember(self, label, value, source_age, now):
        # Invalid source time cannot become valid merely by waiting.
        if not math.isfinite(source_age) or source_age < 0:
            source_age = None
        self.items[label] = (value, source_age, now)

    def snapshot(self, now):
        out = {}
        for label in ('leader', 'follower', 'teleop'):
            if label not in self.items:
                out[label] = {'state': 'not_running', 'reason': 'no message received'}
                continue
            value, age, received = self.items[label]
            receipt_age = now - received
            source_age = age + receipt_age if age is not None else None
            fresh = source_age is not None and all(math.isfinite(x) and 0 <= x <= MAX_AGE for x in (receipt_age, source_age))
            item = {'state': 'fresh' if fresh else 'stale', 'receipt_age_s': receipt_age,
                    'source_age_s': source_age}
            # Expired values are never returned as current poses or status.
            if fresh:
                item['value' if label == 'teleop' else 'degrees'] = value
            out[label] = item
        return out


def collect(duration):
    import rclpy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    rclpy.init()
    node = rclpy.create_node('workshop_read_only_observer')
    cache = Cache()
    def joints(label):
        def callback(msg):
            if (not len(JOINTS) <= len(msg.name) <= MAX_JOINT_VALUES or
                    len(msg.position) != len(msg.name) or
                    any(not isinstance(name, str) or len(name) > MAX_JOINT_NAME_CHARS for name in msg.name) or
                    len(set(msg.name)) != len(msg.name)):
                return
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            age = node.get_clock().now().nanoseconds / 1e9 - stamp
            values = dict(zip(msg.name, msg.position))
            if not all(j in values and math.isfinite(values[j]) for j in JOINTS):
                return
            degrees = {j: math.degrees(values[j]) for j in JOINTS}
            if not all(math.isfinite(value) for value in degrees.values()):
                return
            cache.remember(label, degrees, age, time.monotonic())
        return callback
    def status(msg):
        if not isinstance(msg.data, str) or len(msg.data) > MAX_STATUS_CHARS:
            return
        try:
            value = json.loads(msg.data)
            if (not isinstance(value, dict) or type(value.get('schema')) is not int or value['schema'] != 1 or
                    type(value.get('stamp_s')) not in (int, float) or not math.isfinite(value['stamp_s']) or
                    value.get('state') not in ('disabled', 'starting', 'active', 'stopping') or
                    value.get('teleop_mode') not in ('relative', 'absolute') or
                    not isinstance(value.get('reason'), str) or len(value['reason']) > MAX_REASON_CHARS):
                return
            projected = {key: value[key] for key in ('schema', 'stamp_s', 'state', 'teleop_mode', 'reason')}
            now = time.monotonic()
            cache.remember('teleop', projected, now - value['stamp_s'], now)
        except (ValueError, KeyError, TypeError, OverflowError, RecursionError):
            return
    # Best-effort readers accept both reliable and best-effort joint publishers.
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(JointState, '/leader/joint_states', joints('leader'), qos)
    node.create_subscription(JointState, '/follower/joint_states', joints('follower'), qos)
    node.create_subscription(String, '/follower/teleop_status', status, status_qos)
    samples = []
    started = time.monotonic()
    next_sample = started
    try:
        while time.monotonic() - started < duration:
            rclpy.spin_once(node, timeout_sec=.05)
            now = time.monotonic()
            if now >= next_sample:
                samples.append({'elapsed_s': now-started, **cache.snapshot(now)})
                next_sample = now + .2
        return {'mode': 'LIVE — ROS topic observations', 'samples': samples,
                'freshness_note': 'Joint source_age is ROS header age; follower header marks publication, not independent motor acquisition. Teleop stamp is same-host monotonic. Stale disabled latched status means last known state only; no motion permission.'}
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('duration', nargs='?', type=float, default=3)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not .5 <= args.duration <= 5:
        parser.error('duration must be 0.5 to 5 seconds')
    print(json.dumps(collect(args.duration), allow_nan=False))


if __name__ == '__main__':
    main()
