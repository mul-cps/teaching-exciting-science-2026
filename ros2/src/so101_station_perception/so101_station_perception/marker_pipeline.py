"""Single-pass grayscale RGB-D processing for ChArUco and AprilTag outputs."""

import json
import os
import time

import cv2
import message_filters
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from .charuco_world import CharucoWorld
from .latest_cohort import LatestCohort
from .tag_observations import TagObservations, marker_grayscale, stamp_is_fresh, stamp_seconds


def dispatch_shared_gray(color, depth, info, bridge, board_callback, tags_callback):
    """Decode one mono8 image and invoke both existing callbacks with that frame."""
    started = time.perf_counter()
    gray = marker_grayscale(bridge, color)
    decoded = time.perf_counter()
    board_callback(color, info, gray)
    board_done = time.perf_counter()
    tags_callback(color, depth, gray)
    finished = time.perf_counter()
    return {
        "capture_stamp": stamp_seconds(color.header.stamp),
        "decode_ms": (decoded - started) * 1000.0,
        "board_ms": (board_done - decoded) * 1000.0,
        "tags_ms": (finished - board_done) * 1000.0,
        "callback_ms": (finished - started) * 1000.0,
    }


class MarkerPipeline(CharucoWorld):
    """Run the existing standalone perception nodes from one synchronized frame."""

    def __init__(self):
        super().__init__(subscribe_images=False)
        self.require_depth_plane = True
        self.declare_parameter("publish_timing", False)
        self.declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw")
        value = lambda key: self.get_parameter(key).value
        tag_overrides = [Parameter(name="color_topic", value=value("color_topic")),
                         Parameter(name="camera_info_topic", value=value("camera_info_topic")),
                         Parameter(name="max_pair_delta_s", value=value("max_pair_delta_s")),
                         Parameter(name="max_processing_hz", value=value("max_processing_hz")),
                         Parameter(name="depth_topic", value=value("depth_topic")),
                         # Tag centers are computed in optical coordinates. Let
                         # TF carry them into charuco_board instead of relabeling.
                         Parameter(name="diagnostic_frame", value=value("optical_frame"))]
        self.tags = TagObservations(subscribe_images=False, use_global_arguments=False,
                                    parameter_overrides=tag_overrides)
        self.max_age = float(value("max_age_s"))
        self.max_pair_delta = float(value("max_pair_delta_s"))
        self.scheduler = LatestCohort(float(value("max_processing_hz")))
        self.intake_group = MutuallyExclusiveCallbackGroup()
        self.timing_pub = self.create_publisher(
            String, "/so101_perception/marker_pipeline/timing", 10)
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=2,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        color = message_filters.Subscriber(
            self, Image, value("color_topic"), qos_profile=qos, callback_group=self.intake_group)
        depth = message_filters.Subscriber(
            self, Image, value("depth_topic"), qos_profile=qos, callback_group=self.intake_group)
        info = message_filters.Subscriber(
            self, CameraInfo, value("camera_info_topic"), qos_profile=qos, callback_group=self.intake_group)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color, depth, info], queue_size=3, slop=self.max_pair_delta)
        self.sync.registerCallback(self._images)
        # Default group serializes detector/board mutations with _stale_status;
        # the separate intake group keeps message_filters moving during compute.
        self.drain_timer = self.create_timer(.01, self._drain)

    def _images(self, color, depth, info):
        # Reject bad/future metadata before advancing exposure high-water marks.
        # Intake never mutates detector/status state or decodes image buffers.
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = stamp_seconds(color.header.stamp)
        if (not all(stamp_is_fresh(now, stamp_seconds(message.header.stamp), self.max_age)
                    for message in (color, depth, info))
                or color.header.frame_id != self.optical_frame
                or color.header.frame_id != depth.header.frame_id
                or color.header.frame_id != info.header.frame_id
                or abs(stamp_seconds(depth.header.stamp) - stamp) > self.max_pair_delta
                or abs(stamp_seconds(info.header.stamp) - stamp) > self.max_pair_delta):
            return
        key = lambda image: image.header.stamp.sec * 1_000_000_000 + image.header.stamp.nanosec
        self.scheduler.offer((color, depth, info), (key(color), key(depth)), time.monotonic())

    def _drain(self):
        started = time.monotonic()
        item = self.scheduler.take(started)
        if item is None:
            return
        (color, depth, info), receipt = item
        stale = False
        try:
            now = self.get_clock().now().nanoseconds * 1e-9
            stamp = stamp_seconds(color.header.stamp)
            stale = not all(stamp_is_fresh(now, stamp_seconds(message.header.stamp), self.max_age)
                            for message in (color, depth, info))
            if stale:
                self._status("stale", stamp=stamp, reason="queued original image timestamp is stale")
                self.tags._empty(color.header.stamp, color.header.frame_id)
                return
            self._process_images(color, depth, info, (started - receipt) * 1000.)
        finally:
            self.scheduler.complete(stale=stale)

    def close_scheduler(self):
        self.scheduler.close()
        self.drain_timer.cancel()

    def _process_images(self, color, depth, info, queue_wait_ms):
        stamp = stamp_seconds(color.header.stamp)
        now = self.get_clock().now().nanoseconds * 1e-9
        if not stamp_is_fresh(now, stamp, self.max_age):
            return
        if (color.header.frame_id != depth.header.frame_id
                or color.header.frame_id != info.header.frame_id
                or abs(stamp_seconds(depth.header.stamp) - stamp) > self.max_pair_delta
                or abs(stamp_seconds(info.header.stamp) - stamp) > self.max_pair_delta):
            self._status("invalid", stamp=stamp, reason="RGB-D/CameraInfo frames or stamps do not match")
            self.tags._empty(color.header.stamp, color.header.frame_id)
            return
        try:
            metrics = dispatch_shared_gray(
                color, depth, info, self.bridge,
                lambda image, camera_info, gray: self._pair(
                    image, camera_info, prepared_gray=gray, depth_msg=depth),
                lambda image, aligned_depth, gray: self._process_tags_with_info(
                    image, aligned_depth, info, gray),
            )
            metrics["queue_wait_ms"] = queue_wait_ms
            metrics["scheduler"] = self.scheduler.metrics()
            metrics["capture_to_callback_ms"] = (now - stamp) * 1000.0
            metrics["capture_to_completion_ms"] = (
                self.get_clock().now().nanoseconds * 1e-6 - stamp * 1000.0)
            metrics["frame_id"] = color.header.frame_id
            metrics["timing_scope"] = "original_capture_stamp; input_age_at_entry; per_stage_monotonic_duration"
            message = String()
            message.data = json.dumps(metrics, sort_keys=True, allow_nan=False)
            if self.get_parameter("publish_timing").value:
                self.timing_pub.publish(message)
        except (ValueError, TypeError, IndexError, cv2.error) as error:
            self.get_logger().warning(f"Shared marker frame unavailable: {error}")
            self._status("invalid", stamp=stamp, reason=str(error))
            self.tags._empty(color.header.stamp, color.header.frame_id)

    def _process_tags_with_info(self, color, depth, info, gray):
        self.tags._camera_info(info)
        self.tags._images(color, depth, prepared_gray=gray)


def shutdown_executor_bounded(executor, timeout_s=2.):
    """Bound cleanup, retaining node/context ownership if native work is stuck.

    rclpy has no public bounded join for MultiThreadedExecutor workers. Read its
    installed ThreadPoolExecutor worker set only after nonwaiting shutdown stops
    submission. If that runtime contract changes, refuse resource destruction.
    """
    executor.shutdown(timeout_sec=0., wait_for_threads=False)
    try:
        workers = tuple(executor._executor._threads)
    except AttributeError:
        return False
    deadline = time.monotonic() + timeout_s
    for worker in workers:
        worker.join(timeout=max(0., deadline - time.monotonic()))
    return all(not worker.is_alive() for worker in workers)


def configure_ros_domain(environ):
    raw = environ.get("ROS_DOMAIN_ID", "76")
    try:
        domain = int(raw)
    except (TypeError, ValueError) as error:
        raise ValueError("marker pipeline requires ROS_DOMAIN_ID in 1..232") from error
    if not 1 <= domain <= 232:
        raise ValueError("marker pipeline requires ROS_DOMAIN_ID in 1..232; domain 0 is forbidden")
    environ.setdefault("ROS_DOMAIN_ID", "76")
    environ.setdefault("ROS_LOCALHOST_ONLY", "1")


def main(args=None):
    configure_ros_domain(os.environ)
    rclpy.init(args=args)
    pipeline = MarkerPipeline()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(pipeline)
    executor.add_node(pipeline.tags)
    try:
        executor.spin()
    finally:
        pipeline.close_scheduler()
        if not shutdown_executor_bounded(executor):
            pipeline.get_logger().error(
                "Marker cleanup exceeded 2s or worker introspection unavailable; "
                "nodes/context retained. External process supervisor must escalate.")
            raise RuntimeError("marker executor cleanup unavailable; resources retained")
        pipeline.tags.destroy_node()
        pipeline.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
