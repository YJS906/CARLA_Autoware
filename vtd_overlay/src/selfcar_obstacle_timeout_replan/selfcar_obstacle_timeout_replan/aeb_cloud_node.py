# Copyright 2026 selfcar contributors
# SPDX-License-Identifier: Apache-2.0

from collections import deque
import json
import math
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from autoware_internal_debug_msgs.msg import StringStamped
from autoware_perception_msgs.msg import DetectedObjects, PredictedObjects
from geometry_msgs.msg import Transform
from sensor_msgs.msg import PointCloud2

from .cloud_geometry import associate_detection_boxes, filter_cloud


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class AebObjectCloudFilter(Node):
    def __init__(self, **kwargs):
        super().__init__("aeb_object_filter", **kwargs)
        self.declare_parameter("enabled", True)
        self.declare_parameter("message_timeout", 0.5)
        self.declare_parameter("max_object_time_offset", 0.25)
        self.declare_parameter("matching_margin", 0.1)
        self.declare_parameter("object_z_is_ground", True)
        self.declare_parameter("transform_wait", 0.1)
        self.declare_parameter("association_distance", 0.5)
        self.raw = None
        self.detected = None
        self.excluded = None
        self.pending = deque()
        self.last_status = -math.inf
        self.last_clock = None
        self.stats = {"reason": "no_exclusions", "input_points": 0, "removed_points": 0}
        self.buffer = Buffer(cache_time=Duration(seconds=3.0), node=self)
        self.listener = TransformListener(self.buffer, self, spin_thread=False)
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(PointCloud2, "~/output/pointcloud", reliable)
        self.status_publisher = self.create_publisher(StringStamped, "~/status", reliable)
        self.subscriptions_ = [
            self.create_subscription(PredictedObjects, "~/input/objects", self.on_objects, reliable),
            self.create_subscription(DetectedObjects, "~/input/detected_objects", self.on_detected, reliable),
            self.create_subscription(PredictedObjects, "~/input/excluded_objects", self.on_excluded, reliable),
            self.create_subscription(PointCloud2, "~/input/pointcloud", self.on_cloud, sensor),
        ]
        self.timer = self.create_timer(0.02, self.process)

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_objects(self, message):
        self.raw = (message, self.now_seconds())

    def on_excluded(self, message):
        self.excluded = (message, self.now_seconds())

    def on_detected(self, message):
        self.detected = (message, self.now_seconds())

    def fresh(self, stored, now):
        if stored is None:
            return None
        message, received = stored
        timeout = self.get_parameter("message_timeout").value
        age = now - seconds(message.header.stamp)
        if not (math.isfinite(timeout) and timeout > 0
                and 0 <= now - received <= timeout and -0.1 <= age <= timeout):
            return None
        return message

    def inputs(self, cloud, now):
        if not self.get_parameter("enabled").value or not self.get_parameter("use_sim_time").value:
            return None, set(), "disabled"
        excluded = self.fresh(self.excluded, now)
        if excluded is None or not excluded.objects:
            return None, set(), "no_fresh_exclusions"
        objects = self.fresh(self.raw, now)
        detected = self.fresh(self.detected, now)
        if objects is None or objects.header.frame_id != excluded.header.frame_id:
            return None, set(), "no_matching_objects"
        if detected is None or detected.header.frame_id != objects.header.frame_id:
            return None, set(), "no_matching_detections"
        offset = abs(seconds(cloud.header.stamp) - seconds(objects.header.stamp))
        limit = self.get_parameter("max_object_time_offset").value
        if not math.isfinite(limit) or limit <= 0 or offset > limit:
            return None, set(), "object_time_mismatch"
        if abs(seconds(cloud.header.stamp) - seconds(detected.header.stamp)) > 0.1:
            return None, set(), "detection_time_mismatch"
        cloud_age = now - seconds(cloud.header.stamp)
        if not -0.1 <= cloud_age <= self.get_parameter("message_timeout").value:
            return None, set(), "stale_cloud"
        ignored = {bytes(obj.object_id.uuid) for obj in excluded.objects}
        try:
            geometry, matched = associate_detection_boxes(
                objects, detected, ignored, self.get_parameter("association_distance").value)
        except ValueError:
            return None, set(), "uncertain_association"
        return geometry, matched, "matched_snapshot" if matched else "no_unique_association"

    def on_cloud(self, message):
        if len(self.pending) >= 8:
            oldest, _ = self.pending.popleft()
            self.publish(oldest, 0, "transform_queue_full")
        self.pending.append((message, time.monotonic()))
        self.process()

    def publish(self, message, removed, reason, elapsed_ms=0.0):
        self.publisher.publish(message)
        self.stats = {"reason": reason, "input_points": message.width * message.height + removed,
                      "removed_points": removed, "output_points": message.width * message.height,
                      "filter_ms": elapsed_ms}

    def process(self):
        now = self.now_seconds()
        if self.last_clock is not None and now < self.last_clock:
            self.pending.clear()
            self.raw = None
            self.detected = None
            self.excluded = None
        self.last_clock = now
        while self.pending:
            cloud, received = self.pending[0]
            objects, ignored, reason = self.inputs(cloud, now)
            if not ignored:
                self.pending.popleft()
                self.publish(cloud, 0, reason)
                continue
            try:
                if cloud.header.frame_id == objects.header.frame_id:
                    transform = Transform()
                    transform.rotation.w = 1.0
                else:
                    transform = self.buffer.lookup_transform(
                        cloud.header.frame_id, objects.header.frame_id,
                        Time.from_msg(cloud.header.stamp)).transform
            except TransformException:
                # Keep a short FIFO so a newer cloud cannot starve a frame waiting for TF.
                wait = self.get_parameter("transform_wait").value
                if math.isfinite(wait) and 0 < wait <= 0.2 and time.monotonic() - received < wait:
                    break
                self.pending.popleft()
                self.publish(cloud, 0, "transform_unavailable")
                continue
            started = time.perf_counter()
            try:
                output, removed = filter_cloud(
                    cloud, objects, ignored, transform,
                    margin=self.get_parameter("matching_margin").value,
                    z_is_ground=self.get_parameter("object_z_is_ground").value)
                reason = "filtered" if removed else "no_corresponding_points"
            except (ValueError, TypeError, BufferError, OverflowError) as error:
                output, removed, reason = cloud, 0, "uncertain_geometry: " + str(error)
            self.pending.popleft()
            self.publish(output, removed, reason, (time.perf_counter() - started) * 1000)
        if now < self.last_status or now - self.last_status >= 1.0:
            status = StringStamped()
            status.stamp = self.get_clock().now().to_msg()
            status.data = json.dumps(self.stats)
            self.status_publisher.publish(status)
            self.last_status = now


def main(args=None):
    rclpy.init(args=args)
    node = AebObjectCloudFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
