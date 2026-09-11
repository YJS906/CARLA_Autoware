# Copyright 2026 selfcar contributors
# SPDX-License-Identifier: Apache-2.0

import os
import unittest

from rclpy.parameter import Parameter
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2
from autoware_perception_msgs.msg import DetectedObject, DetectedObjects

import test_ros as ros_fixture
from test_cloud_geometry import cloud
from selfcar_obstacle_timeout_replan.aeb_cloud_node import AebObjectCloudFilter


@unittest.skipUnless(
    os.environ.get("SELFCAR_ISOLATED_TEST") == "1" and os.environ.get("ROS_DOMAIN_ID") == "220",
    "Requires isolated --network none test container")
class TestAebTransport(unittest.TestCase):
    spin = ros_fixture.TestRosTransport.spin
    sequence = ros_fixture.TestRosTransport.sequence

    def setUp(self):
        ros_fixture.TestRosTransport.setUp(self)
        self.aeb = AebObjectCloudFilter(
            namespace="/control", parameter_overrides=[Parameter("use_sim_time", value=True)],
            cli_args=["--ros-args", "-r", "~/input/objects:=/perception/object_recognition/objects",
                      "-r", "~/input/detected_objects:=/test/detections",
                      "-r", "~/input/excluded_objects:=/planning/obstacle_timeout_replan/excluded_objects",
                      "-r", "~/input/pointcloud:=/test/raw_cloud", "-r", "~/output/pointcloud:=/test/aeb_cloud"])
        self.executor.add_node(self.aeb)
        self.cloud_pub = self.driver.create_publisher(PointCloud2, "/test/raw_cloud", 1)
        self.detected_pub = self.driver.create_publisher(DetectedObjects, "/test/detections", 1)
        self.clouds = []
        self.cloud_sub = self.driver.create_subscription(PointCloud2, "/test/aeb_cloud", self.clouds.append, 1)
        self.spin(0.05)

    def tick(self, now, **kwargs):
        raw, output = ros_fixture.TestRosTransport.tick(self, now, **kwargs)
        detected = DetectedObjects()
        detected.header = raw.header
        for obj in raw.objects:
            detection = DetectedObject()
            detection.kinematics.pose_with_covariance = obj.kinematics.initial_pose_with_covariance
            detection.kinematics.twist_with_covariance = obj.kinematics.initial_twist_with_covariance
            detection.shape = obj.shape
            detected.objects.append(detection)
        self.detected_pub.publish(detected)
        self.spin(0.002)
        return raw, output

    def tearDown(self):
        self.executor.remove_node(self.aeb)
        self.aeb.destroy_node()
        ros_fixture.TestRosTransport.tearDown(self)

    def send_cloud(self, frame="map"):
        msg = cloud([(7.75, 0, 0, 1), (17.75, 0, 0, 2), (27.75, 0, 0, 3), (50, 0, 0, 4)])
        msg.header.stamp = self.filter.get_clock().now().to_msg()
        msg.header.frame_id = frame
        count = len(self.clouds)
        self.cloud_pub.publish(msg)
        self.spin(0.02)
        if frame == "map":
            self.assertGreater(len(self.clouds), count)
        return msg

    def test_real_timeout_snapshot_removes_same_uuid_cloud_only(self):
        self.sequence(100, 109.9)
        raw = self.send_cloud()
        self.assertEqual(serialize_message(raw), serialize_message(self.clouds[-1]))
        self.sequence(110, 110.2)
        self.tick(110.3, ids=(1, 2, 3))
        raw = self.send_cloud()
        self.assertEqual(self.clouds[-1].width, 2)
        self.assertEqual(bytes(self.clouds[-1].data), bytes(raw.data)[32:])
        self.assertEqual(self.aeb.stats['removed_points'], 2)

    def test_disable_restores_both_planning_and_aeb_input(self):
        self.sequence(100, 110.2)
        self.send_cloud()
        self.assertEqual(self.clouds[-1].width, 2)
        self.filter.set_parameters([Parameter("enabled", value=False)])
        self.tick(110.3)
        raw = self.send_cloud()
        self.assertEqual(serialize_message(raw), serialize_message(self.clouds[-1]))
        self.assertEqual(len(self.output[-1].objects), 2)

    def test_stale_snapshot_and_missing_tf_preserve_cloud(self):
        self.sequence(100, 110.2)
        raw = self.send_cloud(frame="no_such_frame")
        self.spin(0.12)
        self.aeb.process()
        self.spin(0.01)
        self.assertEqual(serialize_message(raw), serialize_message(self.clouds[-1]))
        self.assertEqual(self.aeb.stats['reason'], 'transform_unavailable')
        message, received = self.aeb.excluded
        self.aeb.excluded = (message, received - 1.0)
        raw = self.send_cloud()
        self.assertEqual(serialize_message(raw), serialize_message(self.clouds[-1]))


if __name__ == "__main__":
    unittest.main()
