# Copyright 2026 selfcar contributors
# SPDX-License-Identifier: Apache-2.0
"""Focused transport checks. Run in an isolated ROS domain with --network none."""

import time
import unittest
import os

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.serialization import serialize_message

from autoware_adapi_v1_msgs.msg import OperationModeState, RouteState
from autoware_internal_planning_msgs.msg import ControlPoint, PlanningFactor, PlanningFactorArray
from autoware_perception_msgs.msg import (
    PredictedObject, PredictedObjects, TrafficLightElement, TrafficLightGroup)
from autoware_planning_msgs.msg import LaneletRoute
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tier4_rtc_msgs.msg import CooperateStatus, CooperateStatusArray, State

from selfcar_obstacle_timeout_replan.node import ObstacleTimeoutReplan


@unittest.skipUnless(
    os.environ.get("SELFCAR_ISOLATED_TEST") == "1" and os.environ.get("ROS_DOMAIN_ID") == "220",
    "Requires an isolated --network none container, ROS_DOMAIN_ID=220, SELFCAR_ISOLATED_TEST=1")
class TestRosTransport(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.driver = rclpy.create_node("timeout_replan_test_driver")
        self.filter = ObstacleTimeoutReplan(
            namespace="/planning",
            parameter_overrides=[Parameter("enabled", value=True), Parameter("use_sim_time", value=True)],
            cli_args=["--ros-args", "-r", "~/input/objects:=/perception/object_recognition/objects",
                      "-r", "~/output/objects:=/planning/obstacle_timeout_replan/objects"])
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.driver)
        self.executor.add_node(self.filter)
        self.output = []
        self.raw_received = []
        self.output_sub = self.driver.create_subscription(
            PredictedObjects, "/planning/obstacle_timeout_replan/objects",
            self.output.append, 10)
        self.raw_sub = self.driver.create_subscription(
            PredictedObjects, "/perception/object_recognition/objects",
            self.raw_received.append, 10)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pubs = {}
        for key, topic, msg_type, qos in (
            ("clock", "/clock", Clock, 10),
            ("objects", "/perception/object_recognition/objects", PredictedObjects, 1),
            ("odom", "/localization/kinematic_state", Odometry, 1),
            ("route", "/planning/mission_planning/route", LaneletRoute, durable),
            ("mode", "/system/operation_mode/state", OperationModeState, durable),
            ("state", "/api/routing/state", RouteState, durable),
            ("obstacle", "/planning/planning_factors/obstacle_stop", PlanningFactorArray, 1),
            ("signal", "/planning/planning_factors/traffic_light", PlanningFactorArray, 1),
            ("light", "/planning/scenario_planning/lane_driving/behavior_planning/debug/traffic_signal",
             TrafficLightGroup, 1),
            ("rtc", "/planning/cooperate_status/lane_change_left", CooperateStatusArray, 1),
        ):
            self.pubs[key] = self.driver.create_publisher(msg_type, topic, qos)
        self.spin(0.15)
        self.mode = OperationModeState()
        self.mode.mode = OperationModeState.AUTONOMOUS
        self.mode.is_autoware_control_enabled = True
        self.state = RouteState()
        self.state.state = RouteState.SET
        self.route = LaneletRoute()
        self.route.uuid.uuid = [50] * 16
        self.pubs["route"].publish(self.route)
        self.pubs["mode"].publish(self.mode)
        self.pubs["state"].publish(self.state)
        self.spin(0.03)

    def tearDown(self):
        self.executor.shutdown()
        self.filter.destroy_node()
        self.driver.destroy_node()
        rclpy.shutdown()

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.001)

    def tick(self, now, *, obstacle=True, red=False, light_id=100, ids=(1, 2), speed=0.0,
             running=False):
        clock = Clock()
        clock.clock.sec = int(now)
        clock.clock.nanosec = round((now - int(now)) * 1e9)
        self.pubs["clock"].publish(clock)
        deadline = time.monotonic() + 0.2
        while self.filter._now() + 1e-8 < now and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.001)
        odom = Odometry()
        odom.header.stamp = clock.clock
        odom.twist.twist.linear.x = speed
        self.pubs["odom"].publish(odom)
        for name, active in (("obstacle", obstacle), ("signal", red)):
            factors = PlanningFactorArray()
            factors.header.stamp = clock.clock
            if active:
                factor = PlanningFactor()
                factor.module = "obstacle_stop" if name == "obstacle" else "traffic_light"
                factor.behavior = PlanningFactor.STOP
                point = ControlPoint()
                point.distance = 1.0
                factor.control_points = [point]
                factors.factors = [factor]
            self.pubs[name].publish(factors)
        light = TrafficLightGroup()
        light.traffic_light_group_id = light_id
        element = TrafficLightElement()
        element.shape = TrafficLightElement.CIRCLE
        element.color = TrafficLightElement.RED if red else TrafficLightElement.GREEN
        element.status = TrafficLightElement.SOLID_ON
        light.elements = [element]
        self.pubs["light"].publish(light)
        rtc = CooperateStatusArray()
        rtc.stamp = clock.clock
        if running:
            status = CooperateStatus()
            status.state.type = State.RUNNING
            rtc.statuses = [status]
        self.pubs["rtc"].publish(rtc)
        self.spin(0.004)
        raw = PredictedObjects()
        raw.header.stamp = clock.clock
        raw.header.frame_id = "map"
        for identifier in ids:
            obj = PredictedObject()
            obj.object_id.uuid = [identifier] * 16
            obj.existence_probability = 0.8
            obj.kinematics.initial_pose_with_covariance.pose.position.x = identifier * 10.0
            obj.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0
            obj.shape.dimensions.x = 4.5
            obj.shape.dimensions.y = 2.0
            obj.shape.dimensions.z = 2.0
            raw.objects.append(obj)
        count = len(self.output)
        self.pubs["objects"].publish(raw)
        deadline = time.monotonic() + 0.3
        while len(self.output) == count and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.001)
        self.spin(0.002)
        self.assertGreater(len(self.output), count, "filtered topic stopped publishing")
        return raw, self.output[-1]

    def sequence(self, start, end, **kwargs):
        result = None
        for tenth in range(round(start * 10), round(end * 10) + 1):
            result = self.tick(tenth / 10, **kwargs)
        return result

    def test_rtc_running_but_stopped_filters_at_ten_seconds(self):
        self.sequence(100, 106.9)
        _, output = self.sequence(107, 109.9, running=True)
        self.assertEqual(len(output.objects), 2)
        raw, output = self.sequence(110, 110.2, running=True)
        self.assertEqual(len(output.objects), 0)
        self.assertEqual(self.filter.last_event["reason"], "obstacle_stop_10s")
        self.assertEqual(len(self.raw_received[-1].objects), len(raw.objects))
        _, output = self.tick(110.3, ids=(1, 2, 3), running=True)
        self.assertEqual([obj.object_id.uuid[0] for obj in output.objects], [3])

    def test_red_twenty_seconds_keeps_signal_input_untouched(self):
        _, output = self.sequence(100, 119.9, red=True)
        self.assertEqual(len(output.objects), 2)
        _, output = self.sequence(120, 120.2, red=True)
        self.assertEqual(len(output.objects), 0)
        self.assertEqual(self.filter.last_event["reason"], "standstill_20s")
        self.assertTrue(self.filter._observation(self.filter._now()).signal_wait)
        for publisher in self.filter.publishers:
            self.assertNotIn("traffic", publisher.topic_name)
            self.assertNotIn("trajectory", publisher.topic_name)
            self.assertNotIn("cooperate_commands", publisher.topic_name)

    def test_actual_red_green_red_messages(self):
        self.sequence(100, 102, red=True)
        self.sequence(102.1, 105, red=False)
        _, output = self.sequence(105.1, 105.3, red=True)
        self.assertEqual(len(output.objects), 0)
        self.assertEqual(self.filter.last_event["reason"], "red_green_red_within_10s")

    def test_disabled_is_byte_identical_and_clears_exclusion(self):
        self.sequence(100, 110.2)
        self.assertTrue(self.filter.policy.ignored)
        result = self.filter.set_parameters([Parameter("enabled", value=False)])
        self.assertTrue(result[0].successful)
        raw, output = self.tick(110.3)
        self.assertEqual(serialize_message(raw), serialize_message(output))
        self.assertFalse(self.filter.policy.ignored)

    def test_moving_is_byte_identical_and_new_route_clears_ignore(self):
        raw, output = self.sequence(100, 101, speed=5.0)
        self.assertEqual(serialize_message(raw), serialize_message(output))
        self.sequence(101.1, 111.3)
        self.assertTrue(self.filter.policy.ignored)
        self.route.uuid.uuid = [51] * 16
        self.pubs["route"].publish(self.route)
        self.spin(0.01)
        raw, output = self.tick(111.4)
        self.assertEqual(serialize_message(raw), serialize_message(output))

    def test_manual_mode_does_not_exclude_after_twenty_seconds(self):
        self.mode.mode = OperationModeState.LOCAL
        self.pubs["mode"].publish(self.mode)
        self.spin(0.01)
        raw, output = self.sequence(100, 120.2)
        self.assertEqual(serialize_message(raw), serialize_message(output))


if __name__ == "__main__":
    unittest.main()
