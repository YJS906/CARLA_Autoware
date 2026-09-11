# Copyright 2026 selfcar contributors
# SPDX-License-Identifier: Apache-2.0

"""Object exclusion policy; it never publishes a signal, trajectory or RTC command."""

from dataclasses import fields
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult

from autoware_adapi_v1_msgs.msg import OperationModeState, RouteState
from autoware_internal_debug_msgs.msg import StringStamped
from autoware_internal_planning_msgs.msg import PlanningFactor, PlanningFactorArray
from autoware_perception_msgs.msg import PredictedObjects, TrafficLightElement, TrafficLightGroup
from autoware_planning_msgs.msg import LaneletRoute
from nav_msgs.msg import Odometry
from tier4_rtc_msgs.msg import CooperateStatusArray, State

from .policy import Observation, RecoveryPolicy, Settings


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def has_stop(message, maximum_distance, module):
    return any(
        factor.module == module
        and factor.behavior == PlanningFactor.STOP
        and any(
            math.isfinite(point.distance)
            and point.distance <= maximum_distance
            and math.isfinite(point.velocity)
            and abs(point.velocity) < 1e-3
            for point in factor.control_points
        )
        for factor in message.factors
    )


class ObstacleTimeoutReplan(Node):
    def __init__(self, **kwargs):
        super().__init__("obstacle_timeout_replan", **kwargs)
        self.declare_parameter("enabled", False)
        for field in fields(Settings):
            self.declare_parameter(field.name, getattr(Settings(), field.name))
        self.declare_parameter("stop_distance", 2.0)
        self.declare_parameter("signal_queue_distance", 100.0)
        self.add_on_set_parameters_callback(self._validate_parameters)
        self.policy = RecoveryPolicy(self._settings())
        self.messages = {}
        self.route_key = None
        self.last_status = -math.inf
        self.last_event = None
        self.running_modules = {}
        self.subscriptions_ = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.publisher = self.create_publisher(PredictedObjects, "~/output/objects", qos)
        self.excluded_publisher = self.create_publisher(PredictedObjects, "~/excluded_objects", qos)
        self.status_publisher = self.create_publisher(StringStamped, "~/status", qos)
        self.subscriptions_.append(self.create_subscription(
            PredictedObjects, "~/input/objects", self._on_objects, qos))
        self.subscriptions_.append(self.create_subscription(
            LaneletRoute, "/planning/mission_planning/route", self._on_route, durable))
        for key, topic, msg_type, profile in (
            ("odom", "/localization/kinematic_state", Odometry, best_effort),
            ("mode", "/system/operation_mode/state", OperationModeState, durable),
            ("route_state", "/api/routing/state", RouteState, durable),
            ("obstacle", "/planning/planning_factors/obstacle_stop", PlanningFactorArray, qos),
            ("signal", "/planning/planning_factors/traffic_light", PlanningFactorArray, qos),
            ("light", "/planning/scenario_planning/lane_driving/behavior_planning/"
             "debug/traffic_signal", TrafficLightGroup, qos),
        ):
            self.subscriptions_.append(self.create_subscription(
                msg_type, topic, lambda msg, key=key: self._remember(key, msg), profile))
        for module in (
            "lane_change_left", "lane_change_right",
            "avoidance_by_lane_change_left", "avoidance_by_lane_change_right",
        ):
            self.subscriptions_.append(self.create_subscription(
                CooperateStatusArray, "/planning/cooperate_status/" + module,
                lambda msg, module=module: self._on_rtc(module, msg), qos))
        self.timer = self.create_timer(0.05, self._on_timer)

    def _validate_parameters(self, parameters):
        positive = {field.name for field in fields(Settings)} | {"signal_queue_distance"}
        nonnegative = {"stopped_velocity", "stop_distance"}
        for parameter in parameters:
            if parameter.name not in positive | nonnegative:
                continue
            value = parameter.value
            if (
                type(value) not in (float, int)
                or not math.isfinite(value)
                or value < 0.0
                or (parameter.name not in nonnegative and value == 0.0)
            ):
                return SetParametersResult(successful=False, reason=parameter.name + " is invalid")
        return SetParametersResult(successful=True)

    def _settings(self):
        return Settings(**{
            field.name: self.get_parameter(field.name).value for field in fields(Settings)
        })

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _remember(self, key, message):
        self.messages[key] = (message, self._now())

    def _on_route(self, message):
        key = (bytes(message.uuid.uuid), message.header.stamp.sec, message.header.stamp.nanosec)
        if self.route_key != key:
            self.policy.reset()
            self.last_event = None
        self.route_key = key

    def _on_rtc(self, module, message):
        # Observability only: an RTC acknowledgement/RUNNING label cannot reset a standstill.
        self.running_modules[module] = (
            any(status.state.type == State.RUNNING for status in message.statuses), self._now())

    def _fresh(self, key, now, stamped=True):
        stored = self.messages.get(key)
        if stored is None:
            return None
        message, received = stored
        timeout = self.policy.settings.message_timeout
        if now < received or now - received > timeout:
            return None
        if stamped:
            age = now - stamp_seconds(message.header.stamp)
            if not math.isfinite(age) or age < -0.1 or age > timeout:
                return None
        return message

    def _enabled(self):
        # This experiment is wired only into the VTD simulation launch.
        return (self.get_parameter("enabled").value
                and self.get_parameter("use_sim_time").value)

    def _active(self):
        mode = self.messages.get("mode", (None,))[0]
        route = self.messages.get("route_state", (None,))[0]
        return (
            self._enabled() and self.route_key is not None
            and mode is not None and mode.mode == OperationModeState.AUTONOMOUS
            and mode.is_autoware_control_enabled and not mode.is_in_transition
            and route is not None and route.state == RouteState.SET
        )

    def _observation(self, now):
        self.policy.settings = self._settings()
        objects = self._fresh("objects", now)
        odom = self._fresh("odom", now)
        obstacle = self._fresh("obstacle", now)
        signal = self._fresh("signal", now)
        light = self._fresh("light", now, stamped=False)
        signal_wait = None if signal is None else has_stop(
            signal, self.get_parameter("signal_queue_distance").value, "traffic_light")
        signal_id = None
        phase = "unknown"
        if signal is not None and light is not None and light.traffic_light_group_id != 0:
            signal_id = light.traffic_light_group_id
            # Use the actual path's selected light, never another light elsewhere on the map.
            solid = [element for element in light.elements
                     if element.status in (TrafficLightElement.SOLID_ON, TrafficLightElement.UNKNOWN)]
            if signal_wait and any(element.color == TrafficLightElement.RED for element in solid):
                phase = "red"
            elif not signal_wait and any(
                element.color == TrafficLightElement.GREEN for element in solid
            ):
                phase = "green"
            elif not light.elements or all(
                element.color == TrafficLightElement.UNKNOWN for element in light.elements
            ):
                phase = "unknown"
            else:
                phase = "other"
        speed = None
        if odom is not None:
            velocity = odom.twist.twist.linear
            speed = math.hypot(velocity.x, velocity.y)
            if not math.isfinite(speed):
                speed = None
        return Observation(
            now=now, active=self._active(), speed=speed,
            objects=None if objects is None else tuple(
                bytes(obj.object_id.uuid) for obj in objects.objects),
            obstacle_stop=obstacle is not None and has_stop(
                obstacle, self.get_parameter("stop_distance").value, "obstacle_stop"),
            signal_wait=signal_wait, signal_id=signal_id, signal_phase=phase,
        )

    def _step(self):
        now = self._now()
        obs = self._observation(now)
        event = self.policy.update(obs)
        if event is not None:
            self.last_event = {
                "time": now, "reason": event.reason,
                "newly_ignored": [uid.hex() for uid in event.object_ids],
            }
            self.get_logger().warning(
                "[obstacle_timeout_replan] " + json.dumps(self.last_event)
                + "; traffic-light stops retained")
        if event is not None or now < self.last_status or now - self.last_status >= 1.0:
            status = StringStamped()
            status.stamp = self.get_clock().now().to_msg()
            status.data = json.dumps({
                "active": obs.active, "speed": obs.speed,
                "obstacle_stop": obs.obstacle_stop, "signal_wait": obs.signal_wait,
                "signal_id": obs.signal_id, "signal_phase": obs.signal_phase,
                "obstacle_seconds": self.policy.elapsed(now, self.policy.obstacle_since),
                "standstill_seconds": self.policy.elapsed(now, self.policy.stopped_since),
                "ignored_uuids": [uid.hex() for uid in sorted(self.policy.ignored)],
                "running_modules": [name for name, (running, received) in self.running_modules.items()
                                    if running and 0 <= now - received <= 0.5],
                "last_event": self.last_event,
            }, allow_nan=False)
            self.status_publisher.publish(status)
            self.last_status = now
        return event

    def _publish_objects(self, message):
        excluded = PredictedObjects()
        excluded.header = message.header
        if self._enabled():
            excluded.objects = [obj for obj in message.objects
                                if bytes(obj.object_id.uuid) in self.policy.ignored]
        self.excluded_publisher.publish(excluded)
        if not self.policy.ignored or not self._enabled():
            self.publisher.publish(message)
            return
        output = PredictedObjects()
        output.header = message.header
        output.objects = [obj for obj in message.objects
                          if bytes(obj.object_id.uuid) not in self.policy.ignored]
        self.publisher.publish(output)

    def _on_objects(self, message):
        self._remember("objects", message)
        self._step()
        self._publish_objects(message)

    def _on_timer(self):
        if self._step() is not None:
            message = self._fresh("objects", self._now())
            if message is not None:
                self._publish_objects(message)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleTimeoutReplan()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
