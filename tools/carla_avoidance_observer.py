#!/usr/bin/env python3
"""Read-only avoidance diagnostics. Start before the parent begins its fixture test.

Prints OBSERVER_READY to stderr, JSON result to stdout on completion. No clients,
publishers, simulator connection, setters, parameter changes or vehicle actions.
Example after sourcing ROS/Autoware: python3 script.py --duration 65 > result.json
"""
import argparse
import collections
import json
import math
import signal
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from tier4_planning_msgs.msg import AvoidanceDebugMsgArray
from tier4_rtc_msgs.msg import AutoModeStatus, CooperateStatusArray
from autoware_planning_msgs.msg import Path
from autoware_internal_planning_msgs.msg import PlanningFactorArray
from autoware_adapi_v1_msgs.msg import RouteState, OperationModeState
from visualization_msgs.msg import MarkerArray
from rosgraph_msgs.msg import Clock


def clean(value):
    if isinstance(value, dict):
        return {key: clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=65.0)
    args = parser.parse_args()
    if not 1 <= args.duration <= 180:
        parser.error("duration must be 1..180 seconds")
    rclpy.init(args=[])
    node = rclpy.create_node("carla_readonly_avoidance_observer")
    start = time.monotonic()
    state = {"sim_time": None, "route_state": None, "operation_mode": None}
    result = {"start_wall_time": time.time(), "requested_duration": args.duration,
              "debug_fields": AvoidanceDebugMsgArray.get_fields_and_field_types(),
              "topics": {}, "failure_reasons": {}}
    reasons = collections.Counter()
    stream = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = []

    def callback(topic, kind):
        def receive(msg):
            row = result["topics"][topic]
            row["count"] += 1
            payload = message_to_ordereddict(msg)
            meaningful = True
            if kind == "debug":
                meaningful = bool(msg.avoidance_info)
                for item in msg.avoidance_info:
                    reasons[item.failed_reason or "<empty>"] += 1
            elif kind == "rtc":
                meaningful = bool(msg.statuses)
            elif kind == "path":
                meaningful = bool(msg.points)
                xs = [p.pose.position.x for p in msg.points]
                ys = [p.pose.position.y for p in msg.points]
                payload = {"header": payload["header"], "point_count": len(msg.points),
                           "bounds_count": [len(msg.left_bound), len(msg.right_bound)],
                           "xy_range": [min(xs), max(xs), min(ys), max(ys)] if xs else None,
                           "first": payload["points"][0] if xs else None,
                           "last": payload["points"][-1] if xs else None}
            elif kind == "factor":
                meaningful = bool(msg.factors)
            elif kind == "marker":
                text_markers = [{"namespace": m.ns, "id": m.id, "text": m.text}
                                for m in msg.markers if m.text]
                meaningful = bool(text_markers)
                payload = {"text_markers": text_markers}
            event = {"elapsed_wall": time.monotonic() - start, **state,
                     "message": clean(payload)}
            row["last"] = event
            if meaningful:
                row["nonempty_count"] += 1
                row["samples"].append(event)
        return receive

    def add(cls, topic, kind, qos=stream):
        result["topics"][topic] = {"type": cls.__name__, "count": 0,
                                   "nonempty_count": 0, "samples": [], "last": None}
        subscriptions.append(node.create_subscription(cls, topic, callback(topic, kind), qos))

    def clock(msg):
        state["sim_time"] = msg.clock.sec + msg.clock.nanosec * 1e-9
    subscriptions.append(node.create_subscription(Clock, "/clock", clock, stream))
    subscriptions.append(node.create_subscription(RouteState, "/api/routing/state",
                         lambda m: state.update(route_state=m.state), latched))
    subscriptions.append(node.create_subscription(OperationModeState, "/api/operation_mode/state",
                         lambda m: state.update(operation_mode=m.mode), latched))
    base = "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/"
    add(AvoidanceDebugMsgArray, base + "debug/avoidance_debug_message_array", "debug")
    for module in ("static_obstacle_avoidance", "avoidance_by_lane_change"):
        add(Path, "/planning/path_candidate/" + module, "path")
        add(PlanningFactorArray, "/planning/planning_factors/" + module, "factor")
        add(MarkerArray, base + "info/" + module, "marker")
        for side in ("left", "right"):
            add(CooperateStatusArray, "/planning/cooperate_status/" + module + "_" + side, "rtc")
            add(AutoModeStatus, "/planning/auto_mode_status/" + module + "_" + side,
                "auto_mode", latched)
    print("OBSERVER_READY read-only; wait for test start", file=sys.stderr, flush=True)
    try:
        while rclpy.ok() and time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        result["interrupted"] = True
    finally:
        result["duration_wall"] = time.monotonic() - start
        result["failure_reasons"] = dict(reasons)
        result["final_state"] = state
        print(json.dumps(clean(result), allow_nan=False), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
