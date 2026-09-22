#!/usr/bin/env python3
"""Read native CARLA lights and publish map-linked Autoware signal groups.

This node never ticks CARLA, changes lights, reloads maps or controls vehicles.
OpenDRIVE signal IDs are persisted; runtime actor IDs are reacquired per world.
Unavailable/ambiguous/stale signals are conservatively published RED, never GREEN.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import time

import carla
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from autoware_perception_msgs.msg import TrafficLightElement, TrafficLightGroup, TrafficLightGroupArray


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def validate_manifest(path):
    data = json.loads(path.read_text())
    if data.get("schema") != 1 or data.get("map_name") != "Town05" or not data.get("groups"):
        raise ValueError("Unsupported or empty native Town05 signal manifest")
    ids = [g["group_id"] for g in data["groups"]]
    od_ids = [str(g["opendrive_id"]) for g in data["groups"]]
    if len(ids) != len(set(ids)) or len(od_ids) != len(set(od_ids)):
        raise ValueError("Duplicate signal group or OpenDRIVE ID in manifest")
    map_path = path.parent / "lanelet2_map.osm"
    if sha256(map_path.read_bytes()) != data["map_sha256"]:
        raise ValueError(f"Signal manifest does not match {map_path}; regenerate/reinstall them together")
    return data


def signal_element(state):
    element = TrafficLightElement()
    element.shape = TrafficLightElement.CIRCLE
    element.status = TrafficLightElement.SOLID_ON
    element.confidence = 1.0
    element.color = {
        carla.TrafficLightState.Red: TrafficLightElement.RED,
        carla.TrafficLightState.Yellow: TrafficLightElement.AMBER,
        carla.TrafficLightState.Green: TrafficLightElement.GREEN,
    }.get(state, TrafficLightElement.RED)
    return element


class CarlaTrafficSignalBridge(Node):
    def __init__(self, args):
        super().__init__("carla_traffic_signal_bridge")
        self.manifest = validate_manifest(args.manifest)
        self.client = carla.Client(args.host, args.port)
        self.client.set_timeout(args.timeout)
        self.publisher = self.create_publisher(TrafficLightGroupArray, args.topic, 10)
        self.world_id = None
        self.frame = None
        self.frame_seen_at = time.monotonic()
        self.valid_world = False
        self.actors = {}
        self.next_refresh = 0.0
        self.last_warning = {}
        self.stale_timeout = args.stale_timeout
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1.0/args.rate, self.publish_states, clock=self.wall_clock)
        self.get_logger().info(
            f"Read-only CARLA signals: {len(self.manifest['groups'])} groups -> {args.topic}; "
            "unavailable signals are RED; no simulator ticks or vehicle controls")

    def warn(self, key, message):
        now = time.monotonic()
        if now-self.last_warning.get(key, -100) >= 5:
            self.get_logger().warning(message)
            self.last_warning[key] = now

    def refresh(self, world):
        self.actors = {}
        self.valid_world = False
        wmap = world.get_map()
        if wmap.name.rsplit("/", 1)[-1] not in ("Town05", "Town05_Opt"):
            self.warn("map", f"World {wmap.name} is not native Town05; publishing RED")
            return
        if sha256(wmap.to_opendrive().encode()) != self.manifest["live_xodr_sha256"]:
            self.warn("map_hash", "CARLA OpenDRIVE differs from manifest; publishing RED")
            return
        duplicates = set()
        expected = {str(g["opendrive_id"]) for g in self.manifest["groups"]}
        for actor in world.get_actors().filter("traffic.traffic_light"):
            od_id = str(actor.get_opendrive_id())
            if od_id in self.actors:
                duplicates.add(od_id)
            self.actors[od_id] = actor
        for od_id in duplicates:
            self.actors.pop(od_id, None)
        missing = expected-self.actors.keys()
        extra = self.actors.keys()-expected
        if missing or extra or duplicates:
            self.warn("inventory", f"Signal inventory: missing={sorted(missing)}, "
                      f"unmapped={sorted(extra)}, duplicate={sorted(duplicates)}; "
                      "missing/duplicate groups publish RED")
        self.valid_world = True

    def publish_states(self):
        now = time.monotonic()
        available = False
        try:
            world = self.client.get_world()
            snapshot = world.get_snapshot()
            token = getattr(world, "id", None)
            replaced = token != self.world_id or (self.frame is not None and snapshot.frame < self.frame)
            if replaced:
                self.world_id, self.frame = token, None
                self.actors, self.valid_world = {}, False
                self.next_refresh = 0
            if snapshot.frame != self.frame:
                self.frame, self.frame_seen_at = snapshot.frame, now
            if now >= self.next_refresh:
                self.refresh(world)
                self.next_refresh = now+2
            available = self.valid_world and now-self.frame_seen_at <= self.stale_timeout
            if not available:
                self.warn("stale", "No validated, advancing Town05 snapshot; publishing RED")
        except (RuntimeError, OSError) as error:
            self.valid_world, self.actors, self.next_refresh = False, {}, 0
            self.warn("rpc", f"CARLA read failed: {error}; publishing RED")
        message = TrafficLightGroupArray()
        message.stamp = self.get_clock().now().to_msg()
        for item in self.manifest["groups"]:
            state = None
            actor = self.actors.get(str(item["opendrive_id"])) if available else None
            try:
                if actor is not None and actor.is_alive:
                    state = actor.get_state()
            except RuntimeError as error:
                self.warn("actor", f"Traffic light disappeared: {error}; reacquiring and publishing RED")
                self.next_refresh = 0
            if state not in (carla.TrafficLightState.Red, carla.TrafficLightState.Yellow,
                             carla.TrafficLightState.Green):
                self.warn("unknown", "Missing/Off/Unknown CARLA signal state treated as RED")
            group = TrafficLightGroup()
            group.traffic_light_group_id = item["group_id"]
            group.elements = [signal_element(state)]
            message.traffic_light_groups.append(group)
        self.publisher.publish(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--topic", default="/perception/traffic_light_recognition/traffic_signals")
    parser.add_argument("--rate", type=float, default=10)
    parser.add_argument("--timeout", type=float, default=2)
    parser.add_argument("--stale-timeout", type=float, default=3)
    args, ros_args = parser.parse_known_args()
    if not 1 <= args.rate <= 30 or args.timeout <= 0 or args.stale_timeout <= 0:
        parser.error("Rate must be 1..30Hz; timeout values must be positive")
    rclpy.init(args=ros_args)
    node = None
    try:
        node = CarlaTrafficSignalBridge(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
