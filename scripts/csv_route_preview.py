#!/usr/bin/env python3
"""Durable, visualization-only CSV preview. Never creates a route API client."""

from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import time
from typing import Any


PREVIEW_TOPICS = tuple(
    "/debug/csv/" + name
    for name in (
        "raw_checkpoints",
        "candidate_lanelets",
        "selected_lanelets",
        "corrected_checkpoints",
    )
)


def map_digest(path: str | Path) -> str:
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def save_preview_state(
    path: str | Path, metadata: dict[str, Any], messages: dict[str, Any],
    *, only_if_absent: bool = False, expected_saved_at_ns: int | None = None,
) -> int | None:
    """Retain the preview atomically; bootstrap must not overwrite an explicit CSV load."""
    from rosidl_runtime_py.convert import message_to_ordereddict

    target = Path(path)
    payload = {
        "version": 1,
        "saved_at_ns": time.time_ns(),
        "metadata": metadata,
        "topics": {
            topic: message_to_ordereddict(messages[topic]) for topic in PREVIEW_TOPICS
        },
    }
    # The host launcher owns/creates the private state directory. Do not silently
    # fall back to container-local storage, which disappears at the next restart.
    temporary = None
    # Lock the shared directory inode, not a file replaced by os.replace(). This
    # also works between a root bridge and a host-UID setter without lock-file
    # ownership problems. All preview writers use this same lock.
    directory_fd = os.open(target.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        fcntl.flock(directory_fd, fcntl.LOCK_EX)
        if only_if_absent and target.exists():
            return None
        if expected_saved_at_ns is not None:
            if not target.exists():
                return None
            current = json.loads(target.read_text(encoding="utf-8"))
            if current.get("saved_at_ns") != expected_saved_at_ns:
                return None
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=target.parent, delete=False
        ) as stream:
            temporary = stream.name
            json.dump(payload, stream, ensure_ascii=False, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
            # A root-run Autoware setter and the host-UID preview worker share this
            # file; its parent remains private (0700) on the host.
            os.fchmod(stream.fileno(), 0o644)
        os.replace(temporary, target)
        temporary = None
        os.fsync(directory_fd)
        return payload["saved_at_ns"]
    finally:
        if temporary is not None:
            os.unlink(temporary)
        os.close(directory_fd)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state", required=True, type=Path)
    parser.add_argument("--map", required=True, type=Path, dest="map_path")
    parser.add_argument("--csv", type=Path, help="refresh the preview from this CSV on every start")
    args = parser.parse_args()

    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosidl_runtime_py.set_message import set_message_fields
    from visualization_msgs.msg import MarkerArray

    class PersistentPreview(Node):
        def __init__(self) -> None:
            super().__init__("csv_route_preview", start_parameter_services=False)
            qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.publishers_by_topic = {
                topic: self.create_publisher(MarkerArray, topic, qos)
                for topic in PREVIEW_TOPICS
            }
            self.messages: dict[str, Any] = {}
            self.state_version = None
            self.last_error = None
            self.refresh_on_startup = args.csv is not None
            # Continue republishing when simulation time stops or jumps backwards.
            self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.timer = self.create_timer(1.0, self.tick, clock=self.wall_clock)
            self.get_logger().info(f"CSV preview only; persistent state: {args.state}")
            self.tick()

        def bootstrap(self, *, replace_existing: bool = False) -> None:
            # Snapshot before parsing/matching so an explicit load performed during
            # startup wins over this automatic refresh.
            expected_saved_at_ns = None
            if replace_existing and args.state.exists():
                expected_saved_at_ns = json.loads(
                    args.state.read_text(encoding="utf-8")
                )["saved_at_ns"]
            # Import only parsing, map matching and marker rendering. Never invoke
            # the setter main(), create a route client, or wait for Autoware.
            from set_route_from_csv import (
                LaneletMapMatcher, MapMatchError, RouteCsvError,
                RoutePreviewPublisher, build_argument_parser, load_route_csv,
            )

            if args.csv is None:
                raise ValueError("no saved CSV preview and no bootstrap --csv was configured")
            try:
                points = load_route_csv(args.csv)
            except RouteCsvError as error:
                raise ValueError(str(error)) from error
            metadata = {
                "source_csv": os.environ.get("AUTOWARE_CSV_SOURCE_PATH", str(args.csv)),
                "csv_text": args.csv.read_text(encoding="utf-8-sig"),
                # Provenance only; saved visualization remains valid across map-file updates.
                "map_sha256": map_digest(args.map_path),
                "frame_id": "map",
            }
            renderer = RoutePreviewPublisher(self, "map", state_path=str(args.state))
            raw_messages = renderer.render_messages(points)
            version = save_preview_state(
                args.state, metadata, raw_messages,
                only_if_absent=expected_saved_at_ns is None,
                expected_saved_at_ns=expected_saved_at_ns,
            )
            if version is None:
                return  # An explicit load won the race; restore that saved preview.
            self.get_logger().info(
                f"Refreshed CSV preview from {metadata['source_csv']} "
                f"({len(points)} checkpoints); visualization only, no route request"
            )
            # Publish original points before map matching, and retain them on failure.
            for topic, array in raw_messages.items():
                self.publishers_by_topic[topic].publish(array)
            defaults = build_argument_parser().parse_args([str(args.csv), "--preview-only"])
            try:
                matcher = LaneletMapMatcher(
                    args.map_path,
                    candidate_radius=defaults.candidate_radius,
                    candidate_count=defaults.candidate_count,
                    direction_threshold=math.radians(defaults.direction_threshold),
                    vehicle_width=defaults.vehicle_width,
                    goal_margin=defaults.goal_margin,
                )
                result = matcher.match(points)
            except MapMatchError as error:
                self.get_logger().warning(f"CSV map matching failed; retaining raw markers: {error}")
                return
            saved = save_preview_state(
                args.state, metadata, renderer.render_messages(points, result),
                expected_saved_at_ns=version,
            )
            if saved is not None:
                self.get_logger().info("Saved matched CSV preview; no route request")

        def tick(self) -> None:
            try:
                if self.refresh_on_startup:
                    self.refresh_on_startup = False
                    try:
                        self.bootstrap(replace_existing=True)
                    except (OSError, ValueError, KeyError, TypeError) as error:
                        self.get_logger().warning(
                            f"CSV startup refresh failed; restoring saved preview: {error}"
                        )
                elif not args.state.exists():
                    self.state_version = None
                    self.bootstrap()
                # After startup, restore explicit loads/overrides from the saved
                # state. Do not keep replacing them with the startup CSV.
                stat = args.state.stat()
                version = (stat.st_ino, stat.st_mtime_ns, stat.st_size)
                if version != self.state_version:
                    payload = json.loads(args.state.read_text(encoding="utf-8"))
                    if payload.get("version") != 1:
                        raise ValueError("unsupported CSV preview state version")
                    if set(payload["topics"]) != set(PREVIEW_TOPICS):
                        raise ValueError("CSV preview must contain all four marker arrays")
                    messages = {}
                    for topic in PREVIEW_TOPICS:
                        array = MarkerArray()
                        set_message_fields(array, payload["topics"][topic])
                        for marker in array.markers:
                            if not marker.header.frame_id:
                                raise ValueError("CSV marker has an empty coordinate frame")
                            # Static annotations use the latest transform, not the
                            # previous run's wall/simulation timestamp.
                            marker.header.stamp.sec = 0
                            marker.header.stamp.nanosec = 0
                            marker.lifetime.sec = 0
                            marker.lifetime.nanosec = 0
                        messages[topic] = array
                    self.messages = messages
                    self.state_version = version
                    self.last_error = None
                    count = sum(len(array.markers) - 1 for array in messages.values())
                    self.get_logger().info(
                        f"Restored {count} CSV markers from "
                        f"{payload['metadata'].get('source_csv', 'saved CSV')}; no route request"
                    )
            except (OSError, ValueError, KeyError, TypeError, AttributeError, AssertionError) as error:
                message = str(error)
                if message != self.last_error:
                    self.get_logger().error(f"Cannot reload CSV preview: {message}")
                    self.last_error = message
                # A partial/invalid replacement must not erase the last good preview.
            for topic, array in self.messages.items():
                self.publishers_by_topic[topic].publish(array)

    rclpy.init()
    node = PersistentPreview()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
