"""Offline bridge regression tests; no ROS graph or CARLA server is touched."""
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import carla_traffic_signal_bridge as bridge
from builtin_interfaces.msg import Time


class TrafficSignalTests(unittest.TestCase):
    def fixture(self):
        actor = SimpleNamespace(is_alive=True, get_state=lambda: bridge.carla.TrafficLightState.Green)
        world = SimpleNamespace(id=7, get_snapshot=lambda: SimpleNamespace(frame=10))
        node = SimpleNamespace(
            manifest={"groups": [{"group_id": 123, "opendrive_id": "100"}]},
            client=SimpleNamespace(get_world=lambda: world), world_id=7, frame=10,
            frame_seen_at=100, valid_world=True, actors={"100": actor},
            next_refresh=1000, stale_timeout=3, warn=Mock(), refresh=Mock(),
            publisher=SimpleNamespace(publish=Mock()),
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=10))))
        return node, world, actor

    def publish(self, node, now=100):
        with patch.object(bridge.time, "monotonic", return_value=now):
            bridge.CarlaTrafficSignalBridge.publish_states(node)
        return node.publisher.publish.call_args.args[0].traffic_light_groups[0].elements[0].color

    def test_real_green_and_amber_only(self):
        node, _, actor = self.fixture()
        self.assertEqual(self.publish(node), bridge.TrafficLightElement.GREEN)
        actor.get_state = lambda: bridge.carla.TrafficLightState.Yellow
        self.assertEqual(self.publish(node), bridge.TrafficLightElement.AMBER)

    def test_unknown_off_missing_and_destroyed_are_red(self):
        node, _, actor = self.fixture()
        for state in (bridge.carla.TrafficLightState.Unknown, bridge.carla.TrafficLightState.Off):
            actor.get_state = lambda: state
            self.assertEqual(self.publish(node), bridge.TrafficLightElement.RED)
        actor.is_alive = False
        self.assertEqual(self.publish(node), bridge.TrafficLightElement.RED)
        node.actors.clear()
        self.assertEqual(self.publish(node), bridge.TrafficLightElement.RED)

    def test_frozen_snapshot_uses_wall_time(self):
        node, _, _ = self.fixture()
        self.assertEqual(self.publish(node, now=104), bridge.TrafficLightElement.RED)

    def test_rpc_failure_clears_cached_green(self):
        node, _, _ = self.fixture()
        node.client.get_world = Mock(side_effect=RuntimeError("server disappeared"))
        self.assertEqual(self.publish(node), bridge.TrafficLightElement.RED)
        self.assertFalse(node.valid_world)
        self.assertEqual(node.actors, {})

    def test_world_replacement_clears_previous_actor(self):
        node, world, _ = self.fixture()
        world.id = 8
        self.assertEqual(self.publish(node), bridge.TrafficLightElement.RED)
        node.refresh.assert_called_once_with(world)
        self.assertEqual(node.actors, {})

    def test_frame_reset_reacquires_world_without_id_change(self):
        node, world, _ = self.fixture()
        world.get_snapshot = lambda: SimpleNamespace(frame=1)
        self.assertEqual(self.publish(node), bridge.TrafficLightElement.RED)
        node.refresh.assert_called_once()

    def test_refresh_duplicate_ids_are_not_available(self):
        node, world, actor = self.fixture()
        node.manifest["live_xodr_sha256"] = bridge.sha256(b"test-map")
        actor.get_opendrive_id = lambda: "100"
        world.get_map = lambda: SimpleNamespace(name="Carla/Maps/Town05_Opt", to_opendrive=lambda: "test-map")
        world.get_actors = lambda: SimpleNamespace(filter=lambda _: [actor, actor])
        bridge.CarlaTrafficSignalBridge.refresh(node, world)
        self.assertNotIn("100", node.actors)
        self.assertEqual(self.publish(node), bridge.TrafficLightElement.RED)

    def test_refresh_wrong_map_hash_clears_green(self):
        node, world, _ = self.fixture()
        node.manifest["live_xodr_sha256"] = "different"
        world.get_map = lambda: SimpleNamespace(name="Carla/Maps/Town05_Opt", to_opendrive=lambda: "test-map")
        bridge.CarlaTrafficSignalBridge.refresh(node, world)
        self.assertFalse(node.valid_world)
        self.assertEqual(node.actors, {})

    def test_manifest_rejects_mismatched_map(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            (directory/"lanelet2_map.osm").write_bytes(b"map-one")
            path = directory/"carla_traffic_signals.json"
            data = {"schema": 1, "map_name": "Town05", "map_sha256": bridge.sha256(b"map-one"),
                    "groups": [{"group_id": 123, "opendrive_id": "100"}]}
            path.write_text(json.dumps(data))
            bridge.validate_manifest(path)
            (directory/"lanelet2_map.osm").write_bytes(b"map-two")
            with self.assertRaises(ValueError):
                bridge.validate_manifest(path)

    def test_timer_uses_steady_clock_when_ros_clock_is_stopped(self):
        args = SimpleNamespace(manifest=Path("unused"), host="unused", port=2000,
                               timeout=1, topic="unused", stale_timeout=3, rate=10)
        with patch.object(bridge.Node, "__init__", return_value=None), \
                patch.object(bridge.Node, "create_publisher"), \
                patch.object(bridge.Node, "create_timer") as timer, \
                patch.object(bridge.Node, "get_logger"), \
                patch.object(bridge, "validate_manifest", return_value={"groups": [{}]}), \
                patch.object(bridge.carla, "Client"):
            bridge.CarlaTrafficSignalBridge(args)
        self.assertEqual(timer.call_args.kwargs["clock"].clock_type, bridge.ClockType.STEADY_TIME)


if __name__ == "__main__":
    unittest.main()
