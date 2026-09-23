"""Offline safety/lifecycle tests: no CARLA server connection or driving."""

from collections import deque
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import carla_city_traffic as traffic


def actor(actor_id, role="autopilot", type_id="vehicle.test"):
    return SimpleNamespace(id=actor_id, attributes={"role_name": role}, type_id=type_id,
                           get_location=lambda: SimpleNamespace(x=0, y=0, z=0))


class CityTrafficTests(unittest.TestCase):
    def config(self, value):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            path.write_text(json.dumps(value))
            return traffic.read_config(path)

    def fixture(self):
        city = traffic.CityTraffic.__new__(traffic.CityTraffic)
        city.config = dict(traffic.DEFAULTS)
        city.vehicles, city.walkers, city.controllers = set(), {}, set()
        city.configured_external = set()
        city.previous_walker_positions = {}
        city.errors = deque(maxlen=30)
        city.world, city.client = Mock(), Mock()
        city.configure_vehicle = Mock()
        city.spawn_vehicles, city.spawn_walkers = Mock(), Mock()
        return city

    def test_config_rejects_nonfinite_negative_oversized_and_fractional_counts(self):
        for value in ({"vehicles": -1}, {"walkers": 801}, {"vehicles": 1.2},
                      {"vehicles": True}, {"following_distance_m": float("nan")},
                      {"auto_lane_change": "false"}, {"unknown_setting": 1},
                      {"walker_speed_min_mps": 1.7, "walker_speed_max_mps": 1}):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.config(value)
        self.assertEqual(self.config({"vehicles": 200, "walkers": 350})["walkers"], 350)

    def test_ego_and_user_scenario_roles_are_not_background(self):
        for role in ("ego_vehicle", "hero", "ego", "validation_fixture", "event_cut_in", "scenario"):
            self.assertFalse(traffic.ordinary_vehicle(actor(1, role)))
        self.assertTrue(traffic.ordinary_vehicle(actor(1, "autopilot")))

    def test_shrink_never_removes_external_vehicle_or_pedestrian(self):
        city = self.fixture()
        city.config.update(vehicles=0, walkers=0)
        city.vehicles = {21}
        city.walkers = {31: {"controller": 41}}
        city.controllers = {41}
        city.destroy = Mock()
        city.reconcile([actor(1, "ego_vehicle"), actor(2), actor(21, traffic.ROLE),
                        actor(3, type_id="walker.pedestrian.test"),
                        actor(31, traffic.ROLE, "walker.pedestrian.test")])
        removed = [value for call in city.destroy.call_args_list for value in call.args[0]]
        self.assertEqual(set(removed), {21, 31, 41})
        city.spawn_vehicles.assert_not_called()
        city.spawn_walkers.assert_not_called()

    def test_existing_population_counts_toward_target_without_spawning(self):
        city = self.fixture()
        city.config.update(vehicles=1, walkers=1)
        city.reconcile([actor(1, "ego_vehicle"), actor(2),
                        actor(3, type_id="walker.pedestrian.test")])
        city.spawn_vehicles.assert_not_called()
        city.spawn_walkers.assert_not_called()
        city.configure_vehicle.assert_called_once()

    def test_missing_ego_suspends_population_changes(self):
        city = self.fixture()
        city.reconcile([actor(2)])
        self.assertEqual(city.status, "waiting_for_ego")
        city.spawn_vehicles.assert_not_called()
        city.spawn_walkers.assert_not_called()
        city.configure_vehicle.assert_not_called()

    def test_destroy_filters_unowned_ids_and_never_requests_tick(self):
        city = self.fixture()
        city.vehicles = {21}
        city.world.get_actor.return_value = None
        city.client.apply_batch_sync.return_value = [SimpleNamespace(error="")]
        with patch.object(traffic.carla.command, "DestroyActor", side_effect=lambda value: value):
            city.destroy([1, 21, 99])
        city.client.apply_batch_sync.assert_called_once_with([21], False)
        self.assertEqual(city.vehicles, set())

    def test_orphan_walker_is_reclaimed_when_controller_disappears(self):
        city = self.fixture()
        city.walkers = {31: {"controller": 41}}
        city.controllers = {41}
        city.world.get_actor.side_effect = lambda value: actor(31, type_id="walker.pedestrian.test") if value == 31 else None
        city.destroy = Mock()
        city.maintain_walkers(100)
        city.destroy.assert_called_once_with([41, 31])

    def test_controller_stops_before_deletion(self):
        city = self.fixture()
        city.controllers = {41}
        actions = []
        controller = SimpleNamespace(stop=lambda: actions.append("stop"))
        city.world.get_actor.return_value = controller
        city.client.apply_batch_sync.side_effect = lambda batch, tick: actions.append((batch, tick)) or [SimpleNamespace(error="")]
        with patch.object(traffic.carla.command, "DestroyActor", side_effect=lambda value: value):
            city.destroy([41])
        self.assertEqual(actions, ["stop", ([41], False)])

    def test_walker_motion_uses_snapshot_displacement_when_velocity_reports_zero(self):
        city = self.fixture()
        walker = actor(31, traffic.ROLE, "walker.pedestrian.test")
        walker.get_velocity = lambda: SimpleNamespace(x=0, y=0, z=0)
        def snapshot(elapsed, x):
            sample = SimpleNamespace(get_transform=lambda: SimpleNamespace(location=SimpleNamespace(x=x, y=0, z=0)))
            return SimpleNamespace(timestamp=SimpleNamespace(elapsed_seconds=elapsed), find=lambda actor_id: sample)
        self.assertEqual(city.walker_motion([walker], snapshot(10, 0)), [])
        self.assertEqual(traffic.speed(walker), 0)
        self.assertEqual(city.walker_motion([walker], snapshot(12, 2.4)), [1.2])

    def test_destroy_not_found_is_idempotent_despite_cached_actor(self):
        city = self.fixture()
        city.vehicles = {21}
        city.world.get_actor.return_value = actor(21)
        city.client.apply_batch_sync.return_value = [SimpleNamespace(error="unable to destroy actor: not found")]
        with patch.object(traffic.carla.command, "DestroyActor", side_effect=lambda value: value):
            city.destroy([21])
        self.assertEqual(city.vehicles, set())
        self.assertEqual(list(city.errors), [])

    def test_cleanup_observes_external_tick_before_reporting_final_counts(self):
        city = self.fixture()
        city.world_id = 7
        city.client.get_world.return_value.id = 7
        city.vehicles = {21}
        city.rates = [(1, 100, 5)]
        city.destroy = Mock()
        city.write_state = Mock()
        actions = []
        city.world.wait_for_tick.side_effect = lambda timeout: actions.append("observe_tick")
        city.world.get_actors.side_effect = lambda: actions.append("fresh_actors") or []
        city.write_state.side_effect = lambda: actions.append("write_state")
        city.cleanup()
        self.assertEqual(actions, ["observe_tick", "fresh_actors", "write_state"])
        self.assertEqual(city.vehicles, set())
        self.assertEqual(city.status, "stopped")

    def test_population_ownership_handles_delayed_actor_registration(self):
        summary = traffic.ownership_summary(range(149), range(150))
        self.assertEqual(summary, dict(nonowned=0, owned_visible=149, owned_not_visible=[149]))
        summary = traffic.ownership_summary([1, 2, 3], [2, 3, 4])
        self.assertEqual(summary, dict(nonowned=1, owned_visible=2, owned_not_visible=[4]))


if __name__ == "__main__":
    unittest.main()
