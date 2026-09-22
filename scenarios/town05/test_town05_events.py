"""Offline map/trigger checks; these do not replace a CARLA physics run."""

import contextlib
import io
import os
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import carla
import py_trees
import town05_events as events


class Actor:
    def __init__(self, transform, speed=0):
        self.transform = transform
        self.speed = speed
        self.is_alive = True
        self.controls = []
        self.autopilot = False

    def get_location(self):
        return self.transform.location

    def get_transform(self):
        return self.transform

    def get_velocity(self):
        return self.transform.get_forward_vector() * self.speed

    def apply_control(self, control):
        self.controls.append(control)

    def set_autopilot(self, enabled, port):
        self.autopilot = enabled


class Ego(Actor):
    def apply_control(self, control):
        raise AssertionError("An event must never control ego")

    def set_autopilot(self, enabled, port):
        raise AssertionError("An event must never enable ego autopilot")


class TrafficManager:
    def get_port(self):
        return 8000

    def auto_lane_change(self, *args):
        pass

    def ignore_lights_percentage(self, *args):
        pass

    def ignore_signs_percentage(self, *args):
        pass


class Planner:
    def __init__(self, actor, opt_dict):
        self.options = opt_dict
        self.finished = False

    def set_global_plan(self, plan, **kwargs):
        self.plan = plan

    def done(self):
        return self.finished

    def run_step(self, debug=False):
        return carla.VehicleControl(throttle=0.2)


class EventTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root = Path(os.environ.get("CARLA_ROOT", "/home/a/CARLA/0.9.16"))
        xodr = root / "CarlaUE4/Content/Carla/Maps/OpenDrive/Town05_Opt.xodr"
        if not xodr.is_file():
            raise unittest.SkipTest(f"Installed Town05 XODR required: {xodr}")
        cls.wmap = carla.Map("Town05_Opt", xodr.read_text())
        scenario = ET.parse(Path(__file__).with_name("town05_hazards.xml")).getroot().find("scenario")
        cls.ped_settings = dict(scenario.find("pedestrian_event").attrib)
        cls.car_settings = dict(scenario.find("cut_in_event").attrib)

    def setUp(self):
        self.world = SimpleNamespace(get_map=lambda: self.wmap,
                                     get_settings=lambda: SimpleNamespace(fixed_delta_seconds=0.05))
        self.site = events.site_waypoint(self.wmap, self.ped_settings)
        self.ego = Ego(self.site.previous(30)[0].transform, speed=8)
        self.actors = []
        self.silence = contextlib.redirect_stdout(io.StringIO())
        self.silence.__enter__()
        self.addCleanup(self.silence.__exit__, None, None, None)

    def spawn(self, model, transform, role):
        actor = Actor(transform)
        self.actors.append(actor)
        return actor

    def test_default_map_has_sidewalk_and_same_direction_merge(self):
        ped = events.PedestrianEvent(self.world, self.ego, self.ped_settings, self.spawn)
        car = events.CutInEvent(self.world, self.ego, self.car_settings, self.spawn, TrafficManager())
        self.assertEqual(ped.sidewalk.lane_type, carla.LaneType.Sidewalk)
        self.assertEqual(car.direction, "right")
        self.assertEqual(car.site.lane_id, -3)
        self.assertEqual(car.source.lane_id, -2)
        plan = car.merge_plan(car.source)
        self.assertEqual(plan[-1][0].lane_id, car.site.lane_id)
        self.assertTrue(all(not wp.is_junction for wp, _ in plan))

    def test_gate_rejects_stopped_wrong_lane_reverse_and_passed_ego(self):
        self.ego.transform = self.site.previous(20)[0].transform
        self.assertIsNotNone(events.approach_distance(self.ego, self.wmap, self.site, 25))
        self.ego.speed = 0
        self.assertIsNone(events.approach_distance(self.ego, self.wmap, self.site, 25))
        self.ego.speed = 8
        self.ego.transform = self.site.previous(20)[0].get_left_lane().transform
        self.assertIsNone(events.approach_distance(self.ego, self.wmap, self.site, 25))
        self.ego.transform = self.site.previous(20)[0].transform
        self.ego.transform.rotation = carla.Rotation(yaw=self.ego.transform.rotation.yaw + 180)
        self.assertIsNone(events.approach_distance(self.ego, self.wmap, self.site, 25))
        self.ego.transform = self.site.next(2)[0].transform
        self.assertIsNone(events.approach_distance(self.ego, self.wmap, self.site, 25))
        self.ego.transform = self.site.previous(20)[0].transform
        self.ego.transform.location += carla.Location(z=8)
        self.assertIsNone(events.approach_distance(self.ego, self.wmap, self.site, 25))

    def test_pedestrian_walks_then_crosses_once_and_stops(self):
        ped = events.PedestrianEvent(self.world, self.ego, self.ped_settings, self.spawn)
        ped.update()
        self.assertEqual(ped.phase, "walking")
        self.assertAlmostEqual(ped.actor.controls[-1].speed, 1.2, places=5)
        self.ego.transform = self.site.previous(20)[0].transform
        ped.update()
        self.assertEqual(ped.phase, "crossing")
        self.assertAlmostEqual(ped.actor.controls[-1].speed, 3.5, places=5)
        # Ideal planar motion validates endpoint handling without running physics.
        for _ in range(250):
            command = ped.actor.controls[-1]
            step = command.direction * command.speed * 0.05
            ped.actor.transform.location += carla.Location(x=step.x, y=step.y)
            ped.update()
            if ped.phase == "done":
                break
        self.assertEqual(ped.phase, "done")
        self.assertEqual(ped.actor.controls[-1].speed, 0)
        self.assertEqual(ped.update(), py_trees.common.Status.RUNNING)
        self.assertEqual(ped.phase, "done")

    def test_pedestrian_skips_crossing_when_shifted_refuge_is_invalid(self):
        ped = events.PedestrianEvent(self.world, self.ego, self.ped_settings, self.spawn)
        self.ego.transform = self.site.previous(20)[0].transform
        # Represent a map edit whose shifted endpoint falls back onto a driving lane.
        ped.wmap = SimpleNamespace(get_waypoint=lambda location, **kwargs: self.site)
        ped.update()
        self.assertEqual(ped.phase, "skipped")
        self.assertEqual(ped.actor.controls[-1].speed, 0)

    @patch.object(events, "detect_lane_obstacle", return_value=False)
    @patch.object(events, "LocalPlanner", Planner)
    def test_cutin_waits_approaches_merges_once_and_returns_to_traffic(self, _):
        car = events.CutInEvent(self.world, self.ego, self.car_settings, self.spawn, TrafficManager())
        car.update()
        self.assertEqual(car.phase, "waiting")
        self.ego.transform = car.site.previous(30)[0].transform
        car.update()
        self.assertEqual(car.phase, "approaching")
        self.assertAlmostEqual(car.planner.options["target_speed"], 12)
        self.ego.transform = car.site.previous(12)[0].transform
        car.update()
        self.assertEqual(car.phase, "merging")
        self.assertAlmostEqual(car.planner.options["target_speed"], 25)
        car.actor.transform = car.site.next(12)[0].transform
        car.update()
        self.assertEqual(car.phase, "done")
        self.assertTrue(car.actor.autopilot)
        self.assertEqual(car.update(), py_trees.common.Status.RUNNING)

    @patch.object(events, "detect_lane_obstacle", return_value=False)
    @patch.object(events, "LocalPlanner", Planner)
    def test_cutin_does_not_merge_without_distance_trigger(self, _):
        car = events.CutInEvent(self.world, self.ego, self.car_settings, self.spawn, TrafficManager())
        self.ego.transform = car.site.previous(30)[0].transform
        car.update()
        car.planner.finished = True
        car.update()
        self.assertEqual(car.phase, "done")
        self.assertTrue(car.actor.autopilot)

    @patch.object(events, "LocalPlanner", Planner)
    def test_planner_pid_uses_simulation_timestep(self):
        self.world.get_settings = lambda: SimpleNamespace(fixed_delta_seconds=0.1)
        car = events.CutInEvent(self.world, self.ego, self.car_settings, self.spawn, TrafficManager())
        car.set_plan(car.merge_plan(car.source), car.merge_speed)
        self.assertEqual(car.planner.options["lateral_control_dict"]["dt"], 0.1)
        self.assertEqual(car.planner.options["longitudinal_control_dict"]["dt"], 0.1)

    def test_reject_invalid_values_and_unavailable_adjacent_lane(self):
        for value in ("nan", "inf", "-1"):
            with self.assertRaises(ValueError):
                events.number({"speed": value}, "speed")
        bad = dict(self.car_settings, from_side="right")
        with self.assertRaisesRegex(ValueError, "SAME direction"):
            events.CutInEvent(self.world, self.ego, bad, self.spawn, TrafficManager())


if __name__ == "__main__":
    unittest.main()
