"""Regression contracts for the bridge, without a ROS graph or moving actors."""

import importlib.util
import math
import os
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

import numpy as np
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[2]
CONFIG = Path(os.environ.get("CARLA_TEST_CONFIG_ROOT") or (
    REPO / "config/carla" if (REPO / "config/carla").is_dir() else "/opt/carla-config"
))
spec = importlib.util.spec_from_file_location(
    "vehicle_adapter", PACKAGE / "src/autoware_carla_interface/modules/vehicle_adapter.py"
)
adapter = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = adapter
spec.loader.exec_module(adapter)


def actor_matrix(yaw, translation=(42, -71, 0.7)):
    matrix = np.eye(4)
    matrix[:3, :3] = adapter.rotation_matrix(0, 0, yaw)
    matrix[:3, 3] = translation
    return matrix


class VehicleAdapterTest(unittest.TestCase):
    def test_world_centimetre_wheels_are_measured_in_actor_frame(self):
        for yaw in (0, math.pi / 2, math.pi, -0.7):
            matrix = actor_matrix(yaw)
            wheels = []
            for x, y in ((1.42, -0.814), (1.42, 0.814), (-1.399, -0.765), (-1.399, 0.765)):
                world = (matrix @ [x, y, 0.333, 1])[:3] * 100
                wheels.append(NS(position=NS(x=world[0], y=world[1], z=world[2]),
                                 max_steer_angle=70 if x > 0 else 0))
            actor = NS(get_transform=lambda: NS(get_inverse_matrix=lambda: np.linalg.inv(matrix)),
                       bounding_box=NS(location=NS(z=0.724169), extent=NS(z=0.762417)))
            geometry = adapter.VehicleGeometry.from_actor(actor, NS(wheels=wheels))
            np.testing.assert_allclose(geometry.base_offset, [-1.399, 0, -0.038248], atol=1e-8)
            self.assertAlmostEqual(geometry.wheelbase, 2.819)
            self.assertAlmostEqual(geometry.max_steer_rad, math.radians(70))

    def test_sensor_world_and_ros_tf_positions_agree_at_all_headings(self):
        base = (-1.39923624, -0.00039074, -0.03824741)
        calibration = yaml.safe_load((CONFIG / "sensors/sensor_kit_calibration.yaml").read_text())
        sensors = calibration["sensor_kit_base_link"]
        reflection = np.diag([1, -1, 1])
        for yaw in np.linspace(-math.pi, math.pi, 9):
            matrix = actor_matrix(yaw)
            ros_base = reflection @ adapter.base_world_position(matrix, base)
            ros_rotation = reflection @ matrix[:3, :3] @ reflection
            for name, raw_transform in sensors.items():
                transform = dict(raw_transform)
                if name == "velodyne_top_base_link":
                    transform["z"] += 0.0377  # VLP-16 scan joint
                actual = adapter.base_to_actor_sensor_transform(transform, base)
                sensor_world_ros = reflection @ (matrix @ [actual["x"], actual["y"], actual["z"], 1])[:3]
                tf_sensor_world = ros_base + ros_rotation @ [transform[k] for k in ("x", "y", "z")]
                np.testing.assert_allclose(sensor_world_ros, tf_sensor_world, atol=1e-9)
        # Existing physical mounting positions are retained, now with correct ROS TF.
        front = adapter.base_to_actor_sensor_transform(sensors["CAM_FRONT/camera_link"], base)
        self.assertAlmostEqual(front["x"], 0.8)
        self.assertAlmostEqual(front["z"], 1.6)
        left = adapter.base_to_actor_sensor_transform(sensors["CAM_FRONT_LEFT/camera_link"], base)
        self.assertAlmostEqual(left["y"], -0.55)
        self.assertAlmostEqual(left["yaw"], -55.0)

    def test_rear_axle_twist_has_correct_handedness_units_and_lever_arm(self):
        # A right-hand turn in CARLA is negative yaw rate in ROS.
        # Velocity at actor centre is 10m/s forward and 1.4m/s right; a rear
        # axle 1.4m behind has zero lateral slip at +1 rad/s CARLA yaw rate.
        for yaw in np.linspace(-math.pi, math.pi, 13):
            matrix = actor_matrix(yaw)
            world_velocity = matrix[:3, :3] @ [10, 1.4, 0]
            linear, angular = adapter.body_velocity_ros(
                matrix, world_velocity, [0, 0, math.degrees(1)], [-1.4, 0, 0]
            )
            np.testing.assert_allclose(linear, [10, 0, 0], atol=1e-10)
            np.testing.assert_allclose(angular, [0, 0, -1], atol=1e-10)

    def test_lateral_velocity_is_ros_left_positive(self):
        linear, _ = adapter.body_velocity_ros(np.eye(4), [0, 2, 0], [0, 0, 0], [0, 0, 0])
        np.testing.assert_allclose(linear, [0, -2, 0])

    def test_steering_curve_uses_kmh_and_is_heading_independent(self):
        curve = [(0, 1), (20, .9), (60, .8), (120, .7)]
        for yaw in np.linspace(-math.pi, math.pi, 17):
            matrix = actor_matrix(yaw)
            world_velocity = matrix[:3, :3] @ [40 / 3.6, 0, 0]
            speed = adapter.longitudinal_speed(matrix, world_velocity)
            result = adapter.steering_command(.2, speed, math.radians(70), curve)
            # At 40km/h the simulator applies 0.85 of 70 degrees.
            self.assertAlmostEqual(-result * .85 * math.radians(70), .2)
        self.assertEqual(adapter.steering_command(10, 0, 1, curve), -1)
        self.assertEqual(adapter.steering_command(-10, 0, 1, curve), 1)
        with self.assertRaises(ValueError):
            adapter.steering_command(float("nan"), 0, 1, curve)

    def test_watchdog_requires_fresh_commands_and_handles_clock_reset(self):
        watchdog = adapter.CommandWatchdog(.5)
        self.assertTrue(watchdog.expired(10, 100))
        self.assertFalse(watchdog.accept(8, 10, 100))
        self.assertFalse(watchdog.accept(11, 10, 100))
        self.assertTrue(watchdog.accept(10, 10, 100))
        self.assertFalse(watchdog.expired(10.1, 100.1))
        self.assertTrue(watchdog.expired(10.1, 100.6))  # wall-time command loss
        self.assertTrue(watchdog.expired(10.6, 100.1))  # stale sim-time command
        self.assertTrue(watchdog.expired(0, 100.1))    # simulation reset

    def test_gears_and_direction_change_interlock(self):
        self.assertEqual([adapter.canonical_gear(x) for x in (1, 2, 20, 22)], [1, 2, 20, 22])
        self.assertIsNone(adapter.canonical_gear(0))
        self.assertIsNone(adapter.canonical_gear(255))
        self.assertEqual(adapter.select_gear(20, 2, 3), (2, True))
        self.assertEqual(adapter.select_gear(20, 2, 0), (20, False))
        self.assertEqual(adapter.select_gear(22, 2, 3), (2, True))

    def test_prius_vehicle_profile_matches_measured_geometry(self):
        profile = yaml.safe_load((CONFIG / "vehicle/vehicle_info.param.yaml").read_text())["/**"]["ros__parameters"]
        self.assertAlmostEqual(profile["wheel_base"], 2.81918946)
        self.assertAlmostEqual(profile["wheel_base"] + profile["front_overhang"] + profile["rear_overhang"], 4.51352262497)
        self.assertAlmostEqual(profile["wheel_tread"] + profile["left_overhang"] + profile["right_overhang"], 2.00681447983)
        self.assertAlmostEqual(profile["max_steer_angle"], math.radians(70))
        converter = yaml.safe_load((PACKAGE / "config/raw_vehicle_cmd_converter.param.yaml").read_text())["/**"]["ros__parameters"]
        self.assertFalse(converter["convert_steer_cmd"])
        self.assertEqual(converter["max_steer"], profile["max_steer_angle"])
        self.assertEqual(converter["min_steer"], -profile["max_steer_angle"])


if __name__ == "__main__":
    unittest.main()
