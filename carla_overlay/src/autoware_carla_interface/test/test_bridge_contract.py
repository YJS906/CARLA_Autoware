"""Exercise real bridge callbacks using local fake actors, never a CARLA client.

Run in the CARLA image with --network none; no ROS node is initialized.
"""

import math
import os
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

try:
    import carla
    from autoware_carla_interface import carla_ros
    from autoware_carla_interface.modules.vehicle_adapter import CommandWatchdog, VehicleGeometry
    from tier4_vehicle_msgs.msg import ActuationCommandStamped
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from autoware_vehicle_msgs.msg import GearCommand
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


class Publisher:
    def publish(self, value):
        self.last = value


class FakeActor:
    def __init__(self):
        self.transform = carla.Transform(carla.Location(x=42, y=-5, z=.1), carla.Rotation(yaw=0))
        self.velocity = carla.Vector3D()
        self.angular = carla.Vector3D()
        self.control = carla.VehicleControl()

    def get_transform(self):
        return self.transform

    def set_transform(self, transform):
        self.transform = transform

    def get_velocity(self):
        return self.velocity

    def get_angular_velocity(self):
        return self.angular

    def get_wheel_steer_angle(self, _):
        return 5.0

    def get_control(self):
        return self.control

    def get_light_state(self):
        return carla.VehicleLightState.NONE


@unittest.skipUnless(ROS_AVAILABLE, "Run in the ROS/CARLA image, with --network none")
class BridgeContractTest(unittest.TestCase):
    def setUp(self):
        self.bridge = carla_ros.carla_ros2_interface.__new__(carla_ros.carla_ros2_interface)
        self.bridge._initialize_instance_variables()
        self.bridge.ego_actor = FakeActor()
        self.bridge.vehicle_geometry = VehicleGeometry((-1.4, 0, -.04), 2.82, math.radians(70))
        self.bridge.physics_control = NS(steering_curve=[NS(x=0, y=1), NS(x=20, y=.9), NS(x=60, y=.8)])
        self.bridge.command_watchdog = CommandWatchdog(.5)
        self.bridge.param_values = {"command_timeout_sec": .5}
        self.bridge.logger = NS(error=lambda _: None, warning=lambda _: None)
        self.bridge.timestamp = 100.0
        self.clock = patch.object(carla_ros.time, "monotonic", return_value=200.0)
        self.clock.start()
        self.addCleanup(self.clock.stop)

    def command(self, steer=.2, throttle=.3, brake=0, stamp=100):
        message = ActuationCommandStamped()
        message.header.stamp.sec = stamp
        message.actuation.steer_cmd = steer
        message.actuation.accel_cmd = throttle
        message.actuation.brake_cmd = brake
        self.bridge.control_callback(message)

    def test_callback_steering_and_watchdog_brake(self):
        # At 20km/h, correct normalized control should produce +0.2 rad ROS.
        self.bridge.ego_actor.velocity = carla.Vector3D(x=20 / 3.6)
        self.bridge.applied_gear = 2
        self.bridge.requested_gear = 2
        self.command()
        output = self.bridge.safe_vehicle_control()
        self.assertAlmostEqual(-output.steer * .9 * math.radians(70), .2, places=6)
        self.assertAlmostEqual(output.throttle, .3, places=6)
        self.assertEqual(output.brake, 0)
        with patch.object(carla_ros.time, "monotonic", return_value=200.6):
            output = self.bridge.safe_vehicle_control()
        self.assertEqual(output.throttle, 0)
        self.assertEqual(output.brake, 1)

    def test_gear_reverse_neutral_park_and_moving_interlock(self):
        self.bridge.applied_gear = 2
        self.bridge.ego_actor.velocity = carla.Vector3D(x=3)
        self.command()
        self.bridge.gear_callback(NS(command=GearCommand.REVERSE))
        output = self.bridge.safe_vehicle_control()
        self.assertFalse(output.reverse)
        self.assertEqual(output.throttle, 0)
        self.assertEqual(output.brake, 1)
        self.bridge.ego_actor.velocity = carla.Vector3D()
        output = self.bridge.safe_vehicle_control()
        self.assertTrue(output.reverse)
        self.assertEqual(output.gear, -1)
        self.bridge.gear_callback(NS(command=GearCommand.NEUTRAL))
        output = self.bridge.safe_vehicle_control()
        self.assertTrue(output.manual_gear_shift)
        self.assertEqual(output.gear, 0)
        self.assertEqual(output.throttle, 0)
        self.bridge.gear_callback(NS(command=GearCommand.PARK))
        output = self.bridge.safe_vehicle_control()
        self.assertTrue(output.hand_brake)
        self.assertEqual(output.brake, 1)

    def test_stale_and_nonfinite_commands_cannot_release_brakes(self):
        self.bridge.requested_gear = 2
        self.command(stamp=98)
        self.assertEqual(self.bridge.safe_vehicle_control().brake, 1)
        self.command(steer=float("nan"))
        self.assertEqual(self.bridge.safe_vehicle_control().brake, 1)

    def test_velocity_report_uses_ros_rear_axle_twist_and_actual_gear(self):
        for name in ("vel_state", "steering_state", "ctrl_mode", "gear_state", "actuation_status",
                     "turn_indicators_state", "hazard_lights_state"):
            setattr(self.bridge, "pub_" + name, Publisher())
        self.bridge.ego_actor.velocity = carla.Vector3D(x=10, y=1.4)
        self.bridge.ego_actor.angular = carla.Vector3D(z=math.degrees(1))
        self.bridge.ego_actor.control = carla.VehicleControl(reverse=True, gear=-1)
        self.bridge.ego_status()
        velocity = self.bridge.pub_vel_state.last
        self.assertAlmostEqual(velocity.longitudinal_velocity, 10)
        self.assertAlmostEqual(velocity.lateral_velocity, 0, places=6)
        self.assertAlmostEqual(velocity.heading_rate, -1, places=6)
        self.assertEqual(self.bridge.pub_gear_state.last.report, 20)
        self.assertAlmostEqual(self.bridge.pub_actuation_status.last.status.steer_status,
                               -math.radians(5), places=6)

    def test_initialpose_is_rear_axle_and_does_not_mutate_ros_message(self):
        message = PoseWithCovarianceStamped()
        message.pose.pose.position.x = 100.0
        message.pose.pose.position.y = 20.0
        message.pose.pose.position.z = 0.0
        message.pose.pose.orientation.w = 1.0
        self.bridge.initialpose_callback(message)
        actor = self.bridge.ego_actor.get_transform()
        self.assertAlmostEqual(actor.location.x, 101.4, places=4)
        self.assertAlmostEqual(actor.location.y, -20.0, places=4)
        self.assertAlmostEqual(actor.location.z, 2.04, places=4)
        self.assertEqual(message.pose.pose.position.z, 0.0)
        self.assertEqual(self.bridge.current_control.brake, 1)

    def test_loader_matches_scan_joint_and_reflects_side_camera(self):
        import yaml
        from autoware_carla_interface.modules.sensor_kit_loader import SensorKitLoader
        root = Path(__file__).resolve().parents[1]
        repo = root.parents[2]
        config_root = Path(os.environ.get("CARLA_TEST_CONFIG_ROOT") or (
            repo / "config/carla" if (repo / "config/carla").is_dir() else "/opt/carla-config"
        ))
        calibration = yaml.safe_load((config_root / "sensors/sensor_kit_calibration.yaml").read_text())
        mapping = yaml.safe_load((root / "config/sensor_mapping_light_weight.yaml").read_text())
        loader = SensorKitLoader()
        loader.base_link_origin_carla = (-1.39923624, -.00039074, -.03824741)
        name = "velodyne_top_base_link"
        config = loader._create_sensor_config(name, mapping["sensor_mappings"][name],
                                              calibration["sensor_kit_base_link"][name])
        self.assertAlmostEqual(config.transform["x"], -.39)
        self.assertAlmostEqual(config.transform["z"], 1.84)


if __name__ == "__main__":
    unittest.main()
