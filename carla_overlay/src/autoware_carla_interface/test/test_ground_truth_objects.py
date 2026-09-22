"""Validate real CARLA transforms and ROS messages without a simulator or ROS node."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import uuid

import carla
import numpy as np
from std_msgs.msg import Header
from transforms3d.quaternions import quat2mat

MODULE = Path(__file__).resolve().parents[1] / "src/autoware_carla_interface/modules/ground_truth_objects.py"
spec = importlib.util.spec_from_file_location("ground_truth_objects_tested", MODULE)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def actor(actor_id=2, type_id="vehicle.toyota.prius", offset=(0, 0, 0), box_rotation=None):
    box = carla.BoundingBox(carla.Location(*offset), carla.Vector3D(2, 1, .75))
    box.rotation = box_rotation or carla.Rotation()
    return NS(id=actor_id, type_id=type_id, bounding_box=box, attributes={"number_of_wheels": "4"})


def state(x=10, y=20, z=1, yaw=0, roll=0, pitch=0, velocity=(0, 0, 0), angular=(0, 0, 0)):
    return NS(get_transform=lambda: carla.Transform(carla.Location(x, y, z), carla.Rotation(pitch, yaw, roll)),
              get_velocity=lambda: carla.Vector3D(*velocity),
              get_acceleration=lambda: carla.Vector3D(),
              get_angular_velocity=lambda: carla.Vector3D(*angular))


class GroundTruthTest(unittest.TestCase):
    def setUp(self):
        self.converter = m.GroundTruthObjects(150, uuid.UUID(int=1))
        self.header = Header(frame_id="map")
        self.header.stamp.sec = 123
        self.header.stamp.nanosec = 50000000

    def test_classifications_and_actor_exclusion(self):
        self.assertEqual(m.classify_actor(actor(type_id="walker.pedestrian.0001")), m.ObjectClassification.PEDESTRIAN)
        self.assertEqual(m.classify_actor(actor()), m.ObjectClassification.CAR)
        self.assertEqual(m.classify_actor(actor(type_id="vehicle.gazelle.omafiets")), m.ObjectClassification.BICYCLE)
        truck = actor(); truck.attributes = {"base_type": "truck"}
        self.assertEqual(m.classify_actor(truck), m.ObjectClassification.TRUCK)
        self.assertIsNone(m.classify_actor(actor(type_id="sensor.camera.rgb")))
        self.assertIsNone(m.classify_actor(actor(type_id="controller.ai.walker")))

    def test_world_reflection_box_center_and_local_velocity(self):
        obj = self.converter.actor_message(actor(offset=(1, .5, 1)), state(yaw=90, velocity=(0, 5, 0)))
        pose = obj.kinematics.pose_with_covariance.pose
        np.testing.assert_allclose([pose.position.x, pose.position.y, pose.position.z], [9.5, -21, 2], atol=1e-6)
        self.assertAlmostEqual(pose.orientation.z, -math.sqrt(.5), places=6)
        self.assertAlmostEqual(obj.kinematics.twist_with_covariance.twist.linear.x, 5, places=6)
        self.assertAlmostEqual(obj.kinematics.twist_with_covariance.twist.linear.y, 0, places=6)
        self.assertEqual(obj.shape.dimensions.x, 4)
        self.assertFalse(obj.kinematics.is_stationary)

    def test_rotated_bounding_box_matches_vertices(self):
        actor_ = actor(offset=(1, -.3, .7), box_rotation=carla.Rotation(pitch=9, yaw=21, roll=-14))
        state_ = state(yaw=41, roll=4, pitch=7)
        obj = self.converter.actor_message(actor_, state_)
        pose = obj.kinematics.pose_with_covariance.pose
        q = pose.orientation
        rotation = quat2mat([q.w, q.x, q.y, q.z])
        # Reflection of a nontrivial full 3-D box is a proper ROS rotation.
        np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
        self.assertAlmostEqual(np.linalg.det(rotation), 1, places=6)
        vertices = actor_.bounding_box.get_world_vertices(state_.get_transform())
        local = [rotation.T @ (m._REFLECT @ m.vector(v) - m.vector(pose.position)) for v in vertices]
        for vertex in local:
            np.testing.assert_allclose(np.abs(vertex), [2, 1, .75], atol=2e-6)

    def test_angular_units_and_reference_offset_velocity(self):
        obj = self.converter.actor_message(actor(offset=(1, 0, 0)), state(angular=(0, 0, 90)))
        twist = obj.kinematics.twist_with_covariance.twist
        self.assertAlmostEqual(twist.angular.z, -math.pi / 2)
        self.assertAlmostEqual(twist.linear.y, -math.pi / 2)

    def test_fresh_list_stable_id_and_no_stale_objects(self):
        states = {1: state(0, 0), 2: state(10, 0), 3: state(200, 0), 4: state(20, 0)}
        snapshot = NS(find=states.get)
        actors = [actor(1), actor(2), actor(3), actor(4, "sensor.lidar.ray_cast")]
        first = self.converter.build(actors, snapshot, 1, self.header)
        second = self.converter.build(actors, snapshot, 1, self.header)
        self.assertEqual(len(first.objects), 1)
        self.assertEqual(first.objects[0].object_id, second.objects[0].object_id)
        self.assertEqual(first.header, self.header)
        del states[2]
        self.assertEqual(self.converter.build(actors, snapshot, 1, self.header).objects, [])
        del states[1]
        self.assertEqual(self.converter.build(actors, snapshot, 1, self.header).objects, [])
        other = m.GroundTruthObjects(150, uuid.UUID(int=2)).actor_message(actor(2), state())
        self.assertNotEqual(first.objects[0].object_id, other.object_id)

    def test_invalid_geometry_excluded_and_range_rejected(self):
        invalid = actor(); invalid.bounding_box.extent.x = 0
        self.assertIsNone(self.converter.actor_message(invalid, state()))
        self.assertIsNone(self.converter.actor_message(actor(), state(velocity=(float('nan'), 0, 0))))
        for value in (0, -1, float('inf'), float('nan')):
            with self.assertRaises(ValueError):
                m.GroundTruthObjects(value)


if __name__ == "__main__":
    unittest.main()
