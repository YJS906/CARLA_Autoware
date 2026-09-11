# Copyright 2026 selfcar contributors
# SPDX-License-Identifier: Apache-2.0

import math
import struct
import unittest

import numpy as np

from autoware_perception_msgs.msg import DetectedObject, DetectedObjects, PredictedObject, PredictedObjects, Shape
from geometry_msgs.msg import Transform
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2, PointField

from selfcar_obstacle_timeout_replan.cloud_geometry import associate_detection_boxes, filter_cloud, rotation


def box(uid=1, x=5.0, y=0.0, yaw=0.0):
    obj = PredictedObject()
    obj.object_id.uuid = [uid] * 16
    obj.shape.type = Shape.BOUNDING_BOX
    obj.shape.dimensions.x, obj.shape.dimensions.y, obj.shape.dimensions.z = 4.0, 2.0, 2.0
    pose = obj.kinematics.initial_pose_with_covariance.pose
    pose.position.x, pose.position.y = x, y
    pose.orientation.z, pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return obj


def cloud(points, bigendian=False):
    msg = PointCloud2()
    msg.header.frame_id = "map"
    msg.header.stamp.sec = 100
    msg.height, msg.width = 1, len(points)
    msg.point_step, msg.row_step = 16, len(points) * 16
    msg.is_bigendian = bigendian
    msg.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(("x", "y", "z", "intensity"))]
    msg.data = b"".join(struct.pack((">" if bigendian else "<") + "ffff", *p) for p in points)
    return msg


class TestCloudGeometry(unittest.TestCase):
    def setUp(self):
        self.objects = PredictedObjects()
        self.objects.header.frame_id = "map"
        self.objects.header.stamp.sec = 100
        self.objects.objects = [box()]
        self.ignored = {bytes([1] * 16)}
        self.transform = Transform()
        self.transform.rotation.w = 1.0

    def apply(self, msg, **kwargs):
        return filter_cloud(msg, self.objects, self.ignored, self.transform, **kwargs)

    def test_only_matching_wireframe_removed(self):
        # Rear bottom edge and rear vertical edge match. Face-interior and distant points do not.
        msg = cloud([(3, 0, 0, 11), (3, 1, 1, 12), (5, 0, 2, 13), (50, 0, 0, 14)])
        out, count = self.apply(msg)
        self.assertEqual(count, 2)
        self.assertEqual(bytes(out.data), bytes(msg.data)[32:])
        self.assertEqual(out.header, msg.header)
        self.assertEqual(out.fields, msg.fields)

    def test_new_uuid_protects_overlapping_points(self):
        other = box(uid=2, x=3)
        other.shape.dimensions.x = 0.5
        self.objects.objects.append(other)
        msg = cloud([(3, 0, 0, 11)])
        out, count = self.apply(msg)
        self.assertEqual(count, 0)
        self.assertIs(out, msg)

    def test_new_uuid_with_same_geometry_is_not_excluded(self):
        self.objects.objects = [box(uid=2)]
        msg = cloud([(3, 0, 0, 11)])
        out, count = self.apply(msg)
        self.assertEqual(count, 0)
        self.assertEqual(serialize_message(out), serialize_message(msg))

    def test_rotated_box(self):
        self.objects.objects = [box(yaw=math.pi / 2)]
        msg = cloud([(5, -2, 0, 11), (3, 0, 0, 12)])
        out, count = self.apply(msg)
        self.assertEqual(count, 1)
        self.assertEqual(bytes(out.data), bytes(msg.data)[16:])

    def test_transform_to_base_frame(self):
        # Map x=3 -> base y=2 under a 90-degree rotation followed by translation (0,-1,0).
        self.transform.rotation.z = math.sqrt(0.5)
        self.transform.rotation.w = math.sqrt(0.5)
        self.transform.translation.y = -1.0
        msg = cloud([(0, 2, 0, 11), (10, 2, 0, 12)])
        out, count = self.apply(msg)
        self.assertEqual(count, 1)
        self.assertEqual(bytes(out.data), bytes(msg.data)[16:])

    def test_height_convention_and_vertical_limits(self):
        msg = cloud([(3, 0, 2, 11), (3, 0, 3, 12)])
        out, count = self.apply(msg)
        self.assertEqual(count, 1)
        self.assertEqual(bytes(out.data), bytes(msg.data)[16:])
        self.objects.objects[0].kinematics.initial_pose_with_covariance.pose.position.z = 1.0
        out2, count2 = self.apply(msg, z_is_ground=False)
        self.assertEqual((bytes(out2.data), count2), (bytes(out.data), count))

    def test_tilted_ego_tf_matches_bridge_cloud_and_preserves_new_uuid(self):
        # Ego is at map (100,200), facing +Y. The bridge puts these boxes at
        # base (10,0) and (20,0), regardless of the roll/pitch reported in TF.
        self.objects.objects = [box(x=100.0, y=210.0, yaw=math.pi / 2),
                                box(uid=2, x=100.0, y=220.0, yaw=math.pi / 2)]
        msg = cloud([(8, 0, 2, 1), (8, 1, 1, 2), (18, 0, 2, 3), (40, 40, 4, 4)])
        msg.header.frame_id = "base_link"
        for roll_deg, pitch_deg in ((0.0, -2.6), (4.0, 3.0)):
            with self.subTest(roll=roll_deg, pitch=pitch_deg):
                roll, pitch, yaw = math.radians(roll_deg), math.radians(pitch_deg), math.pi / 2
                cr, sr = math.cos(roll / 2), math.sin(roll / 2)
                cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
                cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
                # Inverse of the ego's full 3D pose, as supplied by TF.
                q = self.transform.rotation
                q.x = -(sr * cp * cy - cr * sp * sy)
                q.y = -(cr * sp * cy + sr * cp * sy)
                q.z = -(cr * cp * sy - sr * sp * cy)
                q.w = cr * cp * cy + sr * sp * sy
                offset = -(rotation(q) @ np.array([100.0, 200.0, 0.0]))
                t = self.transform.translation
                t.x, t.y, t.z = map(float, offset)
                output, removed = self.apply(msg)
                self.assertEqual(removed, 2)
                self.assertEqual(bytes(output.data), bytes(msg.data)[32:])

    def test_nonfinite_points_preserved(self):
        msg = cloud([(3, 0, 0, 11), (float('nan'), 0, 0, 12)])
        out, count = self.apply(msg)
        self.assertEqual(count, 1)
        self.assertEqual(bytes(out.data), bytes(msg.data)[16:])

    def test_empty_exclusion_is_byte_identical(self):
        self.ignored.clear()
        msg = cloud([(3, 0, 0, 11)])
        out, count = self.apply(msg)
        self.assertEqual(count, 0)
        self.assertIs(out, msg)

    def test_big_endian_preserves_all_retained_fields(self):
        msg = cloud([(3, 0, 0, 123.25), (50, 0, 0, 9.75)], bigendian=True)
        out, count = self.apply(msg)
        self.assertEqual(count, 1)
        self.assertTrue(out.is_bigendian)
        self.assertEqual(bytes(out.data), bytes(msg.data)[16:])

    def test_organized_cloud_row_padding(self):
        msg = cloud([(3, 0, 0, 1), (50, 0, 0, 2), (3, 1, 1, 3), (60, 0, 0, 4)])
        raw = bytes(msg.data)
        msg.height, msg.width, msg.row_step = 2, 2, 40
        msg.data = raw[:32] + b'padding!' + raw[32:] + b'padding!'
        out, count = self.apply(msg)
        self.assertEqual((count, out.height, out.width, out.row_step), (2, 1, 2, 32))
        self.assertEqual(bytes(out.data), raw[16:32] + raw[48:64])

    def test_invalid_geometry_is_rejected_before_any_removal(self):
        self.objects.objects.append(box(uid=2))
        self.objects.objects[-1].shape.dimensions.x = 0.0
        with self.assertRaises(ValueError):
            self.apply(cloud([(3, 0, 0, 1)]))

    def test_motion_compensation_uses_cloud_timestamp(self):
        self.objects.objects[0].kinematics.initial_twist_with_covariance.twist.linear.x = 2.0
        msg = cloud([(3.2, 0, 0, 1), (3, 0, 0, 2)])
        msg.header.stamp.nanosec = 100000000
        out, count = self.apply(msg)
        self.assertEqual(count, 1)
        self.assertEqual(bytes(out.data), bytes(msg.data)[16:])

    def detections(self):
        result = DetectedObjects()
        result.header = self.objects.header
        for tracked in self.objects.objects:
            obj = DetectedObject()
            obj.kinematics.pose_with_covariance = tracked.kinematics.initial_pose_with_covariance
            obj.shape = tracked.shape
            result.objects.append(obj)
        return result

    def test_changed_tracker_shape_uses_original_detector_box(self):
        detected = self.detections()
        # Replace the track's shape; detection keeps the original bridge box.
        replacement = Shape()
        replacement.type = Shape.CYLINDER
        replacement.dimensions.x = replacement.dimensions.y = 2.0
        replacement.dimensions.z = 2.0
        self.objects.objects[0].shape = replacement
        geometry, matched = associate_detection_boxes(self.objects, detected, self.ignored)
        msg = cloud([(3, 0, 0, 1)])
        output, count = filter_cloud(msg, geometry, matched, self.transform)
        self.assertEqual((count, output.width), (1, 0))

    def test_two_tracks_near_one_detection_are_ambiguous(self):
        detected = self.detections()
        self.objects.objects.append(box(uid=2, x=5.1))
        geometry, matched = associate_detection_boxes(self.objects, detected, self.ignored)
        self.assertFalse(matched)
        self.assertEqual(bytes(geometry.objects[0].object_id.uuid), bytes(16))

    def test_two_detections_near_one_track_are_ambiguous(self):
        detected = self.detections()
        detected.objects.append(detected.objects[0])
        _, matched = associate_detection_boxes(self.objects, detected, self.ignored)
        self.assertFalse(matched)

    def test_new_detection_without_track_is_protected(self):
        detected = self.detections()
        new = DetectedObject()
        new.kinematics.pose_with_covariance = box(uid=2, x=3).kinematics.initial_pose_with_covariance
        new.shape = box().shape
        detected.objects.append(new)
        geometry, matched = associate_detection_boxes(self.objects, detected, self.ignored)
        msg = cloud([(3, 0, 0, 1)])
        output, count = filter_cloud(msg, geometry, matched, self.transform)
        self.assertEqual(count, 0)
        self.assertIs(output, msg)


if __name__ == "__main__":
    unittest.main()
