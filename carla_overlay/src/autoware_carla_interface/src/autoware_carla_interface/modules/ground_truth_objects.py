"""CARLA actor truth as tracked objects, for planning/control simulation only.

This deliberately bypasses detection and tracking, but not map-based prediction.
It never reads future actor motion, ticks the world, or sends vehicle controls.
The existing bridge calls it once per frame with its own /clock timestamp.
"""

import math
import uuid

import carla
import numpy as np
from autoware_perception_msgs.msg import ObjectClassification, Shape, TrackedObject, TrackedObjects
from transforms3d.quaternions import mat2quat


_REFLECT = np.diag([1.0, -1.0, 1.0])


def classify_actor(actor):
    """Map semantic blueprint metadata, retaining UNKNOWN for unsupported actors."""
    if actor.type_id.startswith("walker.pedestrian."):
        return ObjectClassification.PEDESTRIAN
    if not actor.type_id.startswith("vehicle."):
        return None
    base = actor.attributes.get("base_type", "").lower()
    labels = {
        "car": ObjectClassification.CAR,
        "truck": ObjectClassification.TRUCK,
        "bus": ObjectClassification.BUS,
        "motorcycle": ObjectClassification.MOTORCYCLE,
        "bicycle": ObjectClassification.BICYCLE,
        "van": ObjectClassification.CAR,
    }
    if base in labels:
        return labels[base]
    # CARLA 0.9.x blueprints do not all expose base_type.
    if actor.type_id in {"vehicle.bh.crossbike", "vehicle.diamondback.century", "vehicle.gazelle.omafiets"}:
        return ObjectClassification.BICYCLE
    if actor.attributes.get("number_of_wheels") == "2":
        return ObjectClassification.MOTORCYCLE
    if actor.attributes.get("number_of_wheels") == "4":
        return ObjectClassification.CAR
    return ObjectClassification.UNKNOWN


def vector(value):
    return np.array([value.x, value.y, value.z], dtype=float)


def assign_vector(target, value):
    target.x, target.y, target.z = map(float, value)


def covariance(translation, rotation):
    result = [0.0] * 36
    for index in (0, 7, 14):
        result[index] = translation
    for index in (21, 28, 35):
        result[index] = rotation
    return result


class GroundTruthObjects:
    def __init__(self, max_distance=150.0, session_namespace=None):
        if not math.isfinite(max_distance) or max_distance <= 0:
            raise ValueError("ground_truth_range_m must be finite and positive")
        self.max_distance = max_distance
        # IDs are stable during a bridge session, distinct after simulator restart.
        self.namespace = session_namespace or uuid.uuid4()

    def actor_message(self, actor, state):
        label = classify_actor(actor)
        if label is None:
            return None
        bbox = actor.bounding_box
        actor_matrix = np.asarray(state.get_transform().get_matrix(), dtype=float)
        box_matrix = np.asarray(carla.Transform(bbox.location, bbox.rotation).get_matrix(), dtype=float)
        matrix = actor_matrix @ box_matrix
        position = _REFLECT @ matrix[:3, 3]
        rotation = _REFLECT @ matrix[:3, :3] @ _REFLECT
        dimensions = 2.0 * vector(bbox.extent)
        if not np.isfinite(matrix).all() or not np.isfinite(dimensions).all() or np.any(dimensions <= 0):
            return None

        # CARLA linear velocity is in world axes; Autoware twist is in object axes.
        # Angular velocity is an axial vector in degrees/s: det(S) * S reflects it.
        angular_world = -_REFLECT @ np.radians(vector(state.get_angular_velocity()))
        offset_world = _REFLECT @ (actor_matrix[:3, :3] @ vector(bbox.location))
        velocity_world = _REFLECT @ vector(state.get_velocity()) + np.cross(angular_world, offset_world)
        acceleration_world = _REFLECT @ vector(state.get_acceleration())
        if not all(np.isfinite(value).all() for value in (angular_world, velocity_world, acceleration_world)):
            return None

        message = TrackedObject()
        message.object_id.uuid = list(uuid.uuid5(self.namespace, f"{actor.id}:{actor.type_id}").bytes)
        message.existence_probability = 1.0
        message.classification = [ObjectClassification(label=label, probability=1.0)]
        message.shape.type = Shape.BOUNDING_BOX
        assign_vector(message.shape.dimensions, dimensions)
        kinematics = message.kinematics
        pose = kinematics.pose_with_covariance.pose
        assign_vector(pose.position, position)
        qw, qx, qy, qz = mat2quat(rotation)
        pose.orientation.w, pose.orientation.x = float(qw), float(qx)
        pose.orientation.y, pose.orientation.z = float(qy), float(qz)
        kinematics.pose_with_covariance.covariance = covariance(1e-4, 1e-4)
        assign_vector(kinematics.twist_with_covariance.twist.linear, rotation.T @ velocity_world)
        assign_vector(kinematics.twist_with_covariance.twist.angular, rotation.T @ angular_world)
        kinematics.twist_with_covariance.covariance = covariance(1e-4, 1e-4)
        # CARLA reports acceleration at the actor reference. Rotational offset
        # acceleration is not available; retain uncertainty on this unused estimate.
        assign_vector(kinematics.acceleration_with_covariance.accel.linear, rotation.T @ acceleration_world)
        kinematics.acceleration_with_covariance.covariance = covariance(1.0, 100.0)
        kinematics.orientation_availability = kinematics.AVAILABLE
        kinematics.is_stationary = bool(np.linalg.norm(velocity_world) < 0.05)
        return message

    def build(self, actors, snapshot, ego_id, header):
        """Construct a fresh full list; missing/destroyed actors never persist."""
        message = TrackedObjects(header=header)
        ego_state = snapshot.find(ego_id)
        if ego_state is None:
            return message
        ego_position = vector(ego_state.get_transform().location)
        for actor in actors:
            if actor.id == ego_id or classify_actor(actor) is None:
                continue
            state = snapshot.find(actor.id)
            if state is None:
                continue
            if np.linalg.norm(vector(state.get_transform().location) - ego_position) > self.max_distance:
                continue
            try:
                converted = self.actor_message(actor, state)
            except RuntimeError:
                # An actor can be destroyed by ScenarioRunner after this snapshot.
                continue
            if converted is not None:
                message.objects.append(converted)
        return message
