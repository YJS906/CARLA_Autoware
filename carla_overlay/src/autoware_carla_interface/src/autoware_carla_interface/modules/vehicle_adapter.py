"""Unit/frame conversions for CARLA 0.9.16's PhysX vehicles.

These functions have no ROS/CARLA imports so the adapter's contracts can be
tested without starting a simulator or commanding a vehicle.
"""

from dataclasses import dataclass
import math

import numpy as np


def rotation_matrix(roll, pitch, yaw):
    """Right-handed intrinsic XYZ matrix, used for ROS mount extrinsics."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


@dataclass(frozen=True)
class VehicleGeometry:
    # Rear-axle ground projection, expressed in CARLA actor coordinates (m).
    base_offset: tuple
    wheelbase: float
    max_steer_rad: float

    @classmethod
    def from_actor(cls, actor, physics):
        """WheelPhysicsControl.position is world space in centimetres."""
        if len(physics.wheels) != 4:
            raise ValueError("The CARLA adapter requires a four-wheel vehicle")
        inv = np.asarray(actor.get_transform().get_inverse_matrix())
        wheels = []
        for wheel in physics.wheels:
            p = wheel.position
            wheels.append((inv @ np.array([p.x / 100, p.y / 100, p.z / 100, 1]))[:3])
        front = np.mean(wheels[:2], axis=0)
        rear = np.mean(wheels[2:], axis=0)
        wheelbase = float(front[0] - rear[0])
        steer = math.radians(max(w.max_steer_angle for w in physics.wheels[:2]))
        if not 1.0 < wheelbase < 6.0 or not 0.05 < steer < math.pi / 2:
            raise ValueError("Invalid CARLA wheel geometry; actor physics is not ready")
        # Ground plane is the bottom of the chassis bounding box, not wheel
        # centre height, which changes with suspension travel.
        bbox = actor.bounding_box
        ground = bbox.location.z - bbox.extent.z
        return cls((float(rear[0]), float(rear[1]), float(ground)), wheelbase, steer)


def base_to_actor_sensor_transform(transform, base_offset):
    """ROS base_link extrinsics -> CARLA actor extrinsics; angles are radians."""
    return {
        "x": transform["x"] + base_offset[0],
        "y": -transform["y"] + base_offset[1],
        "z": transform["z"] + base_offset[2],
        "roll": math.degrees(transform["roll"]),
        "pitch": -math.degrees(transform["pitch"]),
        "yaw": -math.degrees(transform["yaw"]),
    }


def base_world_position(actor_matrix, base_offset):
    """CARLA-world coordinates of the rear-axle base_link origin."""
    return (np.asarray(actor_matrix) @ np.array([*base_offset, 1.0]))[:3]


def body_velocity_ros(actor_matrix, world_velocity, world_angular_deg, base_offset):
    """Move twist from actor origin to rear axle, then convert handedness/units."""
    rotation = np.asarray(actor_matrix)[:3, :3]
    linear = rotation.T @ np.asarray(world_velocity)
    angular = rotation.T @ np.radians(world_angular_deg)
    linear = linear + np.cross(angular, base_offset)
    return linear * [1, -1, 1], angular * [-1, 1, -1]


def longitudinal_speed(actor_matrix, world_velocity):
    return float((np.asarray(actor_matrix)[:3, :3].T @ np.asarray(world_velocity))[0])


def steering_command(tire_angle_ros, forward_speed_mps, max_steer_rad, curve):
    """ROS radians -> CARLA normalized steer, including PhysX speed scaling.

    CARLA's steering-curve abscissa is km/h. Its NW PhysX implementation calls
    KmHToCmS(Key.Time) when constructing the speed/steer table. The simulator
    applies this factor; invert it here rather than applying it twice.
    """
    values = [tire_angle_ros, forward_speed_mps, max_steer_rad]
    if not all(math.isfinite(v) for v in values) or max_steer_rad <= 0:
        raise ValueError("Non-finite or invalid steering input")
    keys = sorted((float(x), float(y)) for x, y in curve)
    if not keys or any(not math.isfinite(x + y) or y <= 0 for x, y in keys):
        raise ValueError("Invalid CARLA steering curve")
    ratio = float(np.interp(abs(forward_speed_mps) * 3.6, *zip(*keys)))
    return float(np.clip(-tire_angle_ros / (max_steer_rad * ratio), -1.0, 1.0))


def canonical_gear(command):
    """Autoware GearCommand/Report constants (stable ROS message contract)."""
    if command in (1, 22):
        return command
    if command in (20, 21):
        return 20
    if 2 <= command <= 19 or command in (23, 24):
        return 2
    return None


def select_gear(requested, applied, speed_mps):
    """Defer a direction change until stopped; caller applies the service brake."""
    direction_change = requested != applied and (requested in (2, 20) or requested == 22)
    hold = direction_change and abs(speed_mps) > 0.1
    return (applied if hold else requested), hold


class CommandWatchdog:
    """Reject stale commands and stop on command loss in either time domain."""

    def __init__(self, timeout=0.5):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("command_timeout_sec must be positive")
        self.timeout = timeout
        self.received_wall = None
        self.command_stamp = None

    def accept(self, stamp, sim_now, wall_now):
        if (sim_now is None or not all(math.isfinite(v) for v in (stamp, sim_now, wall_now))
                or not -0.1 <= sim_now - stamp <= self.timeout):
            return False
        self.command_stamp = stamp
        self.received_wall = wall_now
        return True

    def expired(self, sim_now, wall_now):
        if self.received_wall is None or self.command_stamp is None or sim_now is None:
            return True
        return not (0 <= wall_now - self.received_wall <= self.timeout
                    and -0.1 <= sim_now - self.command_stamp <= self.timeout)
