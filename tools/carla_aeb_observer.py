#!/usr/bin/env python3
"""Read-only AEB observer for CARLA trials; never ticks, controls, or sets parameters.

Cloud estimates reproduce z crop, voxel centroids, and connected-component size
in a fixed forward corridor, not the complete MPC-footprint crop. AEB's own
markers, RSS and diagnostics are the authoritative runtime decision evidence.
"""
import argparse
import collections
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
from scipy.spatial import cKDTree

from carla_pedestrian_perception_probe import points_xyz


AEB = '/control/autonomous_emergency_braking'
PARAMS = [
    'use_predicted_object_data', 'use_pointcloud_data', 'use_object_velocity_calculation',
    'use_predicted_trajectory', 'use_imu_path', 'check_autoware_state',
    'detection_range_min_height', 'detection_range_max_height_margin', 'vehicle_height',
    'voxel_grid_x', 'voxel_grid_y', 'voxel_grid_z', 'cluster_tolerance',
    'minimum_cluster_size', 'maximum_cluster_size', 'cluster_minimum_height',
    'expand_width', 'path_footprint_extra_margin', 'mpc_prediction_time_horizon',
    'mpc_prediction_time_interval', 'a_ego_min', 'a_obj_min', 't_response',
    'longitudinal_offset_margin', 'publish_debug_markers', 'publish_debug_pointcloud',
]
TOPICS = {
    AEB + '/debug/markers': 'visualization_msgs/msg/MarkerArray',
    AEB + '/virtual_wall': 'visualization_msgs/msg/MarkerArray',
    AEB + '/debug/rss_distance': 'tier4_debug_msgs/msg/Float32Stamped',
    AEB + '/metrics': 'tier4_metric_msgs/msg/MetricArray',
    '/diagnostics': 'diagnostic_msgs/msg/DiagnosticArray',
    '/autoware/state': 'autoware_system_msgs/msg/AutowareState',
    '/vehicle/status/velocity_status': 'autoware_vehicle_msgs/msg/VelocityReport',
    '/control/trajectory_follower/lateral/predicted_trajectory': 'autoware_planning_msgs/msg/Trajectory',
    '/perception/object_recognition/objects': 'autoware_perception_msgs/msg/PredictedObjects',
    '/perception/obstacle_segmentation/pointcloud': 'sensor_msgs/msg/PointCloud2',
    '/rosout': 'rcl_interfaces/msg/Log',
}


def stamp(value):
    return value.sec + value.nanosec * 1e-9


def xyz(value):
    return [value.x, value.y, value.z]


def parameter_value(value):
    fields = {1: 'bool_value', 2: 'integer_value', 3: 'double_value', 4: 'string_value'}
    return getattr(value, fields[value.type]) if value.type in fields else None


def integer(value):
    # ROS `byte` fields such as DiagnosticStatus.level are bytes in Jazzy.
    return int.from_bytes(value, 'little') if isinstance(value, bytes) else int(value)


def json_value(value):
    if isinstance(value, bytes):
        return list(value)
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f'Cannot serialize {type(value).__name__}')


def cloud_estimate(message, params):
    if message.header.frame_id != 'base_link':
        return {'error': 'observer requires base_link point cloud'}
    points = points_xyz(message)
    forward = points[(points[:, 0] >= -2) & (points[:, 0] <= 40) & (np.abs(points[:, 1]) <= 5)]
    clipped = forward[(forward[:, 2] >= params['detection_range_min_height']) &
                      (forward[:, 2] <= params['vehicle_height'] + params['detection_range_max_height_margin'])]
    result = {'finite_points': len(points), 'forward_corridor_points': len(forward),
              'after_z_crop_points': len(clipped), 'clusters': []}
    if len(clipped) == 0:
        result['after_voxel_points'] = 0
        return result
    leaf = np.array([params['voxel_grid_x'], params['voxel_grid_y'], params['voxel_grid_z']])
    _, inverse = np.unique(np.floor(clipped / leaf).astype(np.int64), axis=0, return_inverse=True)
    counts = np.bincount(inverse)
    centroids = np.column_stack([np.bincount(inverse, weights=clipped[:, i]) / counts for i in range(3)])
    result['after_voxel_points'] = len(centroids)
    tree = cKDTree(centroids)
    seen = set()
    for start in range(len(centroids)):
        if start in seen:
            continue
        queue = [start]
        seen.add(start)
        for index in queue:
            for neighbor in tree.query_ball_point(centroids[index], params['cluster_tolerance']):
                if neighbor not in seen:
                    queue.append(neighbor)
                    seen.add(neighbor)
        cluster = centroids[queue]
        # Save small components too: these are evidence of the min-size gate.
        result['clusters'].append({
            'points': len(queue), 'min_xyz': cluster.min(axis=0).tolist(),
            'max_xyz': cluster.max(axis=0).tolist(),
            'accepted_size_height': bool(params['minimum_cluster_size'] <= len(queue) <= params['maximum_cluster_size']
                                         and cluster[:, 2].max() > params['cluster_minimum_height']),
        })
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=90)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Node('carla_aeb_readonly_observer', parameter_overrides=[Parameter('use_sim_time', value=True)])
    report = {'note': __doc__, 'parameters': {}, 'topics': {}, 'events': [], 'errors': collections.Counter()}
    start = time.monotonic()
    client = node.create_client(GetParameters, AEB + '/get_parameters')
    if not client.wait_for_service(timeout_sec=5):
        raise RuntimeError('AEB parameter service unavailable')
    future = client.call_async(GetParameters.Request(names=PARAMS))
    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    if not future.done() or future.result() is None:
        raise RuntimeError('AEB parameter request timed out')
    report['parameters'] = dict(zip(PARAMS, map(parameter_value, future.result().values)))
    last = {}

    def callback(message, topic):
        report['topics'][topic]['messages'] += 1
        if topic == '/diagnostics' and not any(
            'aeb' in s.name.lower() or 'emergency_braking' in s.name.lower()
            for s in message.status
        ):
            return
        now = time.monotonic()
        interval = .19 if topic.endswith('/pointcloud') else .085
        if topic != '/rosout' and now - last.get(topic, 0) < interval:
            return
        last[topic] = now
        event = {'topic': topic, 'wall_s': now - start, 'ros_s': node.get_clock().now().nanoseconds * 1e-9}
        if hasattr(message, 'header'):
            event.update(stamp=stamp(message.header.stamp), frame=message.header.frame_id)
        try:
            if topic == '/rosout':
                if 'aeb' not in message.name.lower() and '[AEB]' not in message.msg:
                    return
                event.update(name=message.name, level=message.level, text=message.msg)
            elif topic == '/diagnostics':
                event['status'] = [{'name': s.name, 'level': integer(s.level), 'message': s.message,
                                    'values': {v.key: v.value for v in s.values}}
                                   for s in message.status if 'aeb' in s.name.lower() or 'emergency_braking' in s.name.lower()]
                if not event['status']:
                    return
            elif hasattr(message, 'markers'):
                event['markers'] = [{'ns': m.ns, 'id': m.id, 'action': m.action, 'type': m.type,
                                     'frame': m.header.frame_id, 'stamp': stamp(m.header.stamp),
                                     'position': xyz(m.pose.position), 'text': m.text,
                                     'points': [xyz(p) for p in m.points]} for m in message.markers]
            elif topic.endswith('/rss_distance'):
                event.update(stamp=stamp(message.stamp), rss_m=message.data)
            elif hasattr(message, 'metric_array'):
                event['metrics'] = [{'name': m.name, 'value': m.value, 'unit': m.unit} for m in message.metric_array]
            elif topic == '/autoware/state':
                event['state'] = message.state
            elif hasattr(message, 'longitudinal_velocity'):
                event['velocity_mps'] = message.longitudinal_velocity
            elif topic.endswith('/predicted_trajectory'):
                event['points'] = [xyz(p.pose.position) for p in message.points]
                event['path_length_m'] = sum(math.dist(event['points'][i-1], event['points'][i]) for i in range(1, len(event['points'])))
            elif hasattr(message, 'objects'):
                event['objects'] = [{'id': bytes(o.object_id.uuid).hex(),
                                     'position': xyz(o.kinematics.initial_pose_with_covariance.pose.position),
                                     'velocity': xyz(o.kinematics.initial_twist_with_covariance.twist.linear),
                                     'labels': [[c.label, c.probability] for c in o.classification]}
                                    for o in message.objects]
            elif topic.endswith('/pointcloud'):
                event['estimate'] = cloud_estimate(message, report['parameters'])
            report['events'].append(event)
        except Exception as error:
            report['errors'][topic + ': ' + repr(error)] += 1

    qos = QoSProfile(depth=3, reliability=ReliabilityPolicy.BEST_EFFORT)
    subscriptions = []
    for topic, type_name in TOPICS.items():
        report['topics'][topic] = {'messages': 0, 'publishers': [p.node_namespace.rstrip('/') + '/' + p.node_name
                                      for p in node.get_publishers_info_by_topic(topic)]}
        subscriptions.append(node.create_subscription(get_message(type_name), topic,
                             lambda message, topic=topic: callback(message, topic), qos))
    print('AEB_OBSERVER_READY', file=sys.stderr, flush=True)
    try:
        while time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=.025)
    except KeyboardInterrupt:
        pass
    finally:
        report['wall_duration_s'] = time.monotonic() - start
        temporary_output = args.output + '.tmp'
        with open(temporary_output, 'w', encoding='utf-8') as stream:
            json.dump(report, stream, indent=2, default=json_value)
            stream.write('\n')
        os.replace(temporary_output, args.output)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
