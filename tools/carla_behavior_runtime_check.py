#!/usr/bin/env python3
"""Read-only ROS integration check. No CARLA client, publishers, ticks or controls."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.srv import GetParameters
from rosidl_runtime_py.convert import message_to_ordereddict
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from autoware_adapi_v1_msgs.msg import OperationModeState, RouteState
from autoware_perception_msgs.msg import TrackedObjects, PredictedObjects, DetectedObjects
from autoware_planning_msgs.msg import LaneletRoute

TOPICS = {
    'clock': '/clock',
    'odom': '/localization/kinematic_state',
    'tracked': '/perception/object_recognition/tracking/objects',
    'predicted': '/perception/object_recognition/objects',
    'centerpoint': '/perception/object_recognition/detection/centerpoint/objects',
    'detected': '/perception/object_recognition/detection/objects',
    'mode': '/api/operation_mode/state',
    'route_state': '/api/routing/state',
    'route': '/planning/mission_planning/route',
}


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def full_name(name, namespace):
    return namespace.rstrip('/') + '/' + name


def parameter_value(value):
    fields = {0:None, 1:'bool_value', 2:'integer_value', 3:'double_value', 4:'string_value',
              5:'byte_array_value', 6:'bool_array_value', 7:'integer_array_value',
              8:'double_array_value', 9:'string_array_value'}
    key = fields.get(value.type)
    result = getattr(value, key) if key else None
    return list(result) if value.type >= 5 else result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', '--expected-mode', choices=('sensor','ground_truth'), required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=5.0)
    args = parser.parse_args()
    if args.duration < 1:
        parser.error('duration must be at least 1 second')
    rclpy.init()
    node = Node('carla_behavior_runtime_check', parameter_overrides=[Parameter('use_sim_time', value=True)])
    stream = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    latest, last_wall, stamps, counts, class_counts, frames = {}, {}, {}, Counter(), {}, {}
    object_counts, objects_preview, subscribers = {}, {}, []
    all_clock = []

    def receive(key, message):
        latest[key] = message
        last_wall[key] = time.monotonic()
        counts[key] += 1
        stamp = message.clock if key == 'clock' else getattr(getattr(message, 'header', None), 'stamp', None)
        if stamp is not None:
            stamps.setdefault(key, []).append(seconds(stamp))
        if key == 'clock':
            all_clock.append(seconds(message.clock))
        if hasattr(message, 'header'):
            frames[key] = message.header.frame_id
        if hasattr(message, 'objects'):
            object_counts.setdefault(key, []).append(len(message.objects))
            labels = Counter()
            preview = []
            for obj in message.objects:
                label = max(obj.classification, key=lambda c:c.probability).label if obj.classification else -1
                labels[str(label)] += 1
                kin = obj.kinematics
                p = kin.initial_pose_with_covariance.pose if key == 'predicted' else kin.pose_with_covariance.pose
                row = {'label': label, 'position': [p.position.x,p.position.y,p.position.z],
                       'dimensions': [obj.shape.dimensions.x,obj.shape.dimensions.y,obj.shape.dimensions.z]}
                if hasattr(obj, 'object_id'):
                    row['uuid'] = bytes(obj.object_id.uuid).hex()
                if key == 'tracked':
                    row['orientation_availability'] = kin.orientation_availability
                    row['is_stationary'] = kin.is_stationary
                    row['velocity'] = [kin.twist_with_covariance.twist.linear.x,
                                       kin.twist_with_covariance.twist.linear.y,
                                       kin.twist_with_covariance.twist.linear.z]
                preview.append(row)
            class_counts[key] = dict(labels)
            objects_preview[key] = preview

    for key, message_type in [('clock',Clock), ('odom',Odometry), ('tracked',TrackedObjects),
                             ('predicted',PredictedObjects), ('centerpoint',DetectedObjects),
                             ('detected',DetectedObjects), ('mode',OperationModeState),
                             ('route_state',RouteState), ('route',LaneletRoute)]:
        subscribers.append(node.create_subscription(message_type, TOPICS[key],
            lambda msg,k=key:receive(k,msg), latched if key in ('mode','route_state','route') else stream))

    requested = {
        '/autoware_carla_interface': ['carla_perception_mode','ground_truth_range_m','sync_mode','fixed_delta_seconds'],
        '/control/autonomous_emergency_braking': ['use_predicted_object_data','use_pointcloud_data'],
    }
    clients = {name:node.create_client(GetParameters, name + '/get_parameters') for name in requested}
    futures = {}
    started = time.monotonic()
    while time.monotonic() - started < args.duration:
        for name, client in clients.items():
            if name not in futures and client.service_is_ready():
                futures[name] = client.call_async(GetParameters.Request(names=requested[name]))
        rclpy.spin_once(node, timeout_sec=.02)

    now = time.monotonic()
    params = {}
    for name, names in requested.items():
        future = futures.get(name)
        if future is not None and future.done() and future.exception() is None:
            params[name] = dict(zip(names, map(parameter_value,future.result().values)))
        else:
            params[name] = {'error':'parameter service unavailable or timed out'}
    endpoints = {}
    for key, topic in TOPICS.items():
        endpoints[key] = [{'node':full_name(e.node_name,e.node_namespace), 'type':e.topic_type,
                           'reliability':str(e.qos_profile.reliability),
                           'durability':str(e.qos_profile.durability)}
                          for e in node.get_publishers_info_by_topic(topic)]
    active_nodes = sorted(set(full_name(name,namespace) for name,namespace in node.get_node_names_and_namespaces()))
    result = {'requested_mode':args.mode, 'duration_wall_s':now-started, 'parameters':params,
              'publishers':endpoints, 'active_nodes':active_nodes, 'streams':{}, 'checks':[]}
    last_clock = all_clock[-1] if all_clock else None
    for key in TOPICS:
        values = stamps.get(key,[])
        result['streams'][key] = {
            'topic':TOPICS[key], 'received':counts[key], 'frame':frames.get(key),
            'stamp_first':values[0] if values else None, 'stamp_last':values[-1] if values else None,
            'age_wall_s':now-last_wall[key] if key in last_wall else None,
            'age_sim_s':last_clock-values[-1] if last_clock is not None and values else None,
            'stamp_monotonic':all(b>=a-1e-9 for a,b in zip(values,values[1:])),
            'object_count_min':min(object_counts[key]) if key in object_counts else None,
            'object_count_max':max(object_counts[key]) if key in object_counts else None,
            'latest_class_counts':class_counts.get(key), 'latest_objects':objects_preview.get(key),
        }
    for key in ('mode','route_state','route'):
        if key in latest:
            result[key] = message_to_ordereddict(latest[key])
    if 'odom' in latest:
        msg=latest['odom']
        result['odometry'] = {'frame_id':msg.header.frame_id, 'child_frame_id':msg.child_frame_id,
            'position':message_to_ordereddict(msg.pose.pose.position),
            'linear_velocity':message_to_ordereddict(msg.twist.twist.linear)}

    def check(name, passed, detail):
        result['checks'].append({'name':name,'passed':bool(passed),'detail':detail})
    bridge=params['/autoware_carla_interface'];aeb=params['/control/autonomous_emergency_braking']
    check('bridge mode', bridge.get('carla_perception_mode')==args.mode, bridge)
    check('bridge sole tick configuration', bridge.get('sync_mode') is True and bridge.get('fixed_delta_seconds',0)>0, bridge)
    check('one clock publisher',len(endpoints['clock'])==1,endpoints['clock'])
    check('clock advancing',len(all_clock)>=3 and all_clock[-1]>all_clock[0],result['streams']['clock'])
    for key in ('clock','odom','tracked','predicted'):
        s=result['streams'][key]
        good=counts[key]>=3 and s['age_wall_s'] is not None and s['age_wall_s']<1.0
        good=good and s['age_sim_s'] is not None and -.2 <= s['age_sim_s']<.5 and s['stamp_monotonic']
        check(key+' fresh',good,{k:v for k,v in s.items() if k!='latest_objects'})
    for key in ('tracked','predicted','odom'):
        check(key+' map frame',frames.get(key)=='map',frames.get(key))
    check('one tracking publisher',len(endpoints['tracked'])==1,endpoints['tracked'])
    check('one map prediction publisher',len(endpoints['predicted'])==1 and
          'map_based_prediction' in endpoints['predicted'][0]['node'],endpoints['predicted'])
    if args.mode=='ground_truth':
        check('GT tracking source',len(endpoints['tracked'])==1 and endpoints['tracked'][0]['node']=='/autoware_carla_interface',endpoints['tracked'])
        check('sensor detection publishers disabled',not endpoints['centerpoint'] and not endpoints['detected'],
              {'centerpoint':endpoints['centerpoint'],'detected':endpoints['detected']})
        unwanted=[n for n in active_nodes if '/detection/' in n or '/tracking/' in n]
        check('sensor detection and tracking nodes disabled',not unwanted,unwanted)
        check('GT range positive',isinstance(bridge.get('ground_truth_range_m'),(int,float)) and
              bridge['ground_truth_range_m']>0,bridge.get('ground_truth_range_m'))
    else:
        check('sensor tracking source',len(endpoints['tracked'])==1 and
              endpoints['tracked'][0]['node']!='/autoware_carla_interface',endpoints['tracked'])
        check('CenterPoint source present',len(endpoints['centerpoint'])==1,endpoints['centerpoint'])
    check('AEB predicted input mode',aeb.get('use_predicted_object_data') is (args.mode=='ground_truth'),aeb)
    check('AEB LiDAR retained',aeb.get('use_pointcloud_data') is True,aeb)
    check('operation mode state present','mode' in latest,result.get('mode'))
    check('route state present','route_state' in latest,result.get('route_state'))
    result['success']=all(c['passed'] for c in result['checks'])
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({'success':result['success'],'output':str(args.output),
                      'checks':len(result['checks']),
                      'failed':[c['name'] for c in result['checks'] if not c['passed']]}))
    node.destroy_node();rclpy.shutdown()
    return 0 if result['success'] else 2


if __name__=='__main__':
    raise SystemExit(main())
