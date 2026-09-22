#!/usr/bin/env python3
"""Read-only, stationary-walker perception probe. No simulator ticks or controls."""
import argparse
import collections
import json
import math
import time

import carla
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener


CLOUDS = [
    '/sensing/lidar/top/pointcloud_before_sync',
    '/sensing/lidar/concatenated/pointcloud',
    '/sensing/lidar/self_cropped/pointcloud',
    '/sensing/lidar/mirror_cropped/pointcloud',
    '/perception/obstacle_segmentation/range_cropped/pointcloud',
    '/perception/obstacle_segmentation/single_frame/pointcloud',
    '/perception/obstacle_segmentation/pointcloud',
    '/perception/obstacle_segmentation/pointcloud_map_filtered/downsampled/pointcloud',
    '/perception/object_recognition/detection/pointcloud_map_filtered/pointcloud',
    '/perception/object_recognition/detection/clustering/debug/clusters',
    '/control/autonomous_emergency_braking/debug/obstacle_pointcloud',
]
OBJECTS = {
    '/perception/object_recognition/detection/centerpoint/objects': 'autoware_perception_msgs/msg/DetectedObjects',
    '/perception/object_recognition/detection/clustering/objects': 'autoware_perception_msgs/msg/DetectedObjects',
    '/perception/object_recognition/detection/clustering/objects_with_feature': 'tier4_perception_msgs/msg/DetectedObjectsWithFeature',
    '/perception/object_recognition/tracking/objects': 'autoware_perception_msgs/msg/TrackedObjects',
    '/perception/object_recognition/objects': 'autoware_perception_msgs/msg/PredictedObjects',
}


def transform_matrix(msg):
    t, q = msg.transform.translation, msg.transform.rotation
    x,y,z,w = q.x,q.y,q.z,q.w
    f = 2/(x*x+y*y+z*z+w*w)
    m = np.eye(4)
    m[:3,:3] = [[1-f*(y*y+z*z),f*(x*y-z*w),f*(x*z+y*w)],
                 [f*(x*y+z*w),1-f*(x*x+z*z),f*(y*z-x*w)],
                 [f*(x*z-y*w),f*(y*z+x*w),1-f*(x*x+y*y)]]
    m[:3,3] = [t.x,t.y,t.z]
    return m


def points_xyz(message):
    fields = {f.name:f for f in message.fields}
    if not all(name in fields and fields[name].datatype == 7 for name in ('x','y','z')):
        raise ValueError('Expected FLOAT32 x/y/z fields')
    endian = '>' if message.is_bigendian else '<'
    dtype = np.dtype({'names':['x','y','z'], 'formats':[endian+'f4']*3,
                      'offsets':[fields[n].offset for n in ('x','y','z')],
                      'itemsize':message.point_step})
    data = np.ndarray((message.height,message.width),dtype=dtype,buffer=message.data,
                      strides=(message.row_step,message.point_step))
    result = np.column_stack([data[name].reshape(-1) for name in ('x','y','z')])
    return result[np.isfinite(result).all(axis=1)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--actor-id',type=int,required=True)
    parser.add_argument('--duration',type=float,default=8)
    parser.add_argument('--host',default='127.0.0.1')
    parser.add_argument('--port',type=int,default=2000)
    args = parser.parse_args()
    client = carla.Client(args.host,args.port);client.set_timeout(3)
    world = client.get_world()
    actor = None
    for _ in range(20):
        actor = world.get_actor(args.actor_id)
        if actor is not None:break
        time.sleep(.1)
    if actor is None or not actor.type_id.startswith('walker.pedestrian.'):
        raise RuntimeError('Requested actor is not a live CARLA pedestrian')
    rclpy.init()
    node = Node('carla_pedestrian_perception_readonly',parameter_overrides=[Parameter('use_sim_time',value=True)])
    tf = Buffer(node=node);listener = TransformListener(tf,node)
    end = time.monotonic()+1.5
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.025)
    qos = QoSProfile(depth=2,reliability=ReliabilityPolicy.BEST_EFFORT)
    reflection = np.diag([1.,-1.,1.,1.])
    report = {'actor_id':actor.id,'actor_type':actor.type_id,'note':
              'ROI uses live CARLA bounding box and latest ROS TF; diagnostic assumes stationary ego and walker.',
              'clouds':{},'objects':{},'cloud_pipeline':{},'ego':{}}
    subscriptions=[]
    box=actor.bounding_box
    ext=np.array([box.extent.x,box.extent.y,box.extent.z])
    report['bbox_extent_m']=ext.tolist()
    report['actor_xyz_carla']=[actor.get_location().x,actor.get_location().y,actor.get_location().z]
    report['actor_speed_mps']=actor.get_velocity().length()
    egos=[a for a in world.get_actors().filter('vehicle.*') if a.attributes.get('role_name')=='ego_vehicle']
    if len(egos)==1:
        report['ego']={'speed_mps':egos[0].get_velocity().length(),
                       'distance_to_actor_m':egos[0].get_location().distance(actor.get_location())}

    def box_world():
        return np.asarray(actor.get_transform().get_matrix()) @ np.asarray(carla.Transform(box.location,box.rotation).get_matrix())

    def from_map(frame):
        if frame.lstrip('/')=='map':return np.eye(4)
        return transform_matrix(tf.lookup_transform(frame,'map',Time()))

    def cloud(message,topic):
        row=report['clouds'][topic];row['messages']+=1
        now=time.monotonic()
        if now-row.get('_last',0)<.19:return
        row['_last']=now
        try:
            points=points_xyz(message)
            cloud_from_box=from_map(message.header.frame_id)@reflection@box_world()
            box_from_cloud=np.linalg.inv(cloud_from_box)
            local=points@box_from_cloud[:3,:3].T+box_from_cloud[:3,3]
            in_box=(np.abs(local)<=ext+.15).all(axis=1)
            # Exclude ground and feet: at least 0.30 m above the bounding-box bottom.
            upper=in_box&(local[:,2]>-ext[2]+.30)
            halo=(np.abs(local[:,:2])<=ext[:2]+.75).all(axis=1)&(np.abs(local[:,2])<=ext[2]+.3)
            row['samples'].append({'stamp':message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
                'frame':message.header.frame_id,'total_finite_points':len(points),
                'bbox_points':int(in_box.sum()),'upper_body_points':int(upper.sum()),
                'halo_points':int(halo.sum()),
                'bbox_xyz_min':local[in_box].min(axis=0).tolist() if in_box.any() else None,
                'bbox_xyz_max':local[in_box].max(axis=0).tolist() if in_box.any() else None})
        except Exception as error:
            row['errors'][str(error)]+=1

    def objects(message,topic):
        row=report['objects'][topic];row['messages']+=1
        try:
            items = message.feature_objects if hasattr(message,'feature_objects') else message.objects
            items = [item.object if hasattr(item,'object') else item for item in items]
            row['max_objects']=max(row['max_objects'],len(items));row['last_objects']=len(items)
            row['classes']=dict(collections.Counter(max(o.classification,key=lambda c:c.probability).label
                for o in items if o.classification))
            target=from_map(message.header.frame_id)@reflection@box_world()@np.array([0.,0.,0.,1.])
            matches=[]
            for obj in items:
                kin=obj.kinematics
                pose=kin.pose_with_covariance.pose if hasattr(kin,'pose_with_covariance') else kin.initial_pose_with_covariance.pose
                distance=math.hypot(pose.position.x-target[0],pose.position.y-target[1])
                if distance<2:
                    matches.append({'distance_xy_m':distance,'labels':[[c.label,c.probability] for c in obj.classification],
                                    'dimensions':[obj.shape.dimensions.x,obj.shape.dimensions.y,obj.shape.dimensions.z]})
            if matches:row['matched_messages']+=1
            row['last_matches']=matches
            if matches:row['last_nonempty_matches']=matches
        except Exception as error:row['errors'][str(error)]+=1

    types=dict(node.get_topic_names_and_types())
    for topic in CLOUDS:
        report['clouds'][topic]={'messages':0,'samples':[],'errors':collections.Counter()}
        subscriptions.append(node.create_subscription(PointCloud2,topic,lambda msg,t=topic:cloud(msg,t),qos))
    for topic,cls in OBJECTS.items():
        report['objects'][topic]={'messages':0,'max_objects':0,'matched_messages':0,'errors':collections.Counter()}
        subscriptions.append(node.create_subscription(get_message(types.get(topic,[cls])[0]),topic,
            lambda msg,t=topic:objects(msg,t),qos))
    start=time.monotonic()
    while time.monotonic()-start<args.duration:rclpy.spin_once(node,timeout_sec=.02)
    report['sample_wall_s']=time.monotonic()-start
    for topic,row in report['clouds'].items():
        row.pop('_last',None)
        if row['samples']:
            row['roi_summary']={key:{'min':min(r[key] for r in row['samples']),
                'median':float(np.median([r[key] for r in row['samples']])),
                'max':max(r[key] for r in row['samples'])}
                for key in ('bbox_points','upper_body_points','halo_points','total_finite_points')}
    for topic in list(CLOUDS)+list(OBJECTS):
        report['cloud_pipeline'][topic]={'publishers':[p.node_namespace.rstrip('/')+'/'+p.node_name for p in node.get_publishers_info_by_topic(topic)],
            'subscribers':[p.node_namespace.rstrip('/')+'/'+p.node_name for p in node.get_subscriptions_info_by_topic(topic)]}
    print(json.dumps(report,separators=(',',':'),allow_nan=False))
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
