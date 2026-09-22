import json
import math
import time
from pathlib import Path

import carla
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from autoware_adapi_v1_msgs.msg import OperationModeState, RouteState
from autoware_adapi_v1_msgs.srv import ChangeOperationMode
from autoware_internal_planning_msgs.msg import VelocityLimit
from nav_msgs.msg import Odometry

rclpy.init()
node = rclpy.create_node('carla_validation_scene_restore')
last = {}
latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
stream = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
subs = []
for key, cls, topic, qos in (
    ('mode', OperationModeState, '/api/operation_mode/state', latched),
    ('route', RouteState, '/api/routing/state', latched),
    ('velocity_limit', VelocityLimit, '/planning/scenario_planning/current_max_velocity', latched),
    ('odom', Odometry, '/localization/kinematic_state', stream),
):
    subs.append(node.create_subscription(cls, topic, lambda m, k=key: last.update({k:m}), qos))

def spin(seconds):
    end = time.monotonic()+seconds
    while time.monotonic()<end:
        rclpy.spin_once(node, timeout_sec=.03)

report = {'restored': [], 'errors': []}
client = carla.Client('127.0.0.1',2000)
client.set_timeout(15)
world = client.get_world()
assert world.get_map().name.endswith('Town05_Opt')
actors = world.get_actors()
vehicles = list(actors.filter('vehicle.*'))
assert len(vehicles)==1
ego = vehicles[0]
assert ego.attributes.get('role_name')=='ego_vehicle'
stop = node.create_client(ChangeOperationMode,'/api/operation_mode/change_to_stop')
assert stop.wait_for_service(timeout_sec=5)
future = stop.call_async(ChangeOperationMode.Request())
end = time.monotonic()+15
while not future.done() and time.monotonic()<end:
    rclpy.spin_once(node,timeout_sec=.05)
assert future.done() and future.result().status.success
spin(2)
assert last['mode'].mode==OperationModeState.STOP and ego.get_velocity().length()<.05
report['initial_state']={k:message_to_ordereddict(v) for k,v in last.items() if k!='odom'}
assert last['route'].state==RouteState.UNSET
assert last['velocity_limit'].sender!='carla_closed_loop_test'

scene=json.loads(Path('/tmp/carla-validation/before_scene.json').read_text())['actors']
original=next(a for a in scene if a['attributes'].get('role_name')=='ego_vehicle')
original_ego_id=original['id']
expected_npcs=len(scene)-1
def transform(record):
    x,y,z=record['xyz'];roll,pitch,yaw=record['rpy']
    return carla.Transform(carla.Location(x=x,y=y,z=z+.15),carla.Rotation(roll=roll,pitch=pitch,yaw=yaw))
ego.set_target_velocity(carla.Vector3D())
ego.set_target_angular_velocity(carla.Vector3D())
ego.set_transform(transform(original))
spin(2)
assert last['mode'].mode==OperationModeState.STOP and ego.get_velocity().length()<.05
library=world.get_blueprint_library()
for record in scene:
    if record['id']==original_ego_id:continue
    blueprint=library.find(record['type'])
    for name,value in record['attributes'].items():
        if blueprint.has_attribute(name) and blueprint.get_attribute(name).is_modifiable:
            blueprint.set_attribute(name,value)
    actor=world.try_spawn_actor(blueprint,transform(record))
    if actor is None:
        report['errors'].append({'original_id':record['id'],'error':'spawn failed'})
        continue
    entry={'original_id':record['id'],'new_id':actor.id,'type':actor.type_id}
    report['restored'].append(entry)
    try:
        actor.set_autopilot(True,8000)
        entry['autopilot']=True
    except Exception as error:
        entry['autopilot']=False
        report['errors'].append({'new_id':actor.id,'error':str(error)})
    Path('/tmp/carla-validation/scene_restore.json').write_text(json.dumps(report,indent=2))
spin(5)
report['final_state']={k:message_to_ordereddict(v) for k,v in last.items()}
report['ego_final_speed_mps']=ego.get_velocity().length()
report['ego_final_xyz']=[getattr(ego.get_location(),k) for k in ('x','y','z')]
report['vehicle_count']=len(world.get_actors().filter('vehicle.*'))
report['walker_count']=len(world.get_actors().filter('walker.pedestrian.*'))
report['remaining_fixture_ids']=[a.id for a in world.get_actors() if a.attributes.get('role_name') in ('validation_fixture','validation_diagnostic')]
report['success']=(not report['errors'] and len(report['restored'])==expected_npcs
    and last['mode'].mode==OperationModeState.STOP and last['route'].state==RouteState.UNSET
    and report['ego_final_speed_mps']<.05 and not report['remaining_fixture_ids']
    and last['velocity_limit'].sender!='carla_closed_loop_test')
Path('/tmp/carla-validation/scene_restore.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report))
node.destroy_node()
rclpy.shutdown()
