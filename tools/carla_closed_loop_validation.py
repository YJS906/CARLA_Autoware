#!/usr/bin/env python3
"""Controlled CARLA-only driving checks through Autoware's normal ADAPI.

This intentionally engages a simulated ego vehicle. Never use against a real
vehicle ROS graph. CARLA must have exactly one ego_vehicle in native Town05.
The existing bridge remains the only world ticker. Each case ends in STOP,
clears its route and temporary speed cap, and removes only its fixture actors.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import signal
import time

import carla
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rosidl_runtime_py.convert import message_to_ordereddict

from autoware_adapi_v1_msgs.msg import OperationModeState, RouteState
from autoware_adapi_v1_msgs.srv import ChangeOperationMode, ClearRoute, SetRoutePoints
from autoware_control_msgs.msg import Control
from autoware_internal_planning_msgs.msg import (
    PlanningFactorArray, VelocityLimit, VelocityLimitClearCommand,
)
from autoware_perception_msgs.msg import PredictedObjects
from autoware_planning_msgs.msg import LaneletRoute, Trajectory
from nav_msgs.msg import Odometry
from tier4_rtc_msgs.msg import CooperateStatusArray
from visualization_msgs.msg import MarkerArray

from carla_validation_actors import ActorFixture
from carla_native_lane_check import query_driving_lane


SENDER = 'carla_closed_loop_test'
START = (210.340576, 74.101173, 0.5, -89.335365)


def xyz(v):
    return [v.x, v.y, v.z]


def speed(actor):
    return math.sqrt(sum(v*v for v in xyz(actor.get_velocity())))


def polygon(actor):
    points=sorted(set((v.x,v.y) for v in actor.bounding_box.get_world_vertices(actor.get_transform())))
    def cross(o,a,b):return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    def half(seq):
        out=[]
        for p in seq:
            while len(out)>=2 and cross(out[-2],out[-1],p)<=0:out.pop()
            out.append(p)
        return out
    return half(points)[:-1]+half(reversed(points))[:-1]


def polygon_distance(a,b):
    """Exact convex 2D footprint separation; no additional runtime dependency."""
    if len(a)<3 or len(b)<3:raise ValueError('Degenerate actor footprint')
    separated=False
    for poly in (a,b):
        for p,q in zip(poly,poly[1:]+poly[:1]):
            nx,ny=-(q[1]-p[1]),q[0]-p[0]
            pa=[x*nx+y*ny for x,y in a];pb=[x*nx+y*ny for x,y in b]
            if max(pa)<min(pb) or max(pb)<min(pa):separated=True
    if not separated:return 0.0
    def point_segment(p,a,b):
        dx,dy=b[0]-a[0],b[1]-a[1]
        t=max(0,min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy)))
        return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)
    return min(point_segment(p,q,r) for points,edges in ((a,b),(b,a))
               for p in points for q,r in zip(edges,edges[1:]+edges[:1]))


def transform_record(actor):
    t = actor.get_transform()
    return {'id': actor.id, 'type': actor.type_id, 'attributes': dict(actor.attributes),
            'xyz': xyz(t.location), 'rpy': [t.rotation.roll, t.rotation.pitch, t.rotation.yaw]}


class Validation(Node):
    def __init__(self, args):
        super().__init__('carla_closed_loop_validation', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.args = args
        self.result = {'case': args.case, 'speed_cap_kmh': args.speed_kmh,
                       'samples': [], 'events': [], 'planning_factors': {}, 'rtc': {},
                       'success': False, 'result': 'not_started', 'cleanup': {}}
        self.last = {}
        self.received = {}
        self.factor_counts = Counter()
        self.rtc_states = {}
        self.avoidance_reasons = Counter()
        self.drivable_violations = []
        self.native_lane_query_seams = []
        self.minimum_motion_mps = 1.0 if args.case in ('vehicle_stop', 'pedestrian_stop') else 2.0
        self.result['minimum_motion_mps'] = self.minimum_motion_mps
        self.required_stop_hold_s = 5.0 if args.case == 'pedestrian_stop' else 2.0
        self.result['required_stop_hold_s'] = self.required_stop_hold_s
        self.fixture = None
        self.ego = None
        self.limit_sent = False
        self.route_sent = False
        self.case_running = False
        self.stream = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subs = []
        for name, cls, topic, qos in (
            ('mode', OperationModeState, '/api/operation_mode/state', self.latched),
            ('route_state', RouteState, '/api/routing/state', self.latched),
            ('route', LaneletRoute, '/planning/mission_planning/route', self.latched),
            ('odom', Odometry, '/localization/kinematic_state', self.stream),
            ('trajectory', Trajectory, '/planning/trajectory', self.stream),
            ('control', Control, '/control/command/control_cmd', self.stream),
            ('objects', PredictedObjects, '/perception/object_recognition/objects', self.stream),
            ('velocity_limit', VelocityLimit, '/planning/scenario_planning/current_max_velocity', self.latched),
        ):
            def receive(msg, key=name):
                self.last[key] = msg
                self.received[key] = time.monotonic()
            self.subs.append(self.create_subscription(cls, topic, receive, qos))
        for module in ('static_obstacle_avoidance', 'avoidance_by_lane_change',
                       'obstacle_stop', 'obstacle_slow_down', 'obstacle_cruise',
                       'dynamic_obstacle_stop', 'run_out', 'road_user_stop',
                       'crosswalk', 'traffic_light', 'motion_velocity_planner'):
            def factor(msg, key=module):
                if self.case_running and msg.factors:
                    self.factor_counts[key] += 1
                    self.result['planning_factors'][key] = message_to_ordereddict(msg)
                    if self.factor_counts[key] == 1:
                        self.event('first_planning_factor', module=key,
                                   message=message_to_ordereddict(msg))
            self.subs.append(self.create_subscription(PlanningFactorArray,
                '/planning/planning_factors/'+module, factor, self.stream))
        for module in ('static_obstacle_avoidance_left', 'static_obstacle_avoidance_right',
                       'avoidance_by_lane_change_left', 'avoidance_by_lane_change_right'):
            def rtc(msg, key=module):
                if self.case_running and msg.statuses:
                    self.result['rtc'][key] = message_to_ordereddict(msg)
                    self.rtc_states.setdefault(key,set()).update(s.state.type for s in msg.statuses)
            self.subs.append(self.create_subscription(CooperateStatusArray,
                '/planning/cooperate_status/'+module, rtc, self.stream))
        def avoidance_debug(msg):
            if self.case_running:
                self.avoidance_reasons.update(m.text for m in msg.markers if m.text)
        self.subs.append(self.create_subscription(MarkerArray,
            '/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/info/static_obstacle_avoidance',
            avoidance_debug, self.stream))
        self.stop_client = self.create_client(ChangeOperationMode, '/api/operation_mode/change_to_stop')
        self.auto_client = self.create_client(ChangeOperationMode, '/api/operation_mode/change_to_autonomous')
        self.enable_client = self.create_client(ChangeOperationMode, '/api/operation_mode/enable_autoware_control')
        self.route_client = self.create_client(SetRoutePoints, '/api/routing/set_route_points')
        self.clear_client = self.create_client(ClearRoute, '/api/routing/clear_route')
        self.limit_pub = self.create_publisher(VelocityLimit,
            '/planning/scenario_planning/max_velocity_candidates', self.latched)
        self.clear_limit_pub = self.create_publisher(VelocityLimitClearCommand,
            '/planning/scenario_planning/clear_velocity_limit', self.latched)
        self.carla = carla.Client(args.host, args.port)
        self.carla.set_timeout(3)
        self.world = self.carla.get_world()
        if self.world.get_map().name.rsplit('/', 1)[-1] not in ('Town05', 'Town05_Opt'):
            raise RuntimeError('Native Town05 CARLA world required')
        self.world.wait_for_tick(3)  # observation only, never tick the simulator
        self.world_id = self.world.id
        actors = self.world.get_actors()
        egos = [a for a in actors.filter('vehicle.*') if a.attributes.get('role_name') == 'ego_vehicle']
        if len(egos) != 1:
            raise RuntimeError('Exactly one CARLA ego_vehicle is required')
        self.ego = egos[0]
        self.result['original_scene'] = [transform_record(a) for a in actors.filter('vehicle.*')]
        self.result['ego_id'] = self.ego.id
        self.result['map_name'] = self.world.get_map().name
        self.result['world_id'] = self.world_id
        self.result['fixed_delta_seconds'] = self.world.get_settings().fixed_delta_seconds
        self.result['requested_input_mode'] = args.input_mode
        self.original_transform = self.ego.get_transform()
        self.wmap = self.world.get_map()

    def event(self, name, **fields):
        event = {'event': name, 'sim_time': self.get_clock().now().nanoseconds/1e9, **fields}
        self.result['events'].append(event)
        print(json.dumps(event), flush=True)

    def spin(self, seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.03)
            if self.fixture is not None:
                self.fixture.update(self.world.get_snapshot())

    def until(self, predicate, timeout, description):
        end = time.monotonic()+timeout
        while not predicate():
            if time.monotonic() >= end:
                raise RuntimeError('Timeout: '+description)
            self.spin(.05)

    def call(self, client, request, name, timeout=20):
        self.until(client.service_is_ready, 8, name+' service')
        future = client.call_async(request)
        self.until(future.done, timeout, name+' response')
        response = future.result()
        self.event(name, status=message_to_ordereddict(response.status))
        if not response.status.success:
            raise RuntimeError(name+': '+response.status.message)
        return response

    def stop(self):
        self.call(self.stop_client, ChangeOperationMode.Request(), 'stop')
        self.until(lambda: 'mode' in self.last and self.last['mode'].mode == OperationModeState.STOP
                   and speed(self.ego) < .05, 15, 'ego STOP and stationary')

    def waypoint(self, distance):
        choices = self.start_wp.next(distance)
        # Native east-side route remains on the same -3 lane outside junctions.
        choices = [p for p in choices if p.lane_id == self.start_wp.lane_id]
        if len(choices) != 1 or choices[0].is_junction:
            raise RuntimeError('Test point must be a unique non-junction lane')
        return choices[0]

    @staticmethod
    def raised(wp):
        t = wp.transform
        t.location.z += .35
        return t

    def prepare(self):
        self.until(lambda: 'mode' in self.last and 'odom' in self.last, 15, 'initial ROS state')
        self.stop()
        self.call(self.clear_client, ClearRoute.Request(), 'clear_previous_route')
        if self.args.isolate_traffic:
            npc_ids = [a.id for a in self.world.get_actors().filter('vehicle.*') if a.id != self.ego.id]
            walker_ids = [a.id for a in self.world.get_actors().filter('walker.pedestrian.*')]
            if walker_ids:
                raise RuntimeError('Existing pedestrians present: do not overwrite an active scenario')
            replies = self.carla.apply_batch_sync([carla.command.DestroyActor(i) for i in npc_ids], False)
            errors = [r.error for r in replies if r.has_error()]
            if errors:
                raise RuntimeError('NPC isolation failed: '+repr(errors))
            self.event('traffic_isolated', removed_vehicle_ids=npc_ids)
        elif len(self.world.get_actors().filter('vehicle.*')) != 1:
            raise RuntimeError('Use --isolate-traffic for a controlled test with existing NPCs')
        x, y, z, yaw = START
        self.ego.set_target_velocity(carla.Vector3D())
        self.ego.set_target_angular_velocity(carla.Vector3D())
        self.ego.set_transform(carla.Transform(carla.Location(x=x,y=y,z=z), carla.Rotation(yaw=yaw)))
        self.start_wp = self.wmap.get_waypoint(carla.Location(x=x,y=y,z=z))
        self.spin(3)
        if speed(self.ego) >= .05:
            raise RuntimeError('Reset ego did not settle')
        self.fixture = ActorFixture(self.world, self.ego)
        self.hazard_wp = self.waypoint(self.args.obstacle_distance)
        self.hazards = []
        if self.args.case in ('vehicle_stop', 'vehicle_avoidance'):
            locations = [self.hazard_wp]
            if self.args.case == 'vehicle_stop':
                lane = self.hazard_wp
                for _ in range(2):
                    lane = lane.get_left_lane()
                    if lane is None or lane.lane_type != carla.LaneType.Driving or lane.lane_id >= 0:
                        raise RuntimeError('Expected three same-direction lanes')
                    locations.append(lane)
            self.hazards = [self.fixture.parked_vehicle(self.raised(wp)) for wp in locations]
        elif self.args.case == 'pedestrian_stop':
            t = self.hazard_wp.transform
            right = t.get_right_vector()
            offset = self.hazard_wp.lane_width/2+1.0
            t.location += carla.Location(x=right.x*offset, y=right.y*offset, z=.5)
            direction = carla.Vector3D(x=-right.x,y=-right.y,z=0)
            self.hazards = [self.fixture.pedestrian(t,direction,1.8,25.0,travel_distance=offset)]
        self.spin(3)
        self.until(lambda: self.fixture.vehicles_settled, 5, 'fixture vehicles stationary')
        limit = VelocityLimit()
        limit.stamp = self.get_clock().now().to_msg()
        limit.max_velocity = self.args.speed_kmh/3.6
        limit.use_constraints = False
        limit.sender = SENDER
        self.limit_pub.publish(limit)
        self.limit_sent = True
        self.spin(.5)
        self.result['object_publishers'] = [
            p.node_namespace.rstrip('/')+'/'+p.node_name
            for p in self.get_publishers_info_by_topic('/perception/object_recognition/objects')]
        if len(self.result['object_publishers']) != 1:
            raise RuntimeError('Exactly one predicted-object publisher required')
        self.result['tracked_object_publishers'] = [
            p.node_namespace.rstrip('/')+'/'+p.node_name
            for p in self.get_publishers_info_by_topic('/perception/object_recognition/tracking/objects')]
        expected = ('/autoware_carla_interface' if self.args.input_mode == 'ground_truth'
                    else '/perception/object_recognition/tracking/multi_object_tracker')
        if self.result['tracked_object_publishers'] != [expected]:
            raise RuntimeError('Tracked-object publisher does not match requested input mode: '+
                               repr(self.result['tracked_object_publishers']))
        goal = self.waypoint(140).transform
        request = SetRoutePoints.Request()
        request.header.frame_id = 'map'
        request.header.stamp = self.get_clock().now().to_msg()
        request.option.allow_goal_modification = False
        request.goal.position.x, request.goal.position.y = goal.location.x, -goal.location.y
        angle = math.radians(-goal.rotation.yaw)
        request.goal.orientation.z, request.goal.orientation.w = math.sin(angle/2), math.cos(angle/2)
        self.route_sent = True
        route_requested_at = time.monotonic()
        self.call(self.route_client, request, 'set_test_route')
        self.until(lambda: self.received.get('route', 0) >= route_requested_at
                   and len(self.last['route'].segments) > 0, 5, 'fresh route message')
        self.result['lanelet_route'] = message_to_ordereddict(self.last['route'])
        self.until(lambda: 'trajectory' in self.last and len(self.last['trajectory'].points)>2
                   and time.monotonic()-self.received['trajectory']<1, 20, 'fresh trajectory')
        # The velocity smoother only reports its effective limit after receiving
        # a trajectory. Keep ego in STOP until the cap is confirmed on that path.
        self.until(lambda:'velocity_limit' in self.last and
                   self.last['velocity_limit'].max_velocity<=self.args.speed_kmh/3.6+.01,
                   5,'selected test speed limit')
        self.result['selected_velocity_limit']=message_to_ordereddict(self.last['velocity_limit'])
        self.until(lambda: self.last['mode'].is_autonomous_mode_available, 20, 'AUTO availability')
        if not self.last['mode'].is_autoware_control_enabled:
            self.call(self.enable_client, ChangeOperationMode.Request(), 'enable_autoware_control')
        self.result['hazards'] = [transform_record(a) for a in self.hazards]
        self.result['test_start'] = transform_record(self.ego)
        self.result['goal'] = message_to_ordereddict(request.goal)
        self.initial_xy = (self.ego.get_location().x, self.ego.get_location().y)
        self.case_start_frame = self.world.get_snapshot().frame
        self.case_running = True
        self.call(self.auto_client, ChangeOperationMode.Request(), 'engage_autonomous')
        self.until(lambda: self.last['mode'].mode==OperationModeState.AUTONOMOUS
                   and not self.last['mode'].is_in_transition, 15, 'AUTO transition')

    def run(self):
        self.prepare()
        wall_start = time.monotonic()
        sim_start = self.world.get_snapshot().timestamp.elapsed_seconds
        stopped_since = None
        continued_since = None
        max_speed = max_lateral = max_progress = 0.0
        min_clearance = float('inf')
        last_print = -10
        while time.monotonic()-wall_start < self.args.duration:
            self.spin(.1)
            snap = self.world.get_snapshot()
            if self.carla.get_world().id != self.world_id:
                raise RuntimeError('World changed during test')
            if time.monotonic()-self.received.get('odom',0)>1:
                raise RuntimeError('Localization stream stopped')
            position = self.ego.get_location()
            velocity = speed(self.ego)
            max_speed = max(max_speed,velocity)
            # East-side road is almost straight; native -3 center reference at current y.
            travelled = (START[1]-position.y)
            wp = self.wmap.get_waypoint(position)
            # Non-projecting native query distinguishes an actual road position
            # from the nearest lane returned for a vehicle already off the road.
            occupied_lane, seam_evidence = query_driving_lane(self.wmap, position)
            if seam_evidence is not None:
                self.native_lane_query_seams.append(
                    {'frame':snap.frame,'xyz':xyz(position), **seam_evidence})
            if occupied_lane is None or occupied_lane.lane_id >= 0:
                self.drivable_violations.append({'frame':snap.frame, 'xyz':xyz(position),
                    'lane_id':occupied_lane.lane_id if occupied_lane else None})
            ref = wp
            for _ in range(4):
                if ref.lane_id == -3: break
                next_ref = ref.get_right_lane()
                if next_ref is None or next_ref.lane_type != carla.LaneType.Driving: break
                ref = next_ref
            reference = ref.transform
            right = reference.get_right_vector()
            lateral = abs((position.x-reference.location.x)*right.x+
                          (position.y-reference.location.y)*right.y)
            max_lateral = max(max_lateral,lateral)
            max_progress = max(max_progress,travelled)
            gaps = [polygon_distance(polygon(self.ego),polygon(a)) for a in self.hazards if a.is_alive]
            clearance = min(gaps) if gaps else None
            if clearance is not None:min_clearance=min(min_clearance,clearance)
            objects = self.last.get('objects')
            classes = Counter(max(o.classification,key=lambda c:c.probability).label
                              for o in objects.objects if o.classification) if objects else {}
            matched=[]
            if objects:
                for a in self.hazards:
                    p=a.get_location()
                    nearest=min((math.hypot(o.kinematics.initial_pose_with_covariance.pose.position.x-p.x,
                                           o.kinematics.initial_pose_with_covariance.pose.position.y+p.y)
                                 for o in objects.objects),default=None)
                    matched.append({'actor':a.id,'closest_prediction_m':nearest})
            control=self.ego.get_control()
            trajectory=self.last.get('trajectory')
            row={'sim_s':snap.timestamp.elapsed_seconds-sim_start, 'frame':snap.frame,
                 'xyz':xyz(position),'speed_mps':velocity,'progress_m':travelled,
                 'lateral_from_original_lane_m':lateral,'clearance_m':clearance,
                 'mode':self.last['mode'].mode,'throttle':control.throttle,'brake':control.brake,
                 'steer':control.steer,'objects_by_class':dict(classes),'matches':matched,
                 'hazard_xyz':[xyz(a.get_location()) for a in self.hazards if a.is_alive],
                 'native_road_id':wp.road_id,'native_lane_id':wp.lane_id,
                 'native_lane_type':str(wp.lane_type),'native_is_junction':wp.is_junction,
                 'ros_time':self.get_clock().now().nanoseconds/1e9,
                 'object_stamp':objects.header.stamp.sec+objects.header.stamp.nanosec/1e9 if objects else None,
                 'ros_control':message_to_ordereddict(self.last['control']) if 'control' in self.last else None,
                 'trajectory_max_speed':max((p.longitudinal_velocity_mps for p in trajectory.points),default=0) if trajectory else None}
            self.result['samples'].append(row)
            if row['sim_s']-last_print>=2:
                self.event('progress',speed_mps=velocity,progress_m=travelled,
                           lateral_m=lateral,clearance_m=clearance,objects=dict(classes))
                if trajectory:
                    self.result.setdefault('trajectory_snapshots', []).append({
                        'frame':snap.frame, 'sim_s':row['sim_s'],
                        'trajectory':message_to_ordereddict(trajectory)})
                last_print=row['sim_s']
            if any(c['frame'] >= self.case_start_frame for c in self.fixture.collisions):
                self.result['result']='collision'
                break
            if row['mode']!=OperationModeState.AUTONOMOUS:
                self.result['result']='left_autonomous_mode'
                break
            if velocity > self.args.speed_kmh/3.6+.5:
                self.result['result']='speed_cap_exceeded'
                break
            if self.args.case=='baseline' and travelled>=30 and max_speed>=2:
                self.result.update(success=True,result='baseline_drive_pass')
                break
            if self.args.case=='vehicle_avoidance' and travelled>=self.args.obstacle_distance+12:
                if velocity>2 and wp.lane_type==carla.LaneType.Driving and wp.lane_id<0:
                    if continued_since is None:continued_since=row['sim_s']
                    if row['sim_s']-continued_since>=3:
                        passed=max_lateral>1.5 and min_clearance>.1 and not self.drivable_violations
                        self.result.update(success=passed,
                            result='obstacle_passed_and_continued' if passed else 'insufficient_avoidance_clearance',
                            continued_after_pass_s=row['sim_s']-continued_since)
                        break
                else:continued_since=None
            if self.args.case in ('vehicle_stop','pedestrian_stop'):
                relevant = self.args.case=='vehicle_stop' or any(
                    row['hazard_xyz'] and math.hypot(p[0]-self.hazard_wp.transform.location.x,
                    p[1]-self.hazard_wp.transform.location.y)<1.0 for p in row['hazard_xyz'])
                if velocity<.15 and max_speed>=self.minimum_motion_mps and travelled>5 and clearance is not None and clearance<18 and relevant:
                    if stopped_since is None:stopped_since=row['sim_s']
                    if row['sim_s']-stopped_since>=self.required_stop_hold_s:
                        self.result.update(success=clearance>.1,result='stopped_before_hazard',
                                           stopped_clearance_m=clearance)
                        break
                else:stopped_since=None
                if travelled>self.args.obstacle_distance+6:
                    self.result['result']='passed_hazard_without_required_stop'
                    break
        else:
            self.result['result']='timeout_no_completed_maneuver'
        # Apply road membership to every case, including a successful stop.
        if self.drivable_violations and self.result.get('success'):
            self.result.update(success=False, result='drivable_lane_violation')
        self.result.update(max_speed_mps=max_speed,max_lateral_m=max_lateral,
                           max_progress_m=max_progress,min_clearance_m=None if math.isinf(min_clearance) else min_clearance,
                           max_cap_overshoot_kmh=max(0.0,max_speed*3.6-self.args.speed_kmh),
                           drivable_violations=self.drivable_violations,
                           native_lane_query_seams=self.native_lane_query_seams)

    def cleanup(self):
        self.case_running=False
        errors=[]
        if self.ego is not None:
            try:
                self.stop()
                self.result['cleanup']['stop_verified']=True
            except Exception as error:
                errors.append('STOP: '+str(error))
                # A single control command could be overwritten by the live bridge.
                # Freeze this simulated actor and retain the cap/fixtures for recovery.
                self.ego.set_simulate_physics(False)
                self.ego.set_target_velocity(carla.Vector3D())
                self.result['cleanup']['simulator_physics_frozen']=True
                self.result['cleanup']['cap_and_fixtures_retained']=True
                self.result['cleanup']['errors']=errors
                self.result['success']=False
                return
        if self.limit_sent:
            clear=VelocityLimitClearCommand(stamp=self.get_clock().now().to_msg(),command=True,sender=SENDER)
            self.clear_limit_pub.publish(clear)
            self.spin(.4)
            self.result['cleanup']['test_speed_limit_clear_sent']=True
        if self.fixture is not None:
            self.result['collisions']=list(self.fixture.collisions)
            if any(c['frame']>=getattr(self,'case_start_frame',math.inf) for c in self.fixture.collisions):
                self.result.update(success=False,result='collision')
            self.result['pedestrian_states']=self.fixture.pedestrian_states
            self.result['fixture_actor_ids']=self.fixture.actor_ids
            try:self.fixture.cleanup()
            except Exception as error:errors.append('fixture: '+str(error))
            self.fixture=None
        if self.route_sent:
            try:
                self.call(self.clear_client,ClearRoute.Request(),'clear_test_route')
                self.result['cleanup']['route_cleared']=True
            except Exception as error:errors.append('route: '+str(error))
        self.result['factor_message_counts']=dict(self.factor_counts)
        self.result['avoidance_reasons']=dict(self.avoidance_reasons)
        self.result['rtc_observed_states']={k:sorted(v) for k,v in self.rtc_states.items()}
        self.result['fixture_detection_samples']=sum(any(m['closest_prediction_m'] is not None and
            m['closest_prediction_m']<3.0 for m in sample['matches']) for sample in self.result['samples'])
        self.result['cleanup']['errors']=errors
        if errors:self.result['success']=False


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',choices=['baseline','vehicle_stop','vehicle_avoidance','pedestrian_stop'],required=True)
    parser.add_argument('--host',default='127.0.0.1')
    parser.add_argument('--port',type=int,default=2000)
    parser.add_argument('--speed-kmh',type=float,default=20)
    parser.add_argument('--obstacle-distance',type=float,default=45)
    parser.add_argument('--duration',type=float,default=60)
    parser.add_argument('--isolate-traffic',action='store_true')
    parser.add_argument('--input-mode',choices=['sensor','ground_truth'],default='sensor',
                        help='Record the independently verified runtime input mode')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if not 5<=args.speed_kmh<=30 or not 30<=args.obstacle_distance<=90:
        parser.error('Test speed must be 5..30 km/h and obstacle distance 30..90 m')
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(*_):raise KeyboardInterrupt('test interrupted')
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    node=None
    try:
        node=Validation(args)
        node.run()
    except (Exception,KeyboardInterrupt) as error:
        if node is None:raise
        node.result['result']='error'
        node.result['error']=str(error)
        node.event('error',message=str(error))
    finally:
        if node is not None:
            try:node.cleanup()
            except Exception as cleanup_error:
                node.result['success']=False
                node.result['cleanup']['fatal_error']=str(cleanup_error)
            args.output.parent.mkdir(parents=True,exist_ok=True)
            args.output.write_text(json.dumps(node.result,indent=2)+'\n')
            print(json.dumps({k:v for k,v in node.result.items() if k not in
                  ('samples','events','planning_factors','rtc','original_scene',
                   'trajectory_snapshots','lanelet_route')}),flush=True)
            node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0 if node and node.result['success'] else 1


if __name__=='__main__':
    raise SystemExit(main())
