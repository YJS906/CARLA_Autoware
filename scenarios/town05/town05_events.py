"""One-shot proximity events. Only control event actors, never the ego."""

import math

import carla
import py_trees
from agents.navigation.local_planner import LocalPlanner, RoadOption
from srunner.scenariomanager.timer import GameTime
from srunner.tools.scenario_helper import (
    detect_lane_obstacle,
    generate_target_waypoint_list_multilane,
)


def number(settings, key, default=None, minimum=0.0, maximum=10000.0):
    try:
        value = float(settings.get(key, default))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{key} must be a number") from exc
    if not math.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError(f"{key} must be between {minimum} and {maximum}")
    return value


def enabled(settings):
    value = settings.get("enabled", "false").lower()
    if value not in ("true", "false"):
        raise ValueError("event enabled must be true or false")
    return value == "true"


def dot(a, b):
    return a.x * b.x + a.y * b.y


def lane_key(wp):
    return wp.road_id, wp.lane_id


def site_waypoint(wmap, settings):
    location = carla.Location(
        x=number(settings, "x", minimum=-100000),
        y=number(settings, "y", minimum=-100000),
        z=number(settings, "z", 0, minimum=-1000),
    )
    wp = wmap.get_waypoint(location, lane_type=carla.LaneType.Driving)
    if wp is None or wp.is_junction or wp.transform.location.distance(location) > 2:
        raise ValueError("Event x/y/z must be on a driving lane outside a junction")
    return wp


def lane_path(wp, distance, step=1.0):
    """Finite path; reject junctions, forks, lane endings and road transitions."""
    path = [wp]
    travelled = 0.0
    while travelled < distance:
        spacing = min(step, distance - travelled)
        following = path[-1].next(spacing)
        if len(following) != 1:
            raise ValueError("Event path reaches a fork or lane ending")
        nxt = following[0]
        if nxt.is_junction or lane_key(nxt) != lane_key(wp):
            raise ValueError("Event path must stay on one nonjunction road lane")
        if dot(nxt.transform.get_forward_vector(), wp.transform.get_forward_vector()) < 0.95:
            raise ValueError("Event path is too curved; choose a straighter section")
        path.append(nxt)
        travelled += spacing
    return path


def approach_distance(ego, wmap, site, limit):
    """Return forward distance only for moving ego approaching on the target lane."""
    position = ego.get_location()
    wp = wmap.get_waypoint(position, project_to_road=False, lane_type=carla.LaneType.Driving)
    if wp is None or lane_key(wp) != lane_key(site):
        return None
    forward = site.transform.get_forward_vector()
    if dot(ego.get_transform().get_forward_vector(), forward) < 0.8:
        return None
    if dot(ego.get_velocity(), forward) < 0.2:
        return None
    delta = site.transform.location - position
    distance = dot(delta, forward)
    if abs(delta.z) > 2.5 or not 0 < distance <= limit:
        return None
    return distance


class PedestrianEvent(py_trees.behaviour.Behaviour):
    def __init__(self, world, ego, settings, spawn):
        super().__init__("Pedestrian dart-out")
        self.world, self.ego = world, ego
        self.wmap = world.get_map()
        self.site = site_waypoint(self.wmap, settings)
        self.trigger = number(settings, "trigger_distance", 25, 1, 100)
        self.walk_speed = number(settings, "walk_speed", 1.2, 0.1, 3)
        self.run_speed = number(settings, "run_speed", 3.5, 0.1, 8)
        half_walk = number(settings, "walk_distance", 6, 1, 15) / 2
        side = settings.get("side", "right")
        if side not in ("left", "right"):
            raise ValueError("pedestrian side must be left or right")
        sidewalk = self.site
        # Keep the specified lateral direction; do not traverse opposing lanes.
        for _ in range(8):
            sidewalk = getattr(sidewalk, f"get_{side}_lane")()
            if sidewalk is None:
                break
            if sidewalk.lane_type == carla.LaneType.Sidewalk:
                break
            if dot(sidewalk.transform.get_forward_vector(), self.site.transform.get_forward_vector()) < 0.8:
                sidewalk = None
                break
        if sidewalk is None or sidewalk.lane_type != carla.LaneType.Sidewalk:
            raise ValueError("No sidewalk on the requested side of the pedestrian site")
        self.sidewalk = sidewalk
        before, after = sidewalk.previous(half_walk), sidewalk.next(half_walk)
        if len(before) != 1 or len(after) != 1 or any(
            wp.is_junction or lane_key(wp) != lane_key(sidewalk) for wp in before + after
        ):
            raise ValueError("Pedestrian walking segment must stay on the sidewalk")
        self.walk_points = [before[0].transform.location, after[0].transform.location]
        self.end = carla.Location(
            x=number(settings, "end_x", minimum=-100000),
            y=number(settings, "end_y", minimum=-100000),
            z=sidewalk.transform.location.z,
        )
        end_wp = self.wmap.get_waypoint(self.end, lane_type=carla.LaneType.Any)
        if end_wp is None or end_wp.lane_type not in (carla.LaneType.Sidewalk, carla.LaneType.Shoulder):
            raise ValueError("Pedestrian crossing must end on a sidewalk or refuge shoulder")
        if end_wp.transform.location.distance(self.end) > end_wp.lane_width / 2 + 0.2:
            raise ValueError("Pedestrian endpoint is outside the specified refuge")
        self.refuge_lane = lane_key(end_wp)
        self.actor = spawn("walker.pedestrian.0001", sidewalk.transform, "event_pedestrian")
        self.phase, self.walk_index = "walking", 0
        self.cross_started = 0.0
        print(f"[pedestrian] WALKING; trigger <= {self.trigger:g} m on lane {lane_key(self.site)}")

    def move_towards(self, target, speed):
        delta = target - self.actor.get_location()
        length = math.hypot(delta.x, delta.y)
        if length <= 0.35:
            self.actor.apply_control(carla.WalkerControl(speed=0))
            return True
        self.actor.apply_control(carla.WalkerControl(
            direction=carla.Vector3D(delta.x / length, delta.y / length, 0), speed=speed
        ))
        return False

    def update(self):
        if not self.actor.is_alive:
            if self.phase != "lost":
                print("[pedestrian] LOST: actor no longer exists")
                self.phase = "lost"
            return py_trees.common.Status.RUNNING
        if self.phase == "walking":
            distance = approach_distance(self.ego, self.wmap, self.site, self.trigger)
            if distance is not None:
                # Cross laterally from the current sidewalk walking position.
                offset = dot(self.actor.get_location() - self.sidewalk.transform.location,
                             self.site.transform.get_forward_vector())
                fwd = self.site.transform.get_forward_vector()
                endpoint = self.end + carla.Location(x=fwd.x * offset, y=fwd.y * offset)
                refuge = self.wmap.get_waypoint(endpoint, lane_type=carla.LaneType.Any)
                if (refuge is None or lane_key(refuge) != self.refuge_lane
                        or refuge.lane_type not in (carla.LaneType.Sidewalk, carla.LaneType.Shoulder)
                        or refuge.transform.location.distance(endpoint) > refuge.lane_width / 2 + 0.2):
                    self.phase = "skipped"
                    self.actor.apply_control(carla.WalkerControl(speed=0))
                    print("[pedestrian] SKIPPED: shifted endpoint is outside the refuge")
                    return py_trees.common.Status.RUNNING
                self.end = endpoint
                self.phase, self.cross_started = "crossing", GameTime.get_time()
                print(f"[pedestrian] TRIGGERED at {distance:.1f} m; speed={self.run_speed:g} m/s")
            elif self.move_towards(self.walk_points[self.walk_index], self.walk_speed):
                self.walk_index = 1 - self.walk_index
        if self.phase == "crossing":
            if self.move_towards(self.end, self.run_speed):
                self.phase = "done"
                print("[pedestrian] COMPLETE: reached refuge")
            elif GameTime.get_time() - self.cross_started > 30:
                self.actor.apply_control(carla.WalkerControl(speed=0))
                self.phase = "blocked"
                print("[pedestrian] BLOCKED: crossing timed out; check physical obstacles")
        # Event completion does not end the whole city scenario or retrigger.
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if self.actor.is_alive:
            self.actor.apply_control(carla.WalkerControl(speed=0))


class CutInEvent(py_trees.behaviour.Behaviour):
    def __init__(self, world, ego, settings, spawn, traffic_manager):
        super().__init__("Vehicle cut-in")
        self.world, self.ego, self.tm = world, ego, traffic_manager
        self.wmap = world.get_map()
        self.site = site_waypoint(self.wmap, settings)
        side = settings.get("from_side", "left")
        if side not in ("left", "right"):
            raise ValueError("cut_in from_side must be left or right")
        self.source = getattr(self.site, f"get_{side}_lane")()
        if self.source is None or self.source.lane_type != carla.LaneType.Driving or dot(
            self.site.transform.get_forward_vector(), self.source.transform.get_forward_vector()
        ) < 0.95:
            raise ValueError("Cut-in requires an adjacent lane travelling in the SAME direction")
        self.direction = "right" if side == "left" else "left"
        self.trigger = number(settings, "trigger_distance", 16, 3, 60)
        self.start_distance = number(settings, "approach_distance", 35, self.trigger + 2, 100)
        self.approach_speed = number(settings, "approach_speed_kmh", 12, 1, 60) / 3.6
        self.merge_speed = number(settings, "speed_kmh", 25, 1, 80) / 3.6
        self.merge_length = number(settings, "lane_change_distance", 12, 6, 40)
        self.approach_length = number(settings, "max_approach_distance", 20, 1, 60)
        # Validate the entire manoeuvre corridor before creating the vehicle.
        lane_path(self.source, self.approach_length + self.merge_length + 16)
        lane_path(self.site, self.approach_length + self.merge_length + 16)
        self.merge_plan(self.source)
        self.actor = spawn("vehicle.tesla.model3", self.source.transform, "event_cut_in")
        self.actor.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
        self.phase, self.planner, self.started = "waiting", None, 0.0
        print(f"[cut_in] WAITING; target lane={lane_key(self.site)}, trigger <= {self.trigger:g} m")

    def merge_plan(self, source):
        plan, target = generate_target_waypoint_list_multilane(
            source, self.direction, distance_same_lane=0,
            distance_other_lane=15, total_lane_change_distance=self.merge_length, check=True,
        )
        if not plan or target != self.site.lane_id or any(
            wp.is_junction or wp.road_id != self.site.road_id for wp, _ in plan
        ):
            raise ValueError("No permitted cut-in path to the target lane at this position")
        return plan

    def set_plan(self, plan, speed):
        dt = self.world.get_settings().fixed_delta_seconds or 0.05
        self.planner = LocalPlanner(self.actor, opt_dict={
            "target_speed": speed * 3.6,
            "dt": dt,
            "lateral_control_dict": {"K_P": 1.95, "K_I": 0.05, "K_D": 0.2, "dt": dt},
            "longitudinal_control_dict": {"K_P": 1.0, "K_I": 0.05, "K_D": 0, "dt": dt},
            "max_throttle": 0.7,
        })
        self.planner.set_global_plan(plan, stop_waypoint_creation=True, clean_queue=True)

    def release(self, reason):
        self.phase = "done"
        self.actor.apply_control(carla.VehicleControl())
        self.actor.set_autopilot(True, self.tm.get_port())
        self.tm.auto_lane_change(self.actor, False)
        self.tm.ignore_lights_percentage(self.actor, 0)
        self.tm.ignore_signs_percentage(self.actor, 0)
        print(f"[cut_in] {reason}; handed back to normal Traffic Manager driving")

    def update(self):
        if not self.actor.is_alive:
            if self.phase != "lost":
                print("[cut_in] LOST: actor no longer exists")
                self.phase = "lost"
            return py_trees.common.Status.RUNNING
        if self.phase == "waiting":
            if approach_distance(self.ego, self.wmap, self.site, self.start_distance) is None:
                return py_trees.common.Status.RUNNING
            plan = [(wp, RoadOption.LANEFOLLOW) for wp in lane_path(self.source, self.approach_length)]
            self.set_plan(plan, self.approach_speed)
            self.phase, self.started = "approaching", GameTime.get_time()
            print("[cut_in] APPROACHING in adjacent lane")
        if self.phase == "approaching":
            current = self.wmap.get_waypoint(self.actor.get_location())
            target = getattr(current, f"get_{self.direction}_lane")() if current else None
            distance = None
            if target is not None and lane_key(target) == lane_key(self.site):
                distance = approach_distance(self.ego, self.wmap, target, self.trigger)
            if distance is not None:
                try:
                    self.set_plan(self.merge_plan(current), self.merge_speed)
                except ValueError as exc:
                    self.release(f"SKIPPED: {exc}")
                    return py_trees.common.Status.RUNNING
                self.phase, self.started = "merging", GameTime.get_time()
                print(f"[cut_in] TRIGGERED at {distance:.1f} m; merge length={self.merge_length:g} m")
            elif self.planner.done() or GameTime.get_time() - self.started > 20:
                self.release("SKIPPED: ego did not reach trigger before approach ended")
        if self.phase == "merging":
            wp = self.wmap.get_waypoint(self.actor.get_location())
            if wp is None:
                self.phase = "blocked"
                self.actor.apply_control(carla.VehicleControl(brake=1.0))
                print("[cut_in] BLOCKED: vehicle left the mapped road")
                return py_trees.common.Status.RUNNING
            offset = abs(dot(self.actor.get_location() - wp.transform.location,
                             wp.transform.get_right_vector()))
            aligned = dot(self.actor.get_transform().get_forward_vector(), wp.transform.get_forward_vector()) > 0.98
            if lane_key(wp) == lane_key(self.site) and offset < 0.4 and aligned:
                self.release("COMPLETE: entered ego lane")
            elif self.planner.done() or GameTime.get_time() - self.started > 15:
                self.release("INCOMPLETE: lane change timed out or path ended")
        if self.phase in ("approaching", "merging"):
            control = self.planner.run_step(debug=False)
            if self.phase == "approaching" and detect_lane_obstacle(self.actor):
                control.throttle, control.brake = 0.0, 1.0
            self.actor.apply_control(control)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if self.actor.is_alive and self.phase not in ("done", "lost"):
            self.actor.apply_control(carla.VehicleControl(brake=1.0))
