#!/usr/bin/env python3
"""Maintain bounded city traffic beside the existing CARLA/Autoware ticker.

Only actors spawned by this process are modified or removed. Configuration is
reloaded without restarting the simulator. No operation requests a world tick.
"""

import argparse
from collections import deque
import fcntl
import json
import math
import os
from pathlib import Path
import random
import signal
import time

import carla


DEFAULTS = dict(vehicles=100, walkers=200, vehicle_speed_difference_percent=30,
                following_distance_m=3, auto_lane_change=False, crossing_factor=0.15,
                vehicle_batch_size=16, walker_batch_size=24, update_interval_s=2,
                ego_clearance_m=15, walker_speed_min_mps=1, walker_speed_max_mps=1.6)
RANGES = dict(vehicles=(0, 300), walkers=(0, 800),
              vehicle_speed_difference_percent=(0, 90), following_distance_m=(2, 20),
              crossing_factor=(0, 1), vehicle_batch_size=(1, 50), walker_batch_size=(1, 50),
              update_interval_s=(1, 30), ego_clearance_m=(10, 100),
              walker_speed_min_mps=(0.5, 2), walker_speed_max_mps=(0.5, 2))
INTEGER_KEYS = {"vehicles", "walkers", "vehicle_batch_size", "walker_batch_size"}
EGO_ROLES = {"ego_vehicle", "hero", "ego"}
ORDINARY_ROLES = {"", "autopilot", "background", "background_vehicle", "traffic", "npc"}
ROLE = "dense_city_npc"


def read_config(path):
    raw = json.loads(Path(path).read_text())
    if not isinstance(raw, dict) or set(raw) - set(DEFAULTS):
        raise ValueError("configuration must be an object with only documented keys")
    config = dict(DEFAULTS, **raw)
    for key, (low, high) in RANGES.items():
        value = config[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{key} must be numeric")
        if not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f"{key} must be between {low} and {high}")
        if key in INTEGER_KEYS and not isinstance(value, int):
            raise ValueError(f"{key} must be an integer")
    if not isinstance(config["auto_lane_change"], bool):
        raise ValueError("auto_lane_change must be boolean")
    if config["walker_speed_min_mps"] > config["walker_speed_max_mps"]:
        raise ValueError("minimum walker speed exceeds maximum")
    return config


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + f".{os.getpid()}.tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def distance2(a, b):
    return (a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2


def ordinary_vehicle(actor):
    role = actor.attributes.get("role_name", "")
    return actor.type_id.startswith("vehicle.") and (role in ORDINARY_ROLES or role == ROLE)


def speed(actor):
    velocity = actor.get_velocity()
    return math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)


def ownership_summary(visible_ids, owned_ids):
    """Actor registration may lag spawning; count identities, never subtract totals."""
    visible_ids, owned_ids = set(visible_ids), set(owned_ids)
    return dict(nonowned=len(visible_ids - owned_ids),
                owned_visible=len(visible_ids & owned_ids),
                owned_not_visible=sorted(owned_ids - visible_ids))


class CityTraffic:
    def __init__(self, args):
        self.args, self.config = args, read_config(args.config)
        self.rng = random.Random(args.seed)
        self.running, self.status = True, "starting"
        self.errors, self.rates = deque(maxlen=30), deque(maxlen=60)
        self.spawn_failures = {"vehicles": 0, "walkers": 0, "controllers": 0, "navigation": 0}
        self.vehicles, self.walkers, self.controllers = set(), {}, set()
        self.configured_external = set()
        self.previous_walker_positions = {}
        self.next_update, self.config_text = 0, Path(args.config).read_text()
        self.client = carla.Client(args.host, args.port)
        self.client.set_timeout(10)
        self.world = self.client.get_world()
        self.world_id = self.world.id
        self.map = self.world.get_map()
        if self.map.name.rsplit("/", 1)[-1] not in {"Town05", "Town05_Opt"}:
            raise RuntimeError(f"native Town05 required, found {self.map.name}")
        self.tm = self.client.get_trafficmanager(args.traffic_manager_port)
        library = self.world.get_blueprint_library()
        self.vehicle_blueprints = [bp for bp in library.filter("vehicle.*")
                                   if bp.has_attribute("number_of_wheels")
                                   and int(bp.get_attribute("number_of_wheels")) == 4
                                   and not bp.id.endswith(("carlacola", "cybertruck", "firetruck"))]
        self.walker_blueprints = list(library.filter("walker.pedestrian.*"))
        self.controller_blueprint = library.find("controller.ai.walker")
        self.spawn_points = []
        for transform in self.map.get_spawn_points():
            waypoint = self.map.get_waypoint(transform.location, project_to_road=False,
                                             lane_type=carla.LaneType.Driving)
            if waypoint and not waypoint.is_junction:
                self.spawn_points.append(transform)
        if not self.spawn_points or not self.vehicle_blueprints or not self.walker_blueprints:
            raise RuntimeError("native vehicle spawn points or blueprints unavailable")
        self.apply_config()

    def error(self, message):
        self.errors.append({"time": time.time(), "message": str(message)})
        print(str(message), flush=True)

    def apply_config(self):
        # This is the pedestrian navigation policy, never a world/ticker setting.
        self.world.set_pedestrians_cross_factor(self.config["crossing_factor"])
        for actor_id in self.vehicles:
            actor = self.world.get_actor(actor_id)
            if actor:
                self.configure_vehicle(actor)
        for info in self.walkers.values():
            controller = self.world.get_actor(info["controller"]) if info["controller"] else None
            if controller:
                controller.set_max_speed(min(self.config["walker_speed_max_mps"],
                                             max(self.config["walker_speed_min_mps"], info["speed"])))
        self.configured_external.clear()

    def reload(self):
        try:
            current = Path(self.args.config).read_text()
            if current != self.config_text:
                new = read_config(self.args.config)
                self.config = new
                self.apply_config()
                self.config_text = current
        except (OSError, ValueError) as exc:
            self.error(f"configuration rejected; retaining last valid configuration: {exc}")
        except RuntimeError as exc:
            self.error(f"valid configuration application interrupted; retrying next update: {exc}")

    def configure_vehicle(self, actor):
        self.tm.ignore_lights_percentage(actor, 0)
        self.tm.ignore_signs_percentage(actor, 0)
        self.tm.ignore_vehicles_percentage(actor, 0)
        self.tm.ignore_walkers_percentage(actor, 0)
        self.tm.distance_to_leading_vehicle(actor, self.config["following_distance_m"])
        self.tm.vehicle_percentage_speed_difference(actor, self.config["vehicle_speed_difference_percent"])
        self.tm.auto_lane_change(actor, self.config["auto_lane_change"])

    def destroy(self, ids):
        """The caller supplies owned IDs; refuse all others even during cleanup."""
        owned = self.vehicles | set(self.walkers) | self.controllers
        ids = [actor_id for actor_id in ids if actor_id in owned]
        for offset in range(0, len(ids), 24):
            for actor_id in ids[offset:offset + 24]:
                if actor_id in self.controllers:
                    actor = self.world.get_actor(actor_id)
                    if actor:
                        try:
                            actor.stop()
                        except RuntimeError as exc:
                            self.error(f"controller {actor_id} stop failed; still removing owned actor: {exc}")
            batch = [carla.command.DestroyActor(actor_id) for actor_id in ids[offset:offset + 24]]
            for actor_id, result in zip(ids[offset:offset + 24], self.client.apply_batch_sync(batch, False)):
                if (result.error and "not found" not in result.error.lower()
                        and self.world.get_actor(actor_id)):
                    self.error(f"cannot remove owned actor {actor_id}: {result.error}")
                    continue
                self.vehicles.discard(actor_id)
                self.controllers.discard(actor_id)
                self.walkers.pop(actor_id, None)

    def spawn_vehicles(self, count, actors, ego_locations):
        occupied = [actor.get_location() for actor in actors if actor.type_id.startswith("vehicle.")]
        points = list(self.spawn_points)
        self.rng.shuffle(points)
        batch = []
        for transform in points:
            location = transform.location
            if any(distance2(location, ego) < self.config["ego_clearance_m"] ** 2 for ego in ego_locations):
                continue
            if any(distance2(location, other) < 9 ** 2 for other in occupied):
                continue
            blueprint = self.rng.choice(self.vehicle_blueprints)
            blueprint.set_attribute("role_name", ROLE)
            if blueprint.has_attribute("color"):
                blueprint.set_attribute("color", self.rng.choice(blueprint.get_attribute("color").recommended_values))
            batch.append(carla.command.SpawnActor(blueprint, transform))
            occupied.append(location)
            if len(batch) >= count:
                break
        if not batch:
            return
        for result in self.client.apply_batch_sync(batch, False):
            if result.error:
                self.spawn_failures["vehicles"] += 1
                continue
            self.vehicles.add(result.actor_id)
            actor = self.world.get_actor(result.actor_id)
            try:
                if actor is None:
                    raise RuntimeError("spawned actor missing")
                self.configure_vehicle(actor)
                actor.set_autopilot(True, self.args.traffic_manager_port)
            except RuntimeError as exc:
                self.error(f"vehicle initialization failed: {exc}")
                self.destroy([result.actor_id])

    def spawn_walkers(self, count, actors, ego_locations):
        occupied = [actor.get_location() for actor in actors
                    if actor.type_id.startswith(("walker.", "vehicle."))]
        batch, speeds = [], []
        for _ in range(count * 5):
            location = self.world.get_random_location_from_navigation()
            if location is None:
                self.spawn_failures["navigation"] += 1
                continue
            if any(distance2(location, ego) < self.config["ego_clearance_m"] ** 2 for ego in ego_locations):
                continue
            if any(distance2(location, other) < 2 ** 2 for other in occupied):
                continue
            blueprint = self.rng.choice(self.walker_blueprints)
            if blueprint.has_attribute("role_name"):
                blueprint.set_attribute("role_name", ROLE)
            if blueprint.has_attribute("is_invincible"):
                blueprint.set_attribute("is_invincible", "false")
            batch.append(carla.command.SpawnActor(blueprint, carla.Transform(location)))
            speeds.append(self.rng.uniform(self.config["walker_speed_min_mps"], self.config["walker_speed_max_mps"]))
            occupied.append(location)
            if len(batch) >= count:
                break
        if not batch:
            return
        newborn = []
        for result, velocity in zip(self.client.apply_batch_sync(batch, False), speeds):
            if result.error:
                self.spawn_failures["walkers"] += 1
                continue
            self.walkers[result.actor_id] = dict(controller=None, speed=velocity, target=None,
                                                  last_move=time.monotonic(), location=None)
            newborn.append(result.actor_id)
        commands = [carla.command.SpawnActor(self.controller_blueprint, carla.Transform(), actor_id)
                    for actor_id in newborn]
        if not commands:
            return
        for walker_id, result in zip(newborn, self.client.apply_batch_sync(commands, False)):
            if result.error:
                self.spawn_failures["controllers"] += 1
                self.destroy([walker_id])
                continue
            self.controllers.add(result.actor_id)
            self.walkers[walker_id]["controller"] = result.actor_id
        self.world.wait_for_tick(3)
        for walker_id in newborn:
            if walker_id not in self.walkers:
                continue
            info = self.walkers[walker_id]
            try:
                controller = self.world.get_actor(info["controller"])
                if controller is None:
                    raise RuntimeError("spawned walker controller missing")
                controller.start()
                target = self.world.get_random_location_from_navigation()
                if target is None:
                    raise RuntimeError("no pedestrian navigation destination")
                controller.go_to_location(target)
                controller.set_max_speed(info["speed"])
                info["target"] = target
            except RuntimeError as exc:
                self.error(f"walker initialization failed: {exc}")
                self.destroy([info["controller"], walker_id])

    def maintain_walkers(self, now):
        # Bounded destination refresh; stuck agents keep their collision handling.
        refreshed = 0
        for walker_id, info in list(self.walkers.items()):
            walker = self.world.get_actor(walker_id)
            controller = self.world.get_actor(info["controller"]) if info["controller"] else None
            if walker is None or controller is None:
                self.destroy([info["controller"], walker_id])
                continue
            location = walker.get_location()
            if info["location"] is None or distance2(location, info["location"]) > 0.4 ** 2:
                info["last_move"], info["location"] = now, location
            arrived = info["target"] is None or distance2(location, info["target"]) < 2 ** 2
            if refreshed < 12 and (arrived or now - info["last_move"] > 25):
                target = self.world.get_random_location_from_navigation()
                if target:
                    controller.go_to_location(target)
                    info["target"], info["last_move"] = target, now
                    refreshed += 1

    def reconcile(self, actors):
        ids = {actor.id for actor in actors}
        self.vehicles.intersection_update(ids)
        egos = [actor.get_location() for actor in actors if actor.attributes.get("role_name") in EGO_ROLES]
        if not egos:
            self.status = "waiting_for_ego"
            return
        self.status = "running"
        # Existing bridge background actors retain ownership and autopilot state.
        # Only their explicit Traffic Manager road-rule parameters are aligned.
        for actor in actors:
            if (actor.type_id.startswith("vehicle.") and actor.id not in self.vehicles
                    and actor.id not in self.configured_external
                    and actor.attributes.get("role_name", "") in {"", "autopilot"}):
                self.configure_vehicle(actor)
                self.configured_external.add(actor.id)
        for kind, own, present, batch_size in (
            ("vehicles", self.vehicles, sum(ordinary_vehicle(a) for a in actors), self.config["vehicle_batch_size"]),
            ("walkers", set(self.walkers), sum(a.type_id.startswith("walker.pedestrian.") for a in actors), self.config["walker_batch_size"]),
        ):
            difference = self.config[kind] - present
            if difference < 0:
                selected = sorted(own, reverse=True)[:min(-difference, batch_size)]
                if kind == "walkers":
                    self.destroy([self.walkers[actor_id]["controller"] for actor_id in selected])
                self.destroy(selected)
            elif difference > 0:
                getattr(self, "spawn_" + kind)(min(difference, batch_size), actors, egos)

    def walker_motion(self, walkers, snapshot):
        """AI navigation can move walkers while reporting zero velocity.

        Use two consistent CARLA snapshots and simulation time for pedestrian
        motion statistics; keep the velocity API measurement separately.
        """
        elapsed = snapshot.timestamp.elapsed_seconds
        current, velocities = {}, []
        for walker in walkers:
            sample = snapshot.find(walker.id)
            if sample is None:
                continue
            location = sample.get_transform().location
            previous = self.previous_walker_positions.get(walker.id)
            if previous and elapsed > previous[0]:
                velocities.append(math.sqrt(distance2(location, previous[1])) / (elapsed - previous[0]))
            current[walker.id] = (elapsed, location)
        self.previous_walker_positions = current
        return velocities

    def write_state(self, snapshot=None):
        # Reconcile may spawn actors after the loop's original snapshot.
        snapshot = self.world.get_snapshot()
        actors = list(self.world.get_actors())
        vehicles = [actor for actor in actors if ordinary_vehicle(actor)]
        walkers = [actor for actor in actors if actor.type_id.startswith("walker.pedestrian.")]
        walker_reported_speeds = [speed(actor) for actor in walkers]
        speeds = {"vehicles": [speed(actor) for actor in vehicles],
                  "walkers": self.walker_motion(walkers, snapshot)}
        first = self.rates[0] if self.rates else (0, 0, 0)
        last = self.rates[-1] if self.rates else first
        elapsed = last[0] - first[0]
        counts = dict(vehicles=len(vehicles), walkers=len(walkers),
                      all_vehicles=sum(a.type_id.startswith("vehicle.") for a in actors),
                      egos=sum(a.attributes.get("role_name") in EGO_ROLES for a in actors),
                      controllers=sum(a.type_id == "controller.ai.walker" for a in actors))
        ownership = {
            "vehicles": ownership_summary((actor.id for actor in vehicles), self.vehicles),
            "walkers": ownership_summary((actor.id for actor in walkers), self.walkers),
            "controllers": ownership_summary((actor.id for actor in actors
                                              if actor.type_id == "controller.ai.walker"), self.controllers),
        }
        atomic_json(self.args.state, dict(
            pid=os.getpid(), updated_at=time.time(), status=self.status, world_id=self.world_id,
            map=self.map.name, frame=snapshot.frame if snapshot else last[1], config=self.config,
            counts=counts, owned=dict(vehicles=sorted(self.vehicles), walkers=sorted(self.walkers),
                                      controllers=sorted(self.controllers)),
            nonowned={kind: values["nonowned"] for kind, values in ownership.items()},
            owned_visible_counts={kind: values["owned_visible"] for kind, values in ownership.items()},
            owned_not_visible={kind: values["owned_not_visible"] for kind, values in ownership.items()},
            target_shortfall={kind: max(0, self.config[kind] - counts[kind]) for kind in ("vehicles", "walkers")},
            moving={kind: sum(value > 0.2 for value in values) for kind, values in speeds.items()},
            mean_speed_mps={kind: sum(values) / len(values) if values else 0 for kind, values in speeds.items()},
            walker_speed_source="snapshot_position_delta_per_simulation_second",
            walker_speed_sample_count=len(speeds["walkers"]),
            walker_reported_mean_speed_mps=(sum(walker_reported_speeds) / len(walker_reported_speeds)
                                            if walker_reported_speeds else 0),
            world_fps=(last[1] - first[1]) / elapsed if elapsed > 0 else None,
            simulation_speed_ratio=(last[2] - first[2]) / elapsed if elapsed > 0 else None,
            measurement_window_s=elapsed, native_vehicle_spawn_points=len(self.spawn_points),
            spawn_failures=self.spawn_failures, errors=list(self.errors), tick_owner=False))

    def run(self):
        while self.running:
            try:
                snapshot = self.world.wait_for_tick(3)
            except RuntimeError as exc:
                self.error(f"waiting for existing simulator ticker: {exc}")
                continue
            now = time.monotonic()
            if now < self.next_update:
                continue
            try:
                current_world_id = self.client.get_world().id
            except RuntimeError as exc:
                self.error(f"cannot verify CARLA world identity; waiting: {exc}")
                self.next_update = now + self.config["update_interval_s"]
                continue
            if current_world_id != self.world_id:
                self.status = "world_changed"
                atomic_json(self.args.state, dict(pid=os.getpid(), updated_at=time.time(),
                            status=self.status, previous_world_id=self.world_id,
                            current_world_id=current_world_id, tick_owner=False,
                            errors=list(self.errors)))
                raise RuntimeError("CARLA world changed; refusing to reuse actor IDs")
            self.rates.append((now, snapshot.frame, snapshot.timestamp.elapsed_seconds))
            try:
                self.reload()
                self.maintain_walkers(now)
                self.reconcile(list(self.world.get_actors()))
                self.write_state(snapshot)
            except RuntimeError as exc:
                self.status = "retrying_rpc"
                self.error(f"traffic update failed; retrying without ticking: {exc}")
            self.next_update = now + self.config["update_interval_s"]

    def cleanup(self):
        self.status = "stopped"
        if self.client.get_world().id != self.world_id:
            self.error("world changed: old actors are gone; no new-world actors will be touched")
            return
        self.destroy(sorted(self.controllers))
        self.destroy(sorted(self.walkers))
        self.destroy(sorted(self.vehicles))
        # Actor-list caches can retain destroyed actors until an external tick.
        # Observe that tick; never advance the world ourselves during cleanup.
        try:
            self.world.wait_for_tick(3)
            remaining_ids = {actor.id for actor in self.world.get_actors()}
            self.vehicles.intersection_update(remaining_ids)
            self.controllers.intersection_update(remaining_ids)
            self.walkers = {actor_id: info for actor_id, info in self.walkers.items()
                            if actor_id in remaining_ids}
            if self.vehicles or self.controllers or self.walkers:
                self.status = "cleanup_incomplete"
                self.error("owned actors remain after cleanup; see owned IDs in state")
        except RuntimeError as exc:
            self.status = "cleanup_unverified"
            self.error(f"could not observe cleanup on the existing ticker: {exc}")
        if not self.rates:
            snapshot = self.world.get_snapshot()
            self.rates.append((time.monotonic(), snapshot.frame, snapshot.timestamp.elapsed_seconds))
        self.write_state()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--state", required=True, type=Path)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--traffic-manager-port", type=int, default=8000)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    args.state.parent.mkdir(parents=True, exist_ok=True)
    with args.state.with_suffix(args.state.suffix + ".lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        traffic = CityTraffic(args)
        def stop(_signum, _frame):
            traffic.running = False
        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)
        try:
            traffic.run()
        finally:
            traffic.cleanup()


if __name__ == "__main__":
    main()
