#!/usr/bin/env python3
"""Run a VTD 2025.2 scenario directly in CARLA 0.9.16.

This is intentionally a direct adapter instead of an OpenSCENARIO converter:
the source file uses VIRES' proprietary ``<Scenario>`` format.  The adapter
keeps the source XML as the scenario definition and translates its coordinates,
actors, proximity triggers, paths, and signal-controller phases at runtime.
"""

from __future__ import annotations

import argparse
import json
import math
import random
import signal
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import carla


DEFAULT_SCENARIO = Path(__file__).resolve().parents[1] / "config/carla/scenarios/HL_FMA_VTD_YJS01.xml"
DEFAULT_XODR = Path(__file__).resolve().parents[1] / "config/carla/maps/HL_FMA_VTD_LivingLab.xodr"
ROLE_PREFIX = "vtd_scenario:"


def number(node: ET.Element | None, key: str, default: float = 0.0) -> float:
    return float(node.get(key, default)) if node is not None else default


def vtd_location(node: ET.Element) -> carla.Location:
    """VTD/OpenDRIVE is right-handed; CARLA's Unreal coordinates flip Y."""
    return carla.Location(x=number(node, "X"), y=-number(node, "Y"), z=number(node, "Z"))


def vtd_transform(node: ET.Element, z_offset: float = 0.0) -> carla.Transform:
    loc = vtd_location(node)
    loc.z += z_offset
    return carla.Transform(
        loc,
        carla.Rotation(
            pitch=math.degrees(number(node, "Pitch")),
            yaw=-math.degrees(number(node, "Direction", number(node, "Yaw"))),
            roll=-math.degrees(number(node, "Roll")),
        ),
    )


@dataclass
class Trigger:
    location: carla.Location
    radius: float
    pivot: str
    on_enter: bool
    was_inside: bool = False


@dataclass
class VehicleAction:
    actor_name: str
    trigger: Trigger
    target_speed_mps: float | None = None
    lane_direction: int | None = None
    fired: bool = False


@dataclass
class WalkerAction:
    actor_name: str
    trigger: Trigger
    speed_mps: float
    path_id: str
    loop: bool
    fired: bool = False


@dataclass
class WalkerMotion:
    actor: carla.Actor
    points: list[carla.Location]
    speed_mps: float
    loop: bool
    index: int = 0


@dataclass
class Scenario:
    root: ET.Element
    paths: dict[str, list[carla.Location]] = field(default_factory=dict)
    vehicle_actions: list[VehicleAction] = field(default_factory=list)
    walker_actions: list[WalkerAction] = field(default_factory=list)

    @classmethod
    def read(cls, path: Path) -> "Scenario":
        root = ET.parse(path).getroot()
        if root.tag != "Scenario":
            raise ValueError(f"{path} is not a VTD Scenario XML")
        data = cls(root=root)
        for shape in root.findall("./MovingObjectsControl/PathShape"):
            data.paths[shape.get("ShapeId", "")] = [vtd_location(p) for p in shape.findall("Waypoint")]
        for group in root.findall("./TrafficControl/PlayerActions"):
            name = group.get("Player", "")
            for action in group.findall("Action"):
                trigger = cls._trigger(action)
                if trigger is None:
                    continue
                speed = action.find("SpeedChange")
                lane = action.find("LaneChange")
                data.vehicle_actions.append(
                    VehicleAction(
                        actor_name=name,
                        trigger=trigger,
                        target_speed_mps=number(speed, "Target") if speed is not None else None,
                        lane_direction=int(number(lane, "Direction")) if lane is not None else None,
                    )
                )
        for group in root.findall("./MovingObjectsControl/CharacterActions"):
            name = group.get("Character", "")
            for action in group.findall("Action"):
                trigger = cls._trigger(action)
                motion = action.find("Motion")
                path_ref = action.find("CharacterPath")
                if trigger is None or motion is None or path_ref is None:
                    continue
                data.walker_actions.append(
                    WalkerAction(
                        actor_name=name,
                        trigger=trigger,
                        speed_mps=max(0.0, number(motion, "Speed")),
                        path_id=path_ref.get("PathShape", ""),
                        loop=path_ref.get("Loop", "false").lower() == "true",
                    )
                )
        return data

    @staticmethod
    def _trigger(action: ET.Element) -> Trigger | None:
        pos = action.find("PosAbsolute")
        if pos is None:
            return None
        active = None
        for child in action:
            if child.tag != "PosAbsolute" and "ActiveOnEnter" in child.attrib:
                active = child.get("ActiveOnEnter", "true").lower() == "true"
                break
        return Trigger(vtd_location(pos), number(pos, "Radius", 1.0), pos.get("Pivot", "Ego"), True if active is None else active)

    def audit(self) -> dict[str, object]:
        players = self.root.findall("./TrafficControl/Player")
        characters = self.root.findall("./MovingObjectsControl/Character")
        objects = self.root.findall("./MovingObjectsControl/Object")
        controllers = self.root.findall("./LightSigns/SignalController")
        signals = self.root.findall("./LightSigns/Signal")
        return {
            "format": f"VTD {self.root.get('RevMajor')}.{self.root.get('RevMinor')}",
            "players": len(players),
            "ego_players": sum(p.find("Description").get("Control") == "external" for p in players),
            "npc_vehicles": sum(p.find("Description").get("Control") != "external" for p in players),
            "characters": len(characters),
            "objects": len(objects),
            "path_shapes": len(self.paths),
            "vehicle_actions": len(self.vehicle_actions),
            "character_actions": len(self.walker_actions),
            "signal_controllers": len(controllers),
            "signal_lamps": len(signals),
            "coordinate_transform": "CARLA x=VTD x, y=-VTD y, yaw_deg=-degrees(VTD direction)",
        }


class Runtime:
    def __init__(self, args: argparse.Namespace, scenario: Scenario):
        self.args = args
        self.scenario = scenario
        self.client = carla.Client(args.host, args.port)
        self.client.set_timeout(args.timeout)
        self.world: carla.World | None = None
        self.tm = self.client.get_trafficmanager(args.traffic_manager_port)
        self.actors: dict[str, carla.Actor] = {}
        self.owned: list[carla.Actor] = []
        self.walkers: list[WalkerMotion] = []
        self.signal_actors: list[carla.TrafficLight] = []
        self.signal_programs: list[tuple[carla.TrafficLight, list[tuple[float, carla.TrafficLightState]], float]] = []
        self.running = True

    def prepare_world(self) -> None:
        if self.args.load_map:
            text = self.args.xodr.read_text(encoding="utf-8")
            params = carla.OpendriveGenerationParameters(
                vertex_distance=2.0,
                max_road_length=50.0,
                wall_height=0.0,
                additional_width=0.6,
                smooth_junctions=True,
                enable_mesh_visibility=True,
            )
            print(f"Loading OpenDRIVE: {self.args.xodr}", flush=True)
            self.world = self.client.generate_opendrive_world(text, params)
        else:
            self.world = self.client.get_world()
            if not self.world.get_map().name.endswith("OpenDriveMap"):
                raise RuntimeError("the current CARLA world is not OpenDriveMap; add --load-map")
        self.cleanup_previous()
        self.focus_spectator_on_start()
        print(f"OpenDRIVE ready: {self.world.get_map().name}", flush=True)

    def focus_spectator_on_start(self) -> None:
        """Put the camera above the VTD ego start while an external ego is pending."""
        assert self.world is not None
        ego = next(
            (
                player
                for player in self.scenario.root.findall("./TrafficControl/Player")
                if player.find("Description").get("Control") == "external"
            ),
            None,
        )
        if ego is None:
            return
        position = ego.find("./Init/PosAbsolute")
        transform = vtd_transform(position)
        transform.location.z += 45.0
        transform.rotation.pitch = -62.0
        self.world.get_spectator().set_transform(transform)

    def cleanup_previous(self) -> None:
        assert self.world is not None
        stale = [a for a in self.world.get_actors() if a.attributes.get("role_name", "").startswith(ROLE_PREFIX)]
        if stale:
            self.client.apply_batch_sync([carla.command.DestroyActor(a.id) for a in stale], False)

    @staticmethod
    def _set_role(blueprint: carla.ActorBlueprint, name: str) -> None:
        if blueprint.has_attribute("role_name"):
            blueprint.set_attribute("role_name", ROLE_PREFIX + name)

    def _vehicle_blueprint(self, source_type: str, index: int) -> carla.ActorBlueprint:
        assert self.world is not None
        lib = self.world.get_blueprint_library()
        mapping = [
            (("Actros", "Truck"), "vehicle.carlamotors.carlacola"),
            (("Bus",), "vehicle.mitsubishi.fusorosa"),
            (("BMW",), "vehicle.bmw.grandtourer"),
            (("Audi",), "vehicle.audi.a2"),
            (("VW", "Volkswagen"), "vehicle.volkswagen.t2"),
            (("Hyundai",), "vehicle.hyundai.ioniq"),
        ]
        for words, target in mapping:
            if any(word.lower() in source_type.lower() for word in words):
                matches = lib.filter(target)
                if len(matches):
                    return matches[0]
        choices = list(lib.filter("vehicle.*"))
        choices = [b for b in choices if not b.has_attribute("number_of_wheels") or int(b.get_attribute("number_of_wheels")) >= 4]
        return choices[index % len(choices)]

    def _spawn(self, blueprint: carla.ActorBlueprint, transform: carla.Transform, name: str) -> carla.Actor | None:
        assert self.world is not None
        self._set_role(blueprint, name)
        actor = None
        base_z = transform.location.z
        # VTD permits denser initial placement than CARLA's collision gate.
        # Retrying slightly above the same XY pose keeps the scenario location;
        # physics settles the actor onto the generated road immediately.
        for extra_z in (0.0, 0.75, 1.5, 3.0):
            transform.location.z = base_z + extra_z
            actor = self.world.try_spawn_actor(blueprint, transform)
            if actor is not None:
                break
        if actor is not None:
            self.actors[name] = actor
            self.owned.append(actor)
        return actor

    def find_or_spawn_ego(self) -> carla.Actor:
        assert self.world is not None
        deadline = time.monotonic() + self.args.wait_timeout if self.args.wait_timeout > 0 else None
        next_notice = 0.0
        while True:
            for actor in self.world.get_actors().filter("vehicle.*"):
                if actor.attributes.get("role_name") == self.args.ego_role_name:
                    self.actors["Ego"] = actor
                    print(f"Connected to ego '{self.args.ego_role_name}' (actor {actor.id})", flush=True)
                    return actor
            if self.args.spawn_ego:
                ego = next(
                    (
                        player
                        for player in self.scenario.root.findall("./TrafficControl/Player")
                        if player.find("Description").get("Control") == "external"
                    ),
                    None,
                )
                if ego is None:
                    raise RuntimeError("external VTD ego is missing")
                pos = ego.find("./Init/PosAbsolute")
                bp = self.world.get_blueprint_library().find("vehicle.tesla.model3")
                bp.set_attribute("role_name", self.args.ego_role_name)
                actor = self.world.try_spawn_actor(bp, vtd_transform(pos, 0.5))
                if actor is None:
                    raise RuntimeError("could not spawn preview ego")
                self.actors["Ego"] = actor
                self.owned.append(actor)
                print(f"Spawned preview ego (actor {actor.id})", flush=True)
                return actor
            now = time.monotonic()
            if not self.args.wait_for_ego or (deadline is not None and now >= deadline):
                raise RuntimeError(f"ego role '{self.args.ego_role_name}' was not found")
            if now >= next_notice:
                print(
                    f"Waiting for Autoware ego role '{self.args.ego_role_name}' ... "
                    "launch autoware_carla_interface, or use --spawn-ego for preview",
                    flush=True,
                )
                next_notice = now + 10.0
            time.sleep(0.5)

    def follow_ego_with_spectator(self) -> None:
        assert self.world is not None
        ego = self.actors.get("Ego")
        if ego is None or not ego.is_alive:
            return
        transform = ego.get_transform()
        yaw = math.radians(transform.rotation.yaw)
        transform.location.x -= 11.0 * math.cos(yaw)
        transform.location.y -= 11.0 * math.sin(yaw)
        transform.location.z += 5.0
        transform.rotation.pitch = -18.0
        self.world.get_spectator().set_transform(transform)

    def _route_transform(self, player: ET.Element) -> carla.Transform | None:
        assert self.world is not None
        ref = player.find("./Init/PathRef")
        if ref is None:
            return None
        path = self.scenario.root.find(f"./TrafficControl/Path[@PathId='{ref.get('PathId')}']")
        if path is None or not list(path):
            return None
        first = path.find("Waypoint")
        road_id = int(first.get("TrackId"))
        lane_id = int(ref.get("StartLane", "-1"))
        start_s = number(ref, "StartS")
        waypoint = self.world.get_map().get_waypoint_xodr(road_id, lane_id, start_s)
        if waypoint is None:
            return None
        transform = waypoint.transform
        transform.location.z += 0.5
        return transform

    def spawn_vehicles(self) -> tuple[int, int]:
        assert self.world is not None
        spawned = failed = 0
        for index, player in enumerate(self.scenario.root.findall("./TrafficControl/Player")):
            desc = player.find("Description")
            if desc.get("Control") == "external":
                continue
            name = desc.get("Name", f"vehicle_{index}")
            pos = player.find("./Init/PosAbsolute")
            transform = vtd_transform(pos, 0.5) if pos is not None else self._route_transform(player)
            if transform is None:
                print(f"WARN vehicle has no usable position: {name}", file=sys.stderr)
                failed += 1
                continue
            bp = self._vehicle_blueprint(desc.get("Type", ""), index)
            actor = self._spawn(bp, transform, name)
            if actor is None:
                failed += 1
                continue
            spawned += 1
            initial_speed = number(player.find("./Init/Speed"), "Value")
            has_action = any(a.actor_name == name for a in self.scenario.vehicle_actions)
            if desc.get("Driver") != "No Driver" or initial_speed > 0 or has_action:
                actor.set_autopilot(True, self.args.traffic_manager_port)
                self.tm.set_desired_speed(actor, max(0.0, initial_speed * 3.6))
        return spawned, failed

    def spawn_characters(self) -> tuple[int, int]:
        assert self.world is not None
        blueprints = list(self.world.get_blueprint_library().filter("walker.pedestrian.*"))
        spawned = failed = 0
        for index, char in enumerate(self.scenario.root.findall("./MovingObjectsControl/Character")):
            name = char.get("Name", f"walker_{index}")
            bp = blueprints[index % len(blueprints)]
            if self._spawn(bp, vtd_transform(char.find("StartPosAbs"), 0.2), name):
                spawned += 1
            else:
                failed += 1
        return spawned, failed

    def spawn_objects(self) -> tuple[int, int]:
        assert self.world is not None
        lib = self.world.get_blueprint_library()
        cones = list(lib.filter("static.prop.trafficcone*"))
        barriers = list(lib.filter("static.prop.streetbarrier"))
        spawned = failed = 0
        for index, obj in enumerate(self.scenario.root.findall("./MovingObjectsControl/Object")):
            definition = obj.get("Definition", "")
            choices = cones if "Pylon" in definition else barriers
            if not choices:
                failed += 1
                continue
            if self._spawn(choices[index % len(choices)], vtd_transform(obj.find("StartPosAbs"), 0.05), obj.get("Name", f"object_{index}")):
                spawned += 1
            else:
                failed += 1
        return spawned, failed

    def configure_signals(self) -> tuple[int, int, int]:
        assert self.world is not None
        xroot = ET.parse(self.args.xodr).getroot()
        vtd_programs = {x.get("Id"): x for x in self.scenario.root.findall("./LightSigns/SignalController")}
        resolved = missing = 0
        for controller in xroot.findall("controller"):
            program = vtd_programs.get(controller.get("id"))
            if program is None:
                missing += 1
                continue
            light = None
            for control in controller.findall("control"):
                try:
                    light = self.world.get_traffic_light_from_opendrive_id(control.get("signalId"))
                except RuntimeError:
                    light = None
                if light is not None:
                    break
            if light is None:
                missing += 1
                continue
            phases = []
            states = {"go": carla.TrafficLightState.Green, "attention": carla.TrafficLightState.Yellow, "stop": carla.TrafficLightState.Red}
            for phase in program.findall("Phase"):
                phases.append((number(phase, "Duration"), states.get(phase.get("Type"), carla.TrafficLightState.Red)))
            if not phases:
                missing += 1
                continue
            light.freeze(True)
            self.signal_actors.append(light)
            self.signal_programs.append((light, phases, number(program, "Delay")))
            resolved += 1
        vtd_only = len(vtd_programs) - resolved - missing
        return resolved, missing, vtd_only

    def _trigger_fires(self, trigger: Trigger) -> bool:
        pivot = self.actors.get(trigger.pivot)
        if pivot is None or not pivot.is_alive:
            return False
        p = pivot.get_location()
        inside = math.hypot(p.x - trigger.location.x, p.y - trigger.location.y) <= trigger.radius
        fires = inside and not trigger.was_inside if trigger.on_enter else trigger.was_inside and not inside
        trigger.was_inside = inside
        return fires

    def update_actions(self) -> None:
        for action in self.scenario.vehicle_actions:
            if action.fired or not self._trigger_fires(action.trigger):
                continue
            actor = self.actors.get(action.actor_name)
            if actor is None:
                continue
            actor.set_autopilot(True, self.args.traffic_manager_port)
            if action.target_speed_mps is not None:
                self.tm.set_desired_speed(actor, action.target_speed_mps * 3.6)
            if action.lane_direction is not None:
                self.tm.force_lane_change(actor, action.lane_direction > 0)
            action.fired = True
            print(f"vehicle action: {action.actor_name}", flush=True)
        for action in self.scenario.walker_actions:
            if action.fired or not self._trigger_fires(action.trigger):
                continue
            actor = self.actors.get(action.actor_name)
            points = self.scenario.paths.get(action.path_id, [])
            if actor is not None and len(points) >= 2 and action.speed_mps > 0:
                self.walkers.append(WalkerMotion(actor, points, action.speed_mps, action.loop))
            action.fired = True
            print(f"character action: {action.actor_name}", flush=True)

    def update_walkers(self) -> None:
        active = []
        for motion in self.walkers:
            if not motion.actor.is_alive:
                continue
            current = motion.actor.get_location()
            target = motion.points[motion.index]
            dx, dy, dz = target.x - current.x, target.y - current.y, target.z - current.z
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            if distance < 0.8:
                motion.index += 1
                if motion.index >= len(motion.points):
                    if motion.loop:
                        motion.index = 0
                    else:
                        motion.actor.apply_control(carla.WalkerControl())
                        continue
                target = motion.points[motion.index]
                dx, dy, dz = target.x - current.x, target.y - current.y, target.z - current.z
                distance = max(0.001, math.sqrt(dx * dx + dy * dy + dz * dz))
            motion.actor.apply_control(carla.WalkerControl(carla.Vector3D(dx / distance, dy / distance, dz / distance), motion.speed_mps, False))
            active.append(motion)
        self.walkers = active

    def update_signals(self, elapsed: float) -> None:
        for light, phases, delay in self.signal_programs:
            period = sum(duration for duration, _ in phases)
            cursor = (elapsed + delay) % period
            for duration, state in phases:
                if cursor < duration:
                    light.set_state(state)
                    break
                cursor -= duration

    def run(self) -> None:
        assert self.world is not None
        start = None
        while self.running:
            snapshot = self.world.wait_for_tick(self.args.timeout)
            if snapshot is None:
                raise RuntimeError("timed out waiting for a CARLA tick")
            if start is None:
                start = snapshot.timestamp.elapsed_seconds
            elapsed = snapshot.timestamp.elapsed_seconds - start
            self.update_actions()
            self.update_walkers()
            self.update_signals(elapsed)
            self.follow_ego_with_spectator()
            if self.args.duration > 0 and elapsed >= self.args.duration:
                break

    def cleanup(self) -> None:
        for light in self.signal_actors:
            if light.is_alive:
                light.freeze(False)
        if self.args.keep_actors:
            return
        ids = [a.id for a in self.owned if a.is_alive]
        if ids:
            self.client.apply_batch_sync([carla.command.DestroyActor(i) for i in ids], False)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--scenario", type=Path, default=DEFAULT_SCENARIO)
    p.add_argument("--xodr", type=Path, default=DEFAULT_XODR)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=2000)
    p.add_argument("--timeout", type=float, default=20.0)
    p.add_argument("--load-map", action="store_true", help="generate the CARLA world from the bundled OpenDRIVE")
    p.add_argument("--spawn-ego", action="store_true", help="spawn a preview ego instead of waiting for Autoware")
    p.add_argument("--wait-for-ego", action="store_true", help="wait for the Autoware CARLA interface ego")
    p.add_argument("--wait-timeout", type=float, default=0.0, help="ego wait seconds; 0 waits indefinitely")
    p.add_argument("--ego-role-name", default="ego_vehicle")
    p.add_argument("--traffic-manager-port", type=int, default=8000)
    p.add_argument("--duration", type=float, default=0.0, help="stop after simulation seconds; 0 runs until Ctrl-C")
    p.add_argument("--keep-actors", action="store_true")
    p.add_argument("--dry-run", action="store_true")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    scenario = Scenario.read(args.scenario)
    audit = scenario.audit()
    if args.dry_run:
        print(json.dumps(audit, indent=2, ensure_ascii=False))
        return 0
    if not args.spawn_ego and not args.wait_for_ego:
        print("ERROR: select --spawn-ego for preview or --wait-for-ego for Autoware", file=sys.stderr)
        return 2
    runtime = Runtime(args, scenario)
    signal.signal(signal.SIGINT, lambda *_: setattr(runtime, "running", False))
    signal.signal(signal.SIGTERM, lambda *_: setattr(runtime, "running", False))
    try:
        runtime.prepare_world()
        ego = runtime.find_or_spawn_ego()
        runtime.tm.set_random_device_seed(20250921)
        runtime.tm.set_synchronous_mode(runtime.world.get_settings().synchronous_mode)
        vehicle_result = runtime.spawn_vehicles()
        character_result = runtime.spawn_characters()
        object_result = runtime.spawn_objects()
        signal_result = runtime.configure_signals()
        result = dict(audit)
        result.update(
            map=runtime.world.get_map().name,
            ego_id=ego.id,
            vehicles_spawned=vehicle_result[0],
            vehicles_failed=vehicle_result[1],
            characters_spawned=character_result[0],
            characters_failed=character_result[1],
            objects_spawned=object_result[0],
            objects_failed=object_result[1],
            signal_controllers_mapped=signal_result[0],
            xodr_signal_controllers_unmapped=signal_result[1],
            vtd_only_signal_controllers=signal_result[2],
        )
        print(json.dumps(result, indent=2, ensure_ascii=False), flush=True)
        runtime.run()
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        runtime.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
