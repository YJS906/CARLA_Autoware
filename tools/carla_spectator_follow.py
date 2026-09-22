#!/usr/bin/env python3
import argparse
import math
import sys
import time

import carla


def find_ego(world, role_name):
    for actor in world.get_actors().filter("vehicle.*"):
        if actor.attributes.get("role_name") == role_name:
            return actor
    return None


def compute_spectator_transform(ego_transform, distance, height, pitch_deg):
    yaw_rad = math.radians(ego_transform.rotation.yaw)
    location = carla.Location(
        x=ego_transform.location.x - distance * math.cos(yaw_rad),
        y=ego_transform.location.y - distance * math.sin(yaw_rad),
        z=ego_transform.location.z + height,
    )
    rotation = carla.Rotation(
        pitch=pitch_deg,
        yaw=ego_transform.rotation.yaw,
        roll=0.0,
    )
    return carla.Transform(location, rotation)


def main():
    parser = argparse.ArgumentParser(
        description="Lock the CARLA spectator camera to the ego vehicle."
    )
    parser.add_argument("--host", default="localhost", help="CARLA server host")
    parser.add_argument("--port", type=int, default=2000, help="CARLA server RPC port")
    parser.add_argument("--timeout", type=float, default=10.0, help="Client timeout (s)")
    parser.add_argument(
        "--role", default="ego_vehicle", help="role_name attribute of the ego actor"
    )
    parser.add_argument(
        "--distance", type=float, default=8.0, help="Meters behind the ego (use 0 for top-down)"
    )
    parser.add_argument("--height", type=float, default=4.0, help="Meters above the ego")
    parser.add_argument(
        "--pitch", type=float, default=-15.0, help="Camera pitch in degrees (negative looks down)"
    )
    parser.add_argument("--rate", type=float, default=30.0, help="Update rate in Hz")
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = None
    spectator = None
    next_world_check = 0.0

    period = 1.0 / args.rate if args.rate > 0 else 0.033
    print(
        f"Following actor role_name='{args.role}' on {args.host}:{args.port} "
        f"(distance={args.distance}m, height={args.height}m, pitch={args.pitch}deg)",
        flush=True,
    )

    ego = None
    try:
        while True:
            try:
                # The bridge may load another map after this process starts.
                # World/Actor handles belong to one episode and must be replaced.
                now = time.monotonic()
                if world is None or now >= next_world_check:
                    current_world = client.get_world()
                    if world is None or current_world.id != world.id:
                        world = current_world
                        spectator = world.get_spectator()
                        ego = None
                        print(f"Connected to CARLA world {world.id}", flush=True)
                    next_world_check = now + 1.0

                snapshot = world.get_snapshot()
                if snapshot.id != world.id:
                    world = None
                    time.sleep(period)
                    continue
                if ego is None or not ego.is_alive:
                    ego = find_ego(world, args.role)
                    if ego is None:
                        time.sleep(0.5)
                        continue
                    print(f"Tracking actor id={ego.id} type={ego.type_id}", flush=True)

                # Newly connected clients can have no actor snapshot yet. Using
                # Actor.get_transform() then returns a misleading zero pose.
                ego_snapshot = snapshot.find(ego.id)
                if ego_snapshot is None:
                    time.sleep(period)
                    continue
                spectator.set_transform(
                    compute_spectator_transform(
                        ego_snapshot.get_transform(), args.distance, args.height, args.pitch
                    )
                )
            except RuntimeError as exc:
                print(f"RPC error, reconnecting camera: {exc}", file=sys.stderr, flush=True)
                world = None
                spectator = None
                ego = None
                time.sleep(0.5)
                continue

            time.sleep(period)
    except KeyboardInterrupt:
        print("\nStopped.", flush=True)
        return 0


if __name__ == "__main__":
    sys.exit(main())
