#!/usr/bin/env python3
"""Spawn continuously roaming pedestrians after the Autoware ego is ready.

NPC vehicles are created by autoware_carla_interface so their Traffic Manager
runs in the same synchronous CARLA client that advances the simulation.
"""

import argparse
import random
import time

import carla


def wait_for_ego(client):
    while True:
        try:
            world = client.get_world()
            world.wait_for_tick(5.0)
            if any(
                actor.attributes.get("role_name") == "ego_vehicle"
                for actor in world.get_actors().filter("vehicle.*")
            ):
                return world
        except RuntimeError:
            pass
        time.sleep(1.0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--walkers", type=int, default=30)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    random.seed(args.seed)
    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(120.0)
    world = wait_for_ego(client)
    world.set_pedestrians_seed(args.seed)

    # Let the perception stack finish its first startup before adding actors.
    time.sleep(5.0)

    walker_ids = []
    controller_ids = []
    speeds = {}
    try:
        blueprints = world.get_blueprint_library().filter("walker.pedestrian.*")
        spawn_batch = []
        pending_speeds = []
        for _ in range(args.walkers):
            location = world.get_random_location_from_navigation()
            if location is None:
                continue
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute("is_invincible"):
                blueprint.set_attribute("is_invincible", "false")
            recommended = blueprint.get_attribute("speed").recommended_values
            speed = float(recommended[1]) if len(recommended) > 1 else 1.4
            spawn_batch.append(carla.command.SpawnActor(blueprint, carla.Transform(location)))
            pending_speeds.append(speed)

        for result, speed in zip(client.apply_batch_sync(spawn_batch, False), pending_speeds):
            if not result.error:
                walker_ids.append(result.actor_id)
                speeds[result.actor_id] = speed

        controller_bp = world.get_blueprint_library().find("controller.ai.walker")
        controller_batch = [
            carla.command.SpawnActor(controller_bp, carla.Transform(), walker_id)
            for walker_id in walker_ids
        ]
        controller_for_walker = {}
        for walker_id, result in zip(
            walker_ids, client.apply_batch_sync(controller_batch, False)
        ):
            if not result.error:
                controller_ids.append(result.actor_id)
                controller_for_walker[walker_id] = result.actor_id

        world.wait_for_tick(120.0)
        pairs = []
        for walker_id, controller_id in controller_for_walker.items():
            controller = world.get_actor(controller_id)
            walker = world.get_actor(walker_id)
            if controller is None or walker is None:
                continue
            controller.start()
            target = world.get_random_location_from_navigation()
            if target is not None:
                controller.go_to_location(target)
            controller.set_max_speed(speeds[walker_id])
            pairs.append((walker, controller))

        print(f"spawned {len(pairs)} continuously roaming walkers", flush=True)
        tick_count = 0
        while True:
            world.wait_for_tick(120.0)
            tick_count += 1
            # Refresh destinations so the city stays active after arrival.
            if tick_count % 600 == 0:
                for _, controller in pairs:
                    target = world.get_random_location_from_navigation()
                    if target is not None:
                        controller.go_to_location(target)
    finally:
        for controller_id in controller_ids:
            controller = world.get_actor(controller_id)
            if controller is not None:
                controller.stop()
        client.apply_batch(
            [carla.command.DestroyActor(actor_id) for actor_id in controller_ids + walker_ids]
        )


if __name__ == "__main__":
    main()
