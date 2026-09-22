#!/usr/bin/env python3

"""Town05 city traffic scenario for an externally controlled ego vehicle."""

import random

import py_trees

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import Idle
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenarios.basic_scenario import BasicScenario


class Town05CityScenario(BasicScenario):
    """Keep ordinary Traffic Manager vehicles circulating around Town05."""

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=1800):
        self.timeout = self._get_parameter(config, "scenario_duration", float, timeout)
        self._vehicle_count = max(
            0, self._get_parameter(config, "vehicle_count", int, 30)
        )
        self._spawn_clearance = max(
            0.0, self._get_parameter(config, "spawn_clearance", float, 25.0)
        )
        self._following_distance = max(
            1.0, self._get_parameter(config, "following_distance", float, 3.0)
        )
        self._speed_difference = self._get_parameter(
            config, "speed_difference", float, 10.0
        )
        self._automatic_lane_change = self._get_bool_parameter(
            config, "automatic_lane_change", True
        )
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self._rng = random.Random(906 if not randomize else None)
        super().__init__(
            "Town05CityScenario",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable,
        )

    @staticmethod
    def _get_parameter(config, name, parameter_type, default):
        parameter = config.other_parameters.get(name, {})
        return parameter_type(parameter.get("value", default))

    @staticmethod
    def _get_bool_parameter(config, name, default):
        parameter = config.other_parameters.get(name, {})
        value = str(parameter.get("value", default)).strip().lower()
        return value in ("1", "true", "yes", "on")

    def _setup_scenario_trigger(self, config):
        # Start immediately after ScenarioRunner has found or spawned ego_vehicle.
        return None

    def _initialize_actors(self, config):
        ego_location = self.ego_vehicles[0].get_location()
        spawn_points = list(self._map.get_spawn_points())
        self._rng.shuffle(spawn_points)

        existing_background = [
            actor
            for actor in self._world.get_actors().filter("vehicle.*")
            if actor.id != self.ego_vehicles[0].id
            and actor.attributes.get("role_name") not in ("ego_vehicle", "hero")
        ]
        vehicles_to_spawn = max(0, self._vehicle_count - len(existing_background))

        tm_port = CarlaDataProvider.get_traffic_manager_port()
        traffic_manager = CarlaDataProvider.get_client().get_trafficmanager(tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(
            self._following_distance
        )
        traffic_manager.global_percentage_speed_difference(self._speed_difference)

        for actor in existing_background:
            traffic_manager.auto_lane_change(actor, self._automatic_lane_change)
            traffic_manager.distance_to_leading_vehicle(
                actor, self._following_distance
            )
            traffic_manager.ignore_lights_percentage(actor, 0.0)
            traffic_manager.ignore_signs_percentage(actor, 0.0)

        for spawn_point in spawn_points:
            if len(self.other_actors) >= vehicles_to_spawn:
                break
            if spawn_point.location.distance(ego_location) < self._spawn_clearance:
                continue

            actor = CarlaDataProvider.request_new_actor(
                "vehicle.*",
                spawn_point,
                rolename="background",
                autopilot=True,
                actor_category="car",
                attribute_filter={"base_type": "car"},
            )
            if actor is None:
                continue

            traffic_manager.auto_lane_change(actor, self._automatic_lane_change)
            traffic_manager.distance_to_leading_vehicle(
                actor, self._following_distance
            )
            traffic_manager.ignore_lights_percentage(actor, 0.0)
            traffic_manager.ignore_signs_percentage(actor, 0.0)
            self.other_actors.append(actor)

        print(
            "Town05 city traffic: kept {} and spawned {} background vehicles".format(
                len(existing_background), len(self.other_actors)
            )
        )

    def _create_behavior(self):
        return Idle(self.timeout, name="Town05 city traffic")

    def _create_test_criteria(self):
        return [CollisionTest(self.ego_vehicles[0])]

    def __del__(self):
        self.remove_all_actors()
