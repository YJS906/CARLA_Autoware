"""Owned CARLA actors for externally driven, closed-loop validation.

This module has no entry point. The caller supplies the world, ego and snapshots,
and owns simulation timing. It never ticks, changes world settings or controls ego.
Pass snapshots from the caller's normal loop to ``update`` after spawning. Wait
for ``vehicles_settled`` before starting a parked-obstacle trial. Distances and
speeds are metres and metres/second; collision timestamps are simulation seconds.
"""

import math
import threading
import weakref

import carla


class ActorFixture:
    """Collision sensor plus explicitly owned parked vehicles and pedestrians.

    ``actor_ids`` remains available after cleanup. ``collisions`` returns a copy
    suitable for JSON logs. ``pedestrian_states`` exposes waiting/crossing/stopped
    state and actual travelled distance. Actors are never selected for cleanup by
    blueprint, role name or proximity: only handles spawned by this instance are
    destroyed.
    """

    def __init__(self, world, ego):
        if not ego.is_alive:
            raise ValueError("Cannot attach a validation fixture to a dead ego actor")
        self.world = world
        self.ego = ego
        self.actor_ids = []
        self._owned = {}
        self._vehicles = {}
        self._pedestrians = {}
        self._collision_records = []
        self._lock = threading.Lock()
        self._last_frame = None
        self._last_time = None
        self._closed = False
        self._sensor = self._spawn("sensor.other.collision", carla.Transform(), attach_to=ego)
        fixture_ref = weakref.ref(self)

        def record(event):
            fixture = fixture_ref()
            if fixture is not None:
                fixture._record_collision(event)

        try:
            self._sensor.listen(record)
        except Exception as error:
            self._discard_after_failure([self._sensor.id], error)

    def _spawn(self, blueprint_id, transform, **kwargs):
        if self._closed:
            raise RuntimeError("This validation fixture has been cleaned up")
        try:
            blueprint = self.world.get_blueprint_library().find(blueprint_id)
            if blueprint.has_attribute("role_name"):
                blueprint.set_attribute("role_name", "validation_fixture")
            if blueprint.has_attribute("is_invincible"):
                blueprint.set_attribute("is_invincible", "false")
            actor = self.world.try_spawn_actor(blueprint, transform, **kwargs)
        except Exception as error:
            raise RuntimeError(f"Could not spawn {blueprint_id} at {transform}: {error}") from error
        if actor is None:
            raise RuntimeError(
                f"Could not spawn {blueprint_id} at {transform}; the location may be occupied"
            )
        self.actor_ids.append(actor.id)
        self._owned[actor.id] = actor
        return actor

    def parked_vehicle(self, transform):
        """Spawn a Prius with physics, service brake and handbrake applied.

        Spawning is asynchronous with respect to physics. ``vehicles_settled``
        becomes true only after subsequent snapshots show every parked vehicle
        stationary for at least 0.5 simulation seconds.
        """
        actor = self._spawn("vehicle.toyota.prius", transform)
        try:
            actor.set_simulate_physics(True)
            actor.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))
        except Exception as error:
            self._discard_after_failure([actor.id], error)
        self._vehicles[actor.id] = {"stationary_since": None, "settled": False}
        return actor

    def roadblock(self, transforms):
        """Spawn three parked Priuses; undo this call's actors if any spawn fails."""
        transforms = list(transforms)
        if len(transforms) != 3:
            raise ValueError("roadblock requires exactly three supplied transforms")
        created = []
        try:
            for transform in transforms:
                created.append(self.parked_vehicle(transform))
        except Exception as error:
            self._discard_after_failure([actor.id for actor in created], error)
        return created

    def pedestrian(self, transform, direction, speed, trigger_distance, *, travel_distance=4.0):
        """Spawn a waiting pedestrian, then cross once ego comes within range.

        ``direction`` is a CARLA Vector3D in world coordinates. Its horizontal
        component is normalized. The pedestrian stops after ``travel_distance``
        along this direction and remains there until cleanup. Choose the spawn
        and distance so that this endpoint lies on the intended road lane.
        """
        values = (direction.x, direction.y, direction.z, speed, trigger_distance, travel_distance)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Pedestrian direction and distances must be finite")
        length = math.hypot(direction.x, direction.y)
        if length <= 1e-9 or min(speed, trigger_distance, travel_distance) <= 0:
            raise ValueError("Pedestrian direction, speed and distances must be positive/nonzero")
        direction = carla.Vector3D(direction.x / length, direction.y / length, 0.0)
        actor = self._spawn("walker.pedestrian.0001", transform)
        try:
            actor.apply_control(carla.WalkerControl(direction=direction, speed=0.0))
        except Exception as error:
            self._discard_after_failure([actor.id], error)
        self._pedestrians[actor.id] = {
            "state": "waiting", "direction": direction, "speed": speed,
            "trigger_distance": trigger_distance, "travel_distance": travel_distance,
            "origin": None, "distance": 0.0, "trigger_frame": None,
            "trigger_time": None, "stop_frame": None, "stop_time": None,
        }
        return actor

    @property
    def vehicles_settled(self):
        return all(state["settled"] for state in self._vehicles.values())

    @property
    def pedestrian_states(self):
        return {
            actor_id: {key: state[key] for key in (
                "state", "distance", "trigger_frame", "trigger_time", "stop_frame", "stop_time"
            )}
            for actor_id, state in self._pedestrians.items()
        }

    @property
    def collisions(self):
        with self._lock:
            return [dict(record, normal_impulse=dict(record["normal_impulse"]))
                    for record in self._collision_records]

    def _record_collision(self, event):
        impulse = event.normal_impulse
        record = {
            "frame": event.frame, "timestamp": event.timestamp,
            "ego_id": self.ego.id, "other_actor_id": event.other_actor.id,
            "other_actor_type": event.other_actor.type_id,
            "normal_impulse": {"x": impulse.x, "y": impulse.y, "z": impulse.z},
            "impulse_magnitude": math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2),
        }
        with self._lock:
            self._collision_records.append(record)

    def update(self, snapshot):
        """Observe settling and advance crossing controls using one supplied frame.

        A just-spawned actor may be absent from a cached snapshot. Such actors
        are skipped until a later snapshot; their existence is not guessed from
        immediate RPC/cache visibility. A world/frame reset requires a new fixture.
        """
        if self._closed:
            raise RuntimeError("Cannot update a cleaned-up validation fixture")
        frame, now = snapshot.frame, snapshot.timestamp.elapsed_seconds
        if self._last_frame is not None:
            if frame < self._last_frame or now < self._last_time:
                raise RuntimeError("Simulation frame/time reset; discard this fixture before continuing")
            if frame == self._last_frame:
                return
        self._last_frame, self._last_time = frame, now
        ego_snapshot = snapshot.find(self.ego.id)
        for actor_id, state in self._vehicles.items():
            observed = snapshot.find(actor_id)
            if observed is None:
                state.update(stationary_since=None, settled=False)
                continue
            velocity, angular = observed.get_velocity(), observed.get_angular_velocity()
            stationary = velocity.length() < 0.05 and angular.length() < 0.5
            if not stationary:
                state.update(stationary_since=None, settled=False)
            else:
                if state["stationary_since"] is None:
                    state["stationary_since"] = now
                state["settled"] = now - state["stationary_since"] >= 0.5
        for actor_id, state in self._pedestrians.items():
            observed = snapshot.find(actor_id)
            if observed is None:
                continue
            location = observed.get_transform().location
            actor = self._owned[actor_id]
            if state["state"] == "waiting" and ego_snapshot is not None:
                ego_location = ego_snapshot.get_transform().location
                if location.distance(ego_location) < state["trigger_distance"]:
                    actor.apply_control(carla.WalkerControl(
                        direction=state["direction"], speed=state["speed"]
                    ))
                    state.update(state="crossing", origin=location,
                                 trigger_frame=frame, trigger_time=now)
            if state["state"] == "crossing":
                offset = location - state["origin"]
                state["distance"] = offset.x * state["direction"].x + offset.y * state["direction"].y
                if state["distance"] >= state["travel_distance"]:
                    actor.apply_control(carla.WalkerControl(direction=state["direction"], speed=0.0))
                    state.update(state="stopped", stop_frame=frame, stop_time=now)

    def _destroy(self, actor_ids):
        errors = []
        for actor_id in reversed(actor_ids):
            actor = self._owned.get(actor_id)
            if actor is None:
                continue
            try:
                if actor.is_alive:
                    if actor.type_id == "sensor.other.collision" and actor.is_listening:
                        actor.stop()
                    if not actor.destroy() and actor.is_alive:
                        raise RuntimeError("destroy returned false while actor remains alive")
            except Exception as error:
                errors.append(f"actor {actor_id}: {error}")
                continue
            self._owned.pop(actor_id, None)
            self._vehicles.pop(actor_id, None)
            self._pedestrians.pop(actor_id, None)
        if errors:
            raise RuntimeError("Fixture cleanup incomplete; retry cleanup: " + "; ".join(errors))

    def _discard_after_failure(self, actor_ids, error):
        try:
            self._destroy(actor_ids)
        except Exception as cleanup_error:
            raise RuntimeError(f"Fixture setup failed: {error}; {cleanup_error}") from error
        raise RuntimeError(f"Fixture setup failed: {error}") from error

    def cleanup(self):
        """Destroy only this fixture's actors. Safe to repeat, including after failure."""
        self._closed = True
        self._destroy(list(self._owned))
