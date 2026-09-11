# Copyright 2026 selfcar contributors
# SPDX-License-Identifier: Apache-2.0

"""ROS-independent policy. All times are ROS/simulation seconds."""

from dataclasses import dataclass
import math
from typing import Optional


@dataclass(frozen=True)
class Settings:
    obstacle_timeout: float = 10.0
    standstill_timeout: float = 20.0
    short_green_window: float = 10.0
    stopped_velocity: float = 0.1
    message_timeout: float = 0.5
    forget_after: float = 2.0


@dataclass(frozen=True)
class Observation:
    now: float
    active: bool = True
    speed: Optional[float] = 0.0
    # None means stale/missing; an empty tuple is a fresh empty perception frame.
    objects: Optional[tuple[bytes, ...]] = ()
    obstacle_stop: bool = False
    signal_wait: Optional[bool] = False
    signal_id: Optional[int] = None
    signal_phase: str = "other"


@dataclass(frozen=True)
class Exclusion:
    reason: str
    object_ids: tuple[bytes, ...]


class RecoveryPolicy:
    def __init__(self, settings: Settings = Settings()):
        self.settings = settings
        self.reset()

    def reset(self):
        self.ignored: dict[bytes, float] = {}
        self.last_update: Optional[float] = None
        self.reset_wait()

    def reset_signal(self):
        self.signal_id = None
        self.previous_phase = "other"
        self.green_since = None
        self.saw_red = False

    def reset_wait(self):
        self.stopped_since = None
        self.obstacle_since = None
        self.obstacle_episode = False
        self.standstill_fired = False
        self.reset_signal()

    @staticmethod
    def elapsed(now, since):
        return 0.0 if since is None else max(0.0, now - since)

    def _short_green(self, obs):
        if obs.signal_id is None or obs.signal_phase == "unknown":
            self.reset_signal()
            return False
        if obs.signal_id != self.signal_id:
            self.reset_signal()
            self.signal_id = obs.signal_id
        rapid_red = False
        if obs.signal_phase == "red":
            rapid_red = (
                self.green_since is not None
                and self.elapsed(obs.now, self.green_since) <= self.settings.short_green_window
                and self.obstacle_episode
            )
            self.saw_red = True
            self.green_since = None
        elif obs.signal_phase == "green":
            if self.green_since is None and self.saw_red:
                self.green_since = obs.now
        # An intervening amber phase (or one asynchronous factor/light frame) does
        # not erase a measured green interval. It cannot start that interval either.
        # Missing/UNKNOWN data and a changed signal ID are reset above.
        self.previous_phase = obs.signal_phase
        return rapid_red

    def update(self, obs: Observation) -> Optional[Exclusion]:
        if not math.isfinite(obs.now) or not obs.active:
            self.reset()
            return None
        if self.last_update is not None:
            if obs.now < self.last_update:
                self.reset()
            elif obs.now - self.last_update > self.settings.message_timeout:
                self.reset_wait()
        self.last_update = obs.now

        # Excluded UUIDs remain excluded while visible, including after motion resumes.
        # New UUIDs are never covered by an old exclusion event.
        if obs.objects is not None:
            visible = set(obs.objects)
            for uid, last_seen in list(self.ignored.items()):
                if uid in visible:
                    self.ignored[uid] = obs.now
                elif obs.now - last_seen >= self.settings.forget_after:
                    del self.ignored[uid]

        if (
            obs.speed is None
            or not math.isfinite(obs.speed)
            or abs(obs.speed) > self.settings.stopped_velocity
        ):
            self.reset_wait()
            return None

        if self.stopped_since is None:
            self.stopped_since = obs.now
        self.obstacle_episode |= obs.obstacle_stop
        rapid_red = self._short_green(obs)

        if obs.obstacle_stop and obs.signal_wait is False and obs.objects is not None:
            if self.obstacle_since is None:
                self.obstacle_since = obs.now
        else:
            self.obstacle_since = None

        if obs.objects is None:
            return None
        reason = None
        if (
            not self.standstill_fired
            and self.elapsed(obs.now, self.stopped_since) + 1e-8
            >= self.settings.standstill_timeout
        ):
            reason = "standstill_20s"
            self.standstill_fired = True
        elif rapid_red:
            reason = "red_green_red_within_10s"
        elif (
            self.obstacle_since is not None
            and self.elapsed(obs.now, self.obstacle_since) + 1e-8
            >= self.settings.obstacle_timeout
        ):
            reason = "obstacle_stop_10s"

        if reason is None:
            return None
        added = tuple(sorted(set(obs.objects) - self.ignored.keys()))
        for uid in obs.objects:
            self.ignored[uid] = obs.now
        # A newly appearing obstacle must earn its own full timeout.
        self.obstacle_since = None
        return Exclusion(reason, added)
