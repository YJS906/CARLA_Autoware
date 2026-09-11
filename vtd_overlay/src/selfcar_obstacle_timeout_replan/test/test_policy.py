# Copyright 2026 selfcar contributors
# SPDX-License-Identifier: Apache-2.0

import unittest

from selfcar_obstacle_timeout_replan.policy import Observation, RecoveryPolicy, Settings


A, B, C = (bytes([i]) * 16 for i in (1, 2, 3))


class TestRecoveryPolicy(unittest.TestCase):
    def setUp(self):
        self.policy = RecoveryPolicy()

    def feed(self, start, end, **kwargs):
        events = []
        for tenth in range(round(start * 10), round(end * 10) + 1):
            result = self.policy.update(Observation(now=tenth / 10, **kwargs))
            if result is not None:
                events.append((tenth / 10, result))
        return events

    def test_ten_seconds_even_without_a_candidate(self):
        self.assertEqual(self.feed(0, 9.9, obstacle_stop=True, objects=(A, B)), [])
        self.assertEqual(self.policy.ignored, {})
        result = self.feed(10, 10, obstacle_stop=True, objects=(A, B))[0][1]
        self.assertEqual(result.reason, "obstacle_stop_10s")
        self.assertEqual(set(result.object_ids), {A, B})

    def test_rtc_acceptance_does_not_prove_vehicle_motion(self):
        # There is deliberately no RTC approval input to this clock: WAITING and
        # RUNNING-but-still-stopped have identical physical recovery behavior.
        self.feed(0, 7, obstacle_stop=True, objects=(A,))
        events = self.feed(7.1, 10, obstacle_stop=True, objects=(A,))
        self.assertEqual(events[0][0], 10)

    def test_red_waits_until_twenty_seconds(self):
        options = dict(obstacle_stop=True, objects=(A,), signal_wait=True,
                       signal_id=100, signal_phase="red")
        self.assertEqual(self.feed(0, 19.9, **options), [])
        event = self.feed(20, 20, **options)[0][1]
        self.assertEqual(event.reason, "standstill_20s")
        self.assertEqual(event.object_ids, (A,))
        self.assertEqual(self.feed(20.1, 25, **options), [])

    def test_red_green_red_short_window(self):
        options = dict(obstacle_stop=True, objects=(A,), signal_id=100)
        self.feed(0, 2, signal_wait=True, signal_phase="red", **options)
        self.feed(2.1, 9, signal_wait=False, signal_phase="green", **options)
        event = self.feed(9.1, 9.1, signal_wait=True, signal_phase="red", **options)[0][1]
        self.assertEqual(event.reason, "red_green_red_within_10s")

    def test_different_light_is_not_short_green_cycle(self):
        self.feed(0, 1, objects=(A,), obstacle_stop=True, signal_id=100,
                  signal_phase="red", signal_wait=True)
        self.feed(1.1, 3, objects=(A,), obstacle_stop=True, signal_id=100,
                  signal_phase="green")
        self.assertEqual(self.feed(3.1, 4, objects=(A,), obstacle_stop=True,
                                  signal_id=200, signal_phase="red", signal_wait=True), [])

    def test_amber_between_green_and_red_keeps_measured_interval(self):
        options = dict(obstacle_stop=True, objects=(A,), signal_id=100)
        self.feed(0, 1, signal_wait=True, signal_phase="red", **options)
        self.feed(1.1, 3, signal_wait=False, signal_phase="green", **options)
        self.feed(3.1, 4, signal_wait=True, signal_phase="other", **options)
        result = self.feed(4.1, 4.1, signal_wait=True, signal_phase="red", **options)
        self.assertEqual(result[0][1].reason, "red_green_red_within_10s")

    def test_green_seen_without_prior_red_is_not_a_cycle(self):
        self.feed(0, 3, objects=(A,), obstacle_stop=True, signal_id=100, signal_phase="green")
        self.assertEqual(self.feed(3.1, 4, objects=(A,), obstacle_stop=True,
                                  signal_id=100, signal_phase="red", signal_wait=True), [])

    def test_long_green_is_not_short_cycle(self):
        self.feed(0, 1, objects=(A,), obstacle_stop=True, signal_id=100,
                  signal_phase="red", signal_wait=True)
        # The obstacle stop writer can disappear when another stop owns the output.
        self.feed(1.1, 12, objects=(A,), signal_id=100, signal_phase="green")
        self.assertEqual(self.feed(12.1, 12.1, objects=(A,), signal_id=100,
                                  signal_phase="red", signal_wait=True), [])

    def test_unknown_and_flashing_do_not_count_as_green(self):
        for phase in ("unknown", "other"):
            with self.subTest(phase=phase):
                self.policy.reset()
                self.feed(0, 1, objects=(A,), obstacle_stop=True, signal_id=100,
                          signal_phase="red", signal_wait=True)
                self.feed(1.1, 3, objects=(A,), signal_id=100, signal_phase=phase)
                self.assertEqual(self.feed(3.1, 4, objects=(A,), signal_id=100,
                                          signal_phase="red", signal_wait=True), [])

    def test_twenty_clock_survives_stop_reason_changes(self):
        self.feed(0, 5, objects=(A,), obstacle_stop=True)
        self.feed(5.1, 12, objects=(A,), signal_wait=True, signal_phase="red", signal_id=100)
        self.assertEqual(self.feed(12.1, 19.9, objects=(A,)), [])
        self.assertEqual(self.feed(20, 20, objects=(A,))[0][1].reason, "standstill_20s")

    def test_real_motion_resets_both_clocks_but_keeps_excluded_uuid(self):
        self.feed(0, 10, objects=(A,), obstacle_stop=True)
        self.feed(10.1, 11, objects=(A, B), speed=1.0)
        self.assertEqual(set(self.policy.ignored), {A})
        self.assertEqual(self.feed(11.1, 21, objects=(A, B), obstacle_stop=True), [])
        self.assertEqual(self.feed(21.1, 21.1, objects=(A, B), obstacle_stop=True)[0][1].object_ids, (B,))

    def test_new_uuid_is_not_inherited_and_absent_uuid_expires(self):
        self.feed(0, 10, objects=(A,), obstacle_stop=True)
        self.feed(10.1, 11.9, objects=(B,), speed=1.0)
        self.assertEqual(set(self.policy.ignored), {A})
        self.feed(12, 12, objects=(B,), speed=1.0)
        self.assertEqual(self.policy.ignored, {})

    def test_disabled_manual_or_arrived_restores_original_objects(self):
        self.feed(0, 10, objects=(A,), obstacle_stop=True)
        self.feed(10.1, 10.1, active=False, objects=(A,))
        self.assertEqual(self.policy.ignored, {})
        self.assertEqual(self.feed(10.2, 40, active=False, objects=(A,), obstacle_stop=True), [])

    def test_stale_odometry_does_not_accumulate(self):
        self.feed(0, 9, objects=(A,), obstacle_stop=True)
        self.feed(9.1, 20, objects=(A,), obstacle_stop=True, speed=None)
        self.assertEqual(self.policy.ignored, {})

    def test_stale_signal_blocks_ten_seconds(self):
        self.assertEqual(self.feed(0, 19.9, objects=(A,), obstacle_stop=True, signal_wait=None), [])
        self.assertEqual(self.feed(20, 20, objects=(A,), signal_wait=None)[0][1].reason,
                         "standstill_20s")

    def test_stale_objects_are_not_excluded_until_a_fresh_snapshot(self):
        self.feed(0, 20, objects=None, signal_wait=True)
        self.assertEqual(self.policy.ignored, {})
        self.assertEqual(self.feed(20.1, 20.1, objects=(B,), signal_wait=True)[0][1].object_ids, (B,))

    def test_paused_sim_time_does_not_count_wall_time(self):
        self.feed(0, 9, objects=(A,), obstacle_stop=True)
        for _ in range(1000):
            self.assertIsNone(self.policy.update(Observation(9, objects=(A,), obstacle_stop=True)))
        self.assertEqual(self.policy.ignored, {})

    def test_backward_clock_jump_clears_snapshot_and_wait(self):
        self.feed(0, 10, objects=(A,), obstacle_stop=True)
        self.policy.update(Observation(2, objects=(A,), obstacle_stop=True))
        self.assertEqual(self.policy.ignored, {})
        self.assertEqual(self.policy.stopped_since, 2)

    def test_missing_stop_factor_and_forward_gap_reset_ten_seconds(self):
        self.feed(0, 8, objects=(A,), obstacle_stop=True)
        self.feed(8.1, 8.5, objects=(A,))
        self.assertEqual(self.feed(8.6, 17, objects=(A,), obstacle_stop=True), [])
        self.assertIsNone(self.policy.update(Observation(19, objects=(A,), obstacle_stop=True)))
        self.assertEqual(self.policy.stopped_since, 19)


if __name__ == "__main__":
    unittest.main()
