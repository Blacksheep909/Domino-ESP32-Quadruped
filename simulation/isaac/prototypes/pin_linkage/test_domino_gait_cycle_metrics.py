from __future__ import annotations

import unittest

import numpy as np

from domino_gait_cycle_metrics import GaitCycleTracker


class GaitCycleTrackerTests(unittest.TestCase):
    def make_tracker(self) -> GaitCycleTracker:
        return GaitCycleTracker(
            1,
            ["front_right", "front_left", "rear_left", "rear_right"],
            min_air_steps=2,
            touchdown_confirm_steps=2,
            min_clearance_m=0.004,
            min_body_relative_travel_m=0.020,
            max_tilt_deg=25.0,
            min_touchdown_support_feet=2,
        )

    def initialize(self, tracker: GaitCycleTracker) -> np.ndarray:
        positions = np.zeros((4, 3), dtype=np.float64)
        tracker.initialize_env(
            0,
            np.ones(4, dtype=bool),
            np.zeros(4, dtype=np.float64),
            positions,
        )
        return positions

    def test_counts_confirmed_liftoff_and_touchdown(self) -> None:
        tracker = self.make_tracker()
        positions = self.initialize(tracker)

        for step, x_position in enumerate((0.010, 0.030, 0.050), start=1):
            positions = positions.copy()
            positions[0, 0] = x_position
            tracker.update_env(
                0,
                step,
                np.array([False, True, True, True]),
                np.array([0.006, 0.0, 0.0, 0.0]),
                positions,
                body_tilt_deg=3.0,
                done=False,
            )
        for step in (4, 5):
            tracker.update_env(
                0,
                step,
                np.ones(4, dtype=bool),
                np.zeros(4, dtype=np.float64),
                positions,
                body_tilt_deg=3.0,
                done=False,
            )

        report = tracker.summary()
        self.assertEqual(report["total_valid_cycles"], 1)
        self.assertEqual(report["aggregate_valid_cycles_per_foot"], [1, 0, 0, 0])
        self.assertEqual(report["invalid_reason_counts"], {})

    def test_rejects_low_clearance_cycle(self) -> None:
        tracker = self.make_tracker()
        positions = self.initialize(tracker)

        for step, x_position in enumerate((0.010, 0.030, 0.050), start=1):
            positions = positions.copy()
            positions[0, 0] = x_position
            tracker.update_env(
                0,
                step,
                np.array([False, True, True, True]),
                np.array([0.002, 0.0, 0.0, 0.0]),
                positions,
                body_tilt_deg=3.0,
                done=False,
            )
        for step in (4, 5):
            tracker.update_env(
                0,
                step,
                np.ones(4, dtype=bool),
                np.zeros(4, dtype=np.float64),
                positions,
                body_tilt_deg=3.0,
                done=False,
            )

        report = tracker.summary()
        self.assertEqual(report["total_completed_cycles"], 1)
        self.assertEqual(report["total_valid_cycles"], 0)
        self.assertEqual(report["invalid_reason_counts"]["low_clearance"], 1)

    def test_done_aborts_incomplete_swing(self) -> None:
        tracker = self.make_tracker()
        positions = self.initialize(tracker)
        for step in (1, 2):
            positions = positions.copy()
            positions[0, 0] += 0.010
            tracker.update_env(
                0,
                step,
                np.array([False, True, True, True]),
                np.array([0.006, 0.0, 0.0, 0.0]),
                positions,
                body_tilt_deg=3.0,
                done=False,
            )
        tracker.update_env(
            0,
            3,
            np.ones(4, dtype=bool),
            np.zeros(4, dtype=np.float64),
            positions,
            body_tilt_deg=30.0,
            done=True,
        )

        report = tracker.summary()
        self.assertEqual(report["total_completed_cycles"], 0)
        self.assertEqual(report["aborted_swings_per_env_foot"][0][0], 1)


if __name__ == "__main__":
    unittest.main()
