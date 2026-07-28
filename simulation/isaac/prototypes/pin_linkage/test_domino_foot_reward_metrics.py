from __future__ import annotations

import unittest

import numpy as np

from domino_foot_reward_metrics import DominoFootRewardTracker


class DominoFootRewardTrackerTests(unittest.TestCase):
    def make_tracker(self) -> DominoFootRewardTracker:
        return DominoFootRewardTracker(
            1,
            4,
            step_dt_s=0.02,
            min_cycle_air_time_s=0.04,
            target_cycle_air_time_s=0.08,
            min_cycle_clearance_m=0.004,
            min_cycle_body_relative_travel_m=0.020,
        )

    def initial_state(
        self,
        tracker: DominoFootRewardTracker,
    ) -> tuple[np.ndarray, np.ndarray]:
        world_positions = np.zeros((4, 3), dtype=np.float64)
        body_positions = np.zeros((4, 3), dtype=np.float64)
        tracker.reset_env(
            0,
            np.ones(4, dtype=bool),
            np.zeros(4, dtype=np.float64),
            world_positions,
            body_positions,
        )
        return world_positions, body_positions

    def test_penalizes_only_contacting_foot_slip(self) -> None:
        tracker = self.make_tracker()
        world_positions, body_positions = self.initial_state(tracker)
        world_positions[0, 0] = 0.010
        world_positions[1, 0] = 0.020
        metrics = tracker.update_env(
            0,
            np.array([True, False, True, True]),
            np.array([0.0, 0.010, 0.0, 0.0]),
            world_positions,
            body_positions,
            command_moving=True,
        )
        self.assertAlmostEqual(metrics["foot_slip_m_s"], 0.5)

    def test_rewards_qualified_touchdown_cycle(self) -> None:
        tracker = self.make_tracker()
        world_positions, body_positions = self.initial_state(tracker)
        contacts = np.array([False, True, True, True])
        for body_x in (0.010, 0.025, 0.040):
            body_positions = body_positions.copy()
            body_positions[0, 0] = body_x
            tracker.update_env(
                0,
                contacts,
                np.array([0.006, 0.0, 0.0, 0.0]),
                world_positions,
                body_positions,
                command_moving=True,
            )
        touchdown = tracker.update_env(
            0,
            np.ones(4, dtype=bool),
            np.zeros(4, dtype=np.float64),
            world_positions,
            body_positions,
            command_moving=True,
        )
        self.assertGreater(touchdown["valid_cycle_touchdown"], 0.0)

    def test_rejects_vertical_only_or_short_touchdown(self) -> None:
        tracker = self.make_tracker()
        world_positions, body_positions = self.initial_state(tracker)
        contacts = np.array([False, True, True, True])
        for clearance in (0.003, 0.006):
            tracker.update_env(
                0,
                contacts,
                np.array([clearance, 0.0, 0.0, 0.0]),
                world_positions,
                body_positions,
                command_moving=True,
            )
        touchdown = tracker.update_env(
            0,
            np.ones(4, dtype=bool),
            np.zeros(4, dtype=np.float64),
            world_positions,
            body_positions,
            command_moving=True,
        )
        self.assertEqual(touchdown["valid_cycle_touchdown"], 0.0)

    def test_reset_prevents_teleport_slip(self) -> None:
        tracker = self.make_tracker()
        world_positions, body_positions = self.initial_state(tracker)
        world_positions[:, 0] = 100.0
        tracker.reset_env(
            0,
            np.ones(4, dtype=bool),
            np.zeros(4, dtype=np.float64),
            world_positions,
            body_positions,
        )
        metrics = tracker.update_env(
            0,
            np.ones(4, dtype=bool),
            np.zeros(4, dtype=np.float64),
            world_positions,
            body_positions,
            command_moving=True,
        )
        self.assertEqual(metrics["foot_slip_m_s"], 0.0)

    def test_reports_unequal_completed_air_times(self) -> None:
        tracker = self.make_tracker()
        world_positions, body_positions = self.initial_state(tracker)
        for _ in range(2):
            tracker.update_env(
                0,
                np.array([False, True, True, True]),
                np.array([0.006, 0.0, 0.0, 0.0]),
                world_positions,
                body_positions,
                command_moving=True,
            )
        tracker.update_env(
            0,
            np.array([True, False, True, True]),
            np.array([0.0, 0.006, 0.0, 0.0]),
            world_positions,
            body_positions,
            command_moving=True,
        )
        for _ in range(3):
            tracker.update_env(
                0,
                np.array([True, False, True, True]),
                np.array([0.0, 0.006, 0.0, 0.0]),
                world_positions,
                body_positions,
                command_moving=True,
            )
        metrics = tracker.update_env(
            0,
            np.ones(4, dtype=bool),
            np.zeros(4, dtype=np.float64),
            world_positions,
            body_positions,
            command_moving=True,
        )
        self.assertGreater(metrics["air_contact_time_variance_s2"], 0.0)


if __name__ == "__main__":
    unittest.main()
