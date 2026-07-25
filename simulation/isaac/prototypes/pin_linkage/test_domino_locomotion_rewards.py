"""Contract tests for Domino's command-motion reward terms."""

from __future__ import annotations

import math
import unittest

import numpy as np

from domino_locomotion_rewards import command_motion_terms


class CommandMotionRewardTests(unittest.TestCase):
    def test_stationary_policy_is_explicitly_stagnant(self) -> None:
        terms = command_motion_terms(
            np.array([0.0, 0.0]),
            np.array([0.12, 0.0]),
            tracking_sigma_m_s=0.06,
            stagnation_speed_m_s=0.03,
        )
        self.assertAlmostEqual(terms["directional_progress_m_s"], 0.0)
        self.assertAlmostEqual(terms["stagnation"], 1.0)
        self.assertAlmostEqual(terms["velocity_tracking"], math.exp(-4.0))

    def test_command_tracking_removes_stagnation(self) -> None:
        terms = command_motion_terms(
            np.array([0.12, 0.0]),
            np.array([0.12, 0.0]),
            tracking_sigma_m_s=0.06,
            stagnation_speed_m_s=0.03,
        )
        self.assertAlmostEqual(terms["directional_progress_m_s"], 0.12)
        self.assertAlmostEqual(terms["stagnation"], 0.0)
        self.assertAlmostEqual(terms["velocity_tracking"], 1.0)

    def test_backward_motion_is_not_rewarded_as_progress(self) -> None:
        terms = command_motion_terms(
            np.array([-0.05, 0.0]),
            np.array([0.12, 0.0]),
            tracking_sigma_m_s=0.06,
            stagnation_speed_m_s=0.03,
        )
        self.assertLess(terms["directional_progress_m_s"], 0.0)
        self.assertAlmostEqual(terms["stagnation"], 1.0)

    def test_zero_command_disables_stagnation(self) -> None:
        terms = command_motion_terms(
            np.array([0.0, 0.0]),
            np.array([0.0, 0.0]),
            tracking_sigma_m_s=0.06,
            stagnation_speed_m_s=0.03,
        )
        self.assertAlmostEqual(terms["stagnation"], 0.0)
        self.assertAlmostEqual(terms["directional_progress_m_s"], 0.0)

    def test_velocity_v2_weights_reject_the_stationary_optimum(self) -> None:
        stationary = command_motion_terms(
            np.array([0.0, 0.0]),
            np.array([0.12, 0.0]),
            tracking_sigma_m_s=0.06,
            stagnation_speed_m_s=0.03,
        )
        tracking = command_motion_terms(
            np.array([0.12, 0.0]),
            np.array([0.12, 0.0]),
            tracking_sigma_m_s=0.06,
            stagnation_speed_m_s=0.03,
        )

        def weighted_score(terms: dict[str, float]) -> float:
            return (
                10.0 * terms["velocity_tracking"]
                + 20.0 * terms["directional_progress_m_s"]
                - 3.0 * terms["stagnation"]
            )

        self.assertLess(weighted_score(stationary), 0.0)
        self.assertGreater(weighted_score(tracking), 12.0)
        self.assertGreater(weighted_score(tracking) - weighted_score(stationary), 15.0)


if __name__ == "__main__":
    unittest.main()
