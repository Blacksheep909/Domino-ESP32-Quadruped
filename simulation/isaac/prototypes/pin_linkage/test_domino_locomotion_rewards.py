"""Contract tests for Domino's command-motion reward terms."""

from __future__ import annotations

import math
import unittest

import numpy as np

from domino_locomotion_rewards import (
    command_motion_terms,
    quadruped_posture_terms,
    quadruped_support_terms,
)


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


class QuadrupedSupportRewardTests(unittest.TestCase):
    def test_diagonal_trot_has_balanced_support(self) -> None:
        terms = quadruped_support_terms(np.array([1.0, 0.0, 0.0, 1.0]))
        self.assertAlmostEqual(terms["front_rear_support_balance"], 1.0)
        self.assertAlmostEqual(terms["axle_support_imbalance"], 0.0)
        self.assertAlmostEqual(terms["same_axle_airborne_pairs"], 0.0)
        self.assertAlmostEqual(terms["excess_airborne_feet"], 0.0)

    def test_all_feet_contact_does_not_earn_trot_support(self) -> None:
        terms = quadruped_support_terms(np.ones(4))
        self.assertAlmostEqual(terms["front_rear_support_balance"], 0.0)
        self.assertAlmostEqual(terms["axle_support_imbalance"], 0.0)
        self.assertAlmostEqual(terms["same_axle_airborne_pairs"], 0.0)
        self.assertAlmostEqual(terms["excess_airborne_feet"], 0.0)

    def test_hind_leg_hop_loses_support_balance(self) -> None:
        terms = quadruped_support_terms(np.array([0.0, 0.0, 1.0, 1.0]))
        self.assertAlmostEqual(terms["front_rear_support_balance"], 0.0)
        self.assertAlmostEqual(terms["axle_support_imbalance"], 1.0)
        self.assertAlmostEqual(terms["same_axle_airborne_pairs"], 1.0)
        self.assertAlmostEqual(terms["excess_airborne_feet"], 0.0)

    def test_full_flight_is_penalized(self) -> None:
        terms = quadruped_support_terms(np.zeros(4))
        self.assertAlmostEqual(terms["front_rear_support_balance"], 0.0)
        self.assertAlmostEqual(terms["axle_support_imbalance"], 0.0)
        self.assertAlmostEqual(terms["same_axle_airborne_pairs"], 2.0)
        self.assertAlmostEqual(terms["excess_airborne_feet"], 2.0)


class QuadrupedPostureRewardTests(unittest.TestCase):
    def test_level_body_with_forward_front_feet_has_no_penalty(self) -> None:
        terms = quadruped_posture_terms(
            np.array([0.0, 0.0, -1.0]),
            np.array(
                [
                    [0.20, -0.10, -0.25],
                    [0.20, 0.10, -0.25],
                    [-0.20, 0.10, -0.25],
                    [-0.20, -0.10, -0.25],
                ]
            ),
            front_foot_min_body_x_m=0.20,
            reach_normalization_m=0.10,
        )
        self.assertAlmostEqual(terms["pitch_orientation_sq"], 0.0)
        self.assertAlmostEqual(terms["front_foot_backward_reach"], 0.0)
        self.assertAlmostEqual(terms["front_pair_backward_reach"], 0.0)

    def test_nose_down_pitch_and_backward_front_feet_are_penalized(self) -> None:
        terms = quadruped_posture_terms(
            np.array([0.5, 0.0, -0.8660254]),
            np.array(
                [
                    [0.10, -0.10, -0.25],
                    [0.10, 0.10, -0.25],
                    [-0.20, 0.10, -0.25],
                    [-0.20, -0.10, -0.25],
                ]
            ),
            front_foot_min_body_x_m=0.20,
            reach_normalization_m=0.10,
        )
        self.assertAlmostEqual(terms["pitch_orientation_sq"], 0.25)
        self.assertAlmostEqual(terms["front_foot_backward_reach"], 1.0)
        self.assertAlmostEqual(terms["front_pair_backward_reach"], 1.0)

    def test_single_backward_front_foot_does_not_trigger_pair_penalty(self) -> None:
        terms = quadruped_posture_terms(
            np.array([0.0, 0.0, -1.0]),
            np.array(
                [
                    [0.10, -0.10, -0.25],
                    [0.20, 0.10, -0.25],
                    [-0.20, 0.10, -0.25],
                    [-0.20, -0.10, -0.25],
                ]
            ),
            front_foot_min_body_x_m=0.20,
            reach_normalization_m=0.10,
        )
        self.assertAlmostEqual(terms["front_foot_backward_reach"], 0.5)
        self.assertAlmostEqual(terms["front_pair_backward_reach"], 0.0)


if __name__ == "__main__":
    unittest.main()
