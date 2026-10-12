"""Pure locomotion reward terms shared by Domino's Isaac environment and tests."""

from __future__ import annotations

import math

import numpy as np


def command_motion_terms(
    velocity_xy_m_s: np.ndarray,
    command_xy_m_s: np.ndarray,
    tracking_sigma_m_s: float,
    stagnation_speed_m_s: float,
) -> dict[str, float]:
    """Return velocity tracking, directional progress, and stagnation terms."""
    velocity = np.asarray(velocity_xy_m_s, dtype=np.float64).reshape(-1)[:2]
    command = np.asarray(command_xy_m_s, dtype=np.float64).reshape(-1)[:2]
    velocity_error_sq = float(np.sum(np.square(velocity - command)))
    sigma = max(float(tracking_sigma_m_s), 1.0e-6)
    tracking = float(math.exp(-velocity_error_sq / (sigma * sigma)))

    command_speed = float(np.linalg.norm(command))
    if command_speed <= 1.0e-6:
        return {
            "velocity_error_sq": velocity_error_sq,
            "velocity_tracking": tracking,
            "directional_progress_m_s": 0.0,
            "stagnation": 0.0,
        }

    command_direction = command / command_speed
    directional_progress = float(np.dot(velocity, command_direction))
    stagnation_threshold = max(float(stagnation_speed_m_s), 1.0e-6)
    stagnation = float(np.clip((stagnation_threshold - directional_progress) / stagnation_threshold, 0.0, 1.0))
    return {
        "velocity_error_sq": velocity_error_sq,
        "velocity_tracking": tracking,
        "directional_progress_m_s": directional_progress,
        "stagnation": stagnation,
    }


def quadruped_support_terms(foot_contacts: np.ndarray) -> dict[str, float]:
    """Reward diagonal support and reject bounding or axle-only support."""
    contacts = np.asarray(foot_contacts, dtype=np.float64).reshape(-1)
    if contacts.shape != (4,):
        raise ValueError(f"Expected four foot contacts, received {contacts.shape}.")

    front_support = float(np.mean(contacts[:2]))
    rear_support = float(np.mean(contacts[2:]))
    contact_count = float(np.sum(contacts))
    airborne = 1.0 - contacts
    airborne_count = float(np.sum(airborne))
    two_foot_factor = max(1.0 - abs(contact_count - 2.0) / 2.0, 0.0)
    return {
        "front_rear_support_balance": 2.0 * min(front_support, rear_support) * two_foot_factor,
        "axle_support_imbalance": abs(front_support - rear_support),
        "same_axle_airborne_pairs": float(
            airborne[0] * airborne[1] + airborne[2] * airborne[3]
        ),
        "excess_airborne_feet": max(airborne_count - 2.0, 0.0),
    }


def quadruped_posture_terms(
    projected_gravity_body: np.ndarray,
    body_relative_foot_positions_m: np.ndarray,
    *,
    front_foot_min_body_x_m: float,
    reach_normalization_m: float,
) -> dict[str, float]:
    """Return pitch and front-foot overreach penalties in the body frame."""
    gravity = np.asarray(projected_gravity_body, dtype=np.float64).reshape(-1)
    feet = np.asarray(body_relative_foot_positions_m, dtype=np.float64).reshape(-1, 3)
    if gravity.shape != (3,):
        raise ValueError(f"Expected projected gravity shaped (3,), received {gravity.shape}.")
    if feet.shape != (4, 3):
        raise ValueError(f"Expected four body-relative foot positions, received {feet.shape}.")

    normalization = max(float(reach_normalization_m), 1.0e-6)
    front_reach_deficit = np.maximum(
        float(front_foot_min_body_x_m) - feet[:2, 0],
        0.0,
    )
    front_pair_deficit = float(np.min(front_reach_deficit))
    return {
        "pitch_orientation_sq": float(gravity[0] * gravity[0]),
        "front_foot_backward_reach": float(
            np.mean(np.square(front_reach_deficit / normalization))
        ),
        # This term is zero unless both front feet are behind the allowed
        # boundary, which targets the synchronized faceplant exploit.
        "front_pair_backward_reach": float(
            np.square(front_pair_deficit / normalization)
        ),
    }
