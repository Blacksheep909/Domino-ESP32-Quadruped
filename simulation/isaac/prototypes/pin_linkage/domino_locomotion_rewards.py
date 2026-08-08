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
    """Reward front/rear support balance and penalize flight beyond a trot."""
    contacts = np.asarray(foot_contacts, dtype=np.float64).reshape(-1)
    if contacts.shape != (4,):
        raise ValueError(f"Expected four foot contacts, received {contacts.shape}.")

    front_support = float(np.mean(contacts[:2]))
    rear_support = float(np.mean(contacts[2:]))
    contact_count = float(np.sum(contacts))
    airborne_count = float(np.sum(1.0 - contacts))
    two_foot_factor = max(1.0 - abs(contact_count - 2.0) / 2.0, 0.0)
    return {
        "front_rear_support_balance": 2.0 * min(front_support, rear_support) * two_foot_factor,
        "excess_airborne_feet": max(airborne_count - 2.0, 0.0),
    }
