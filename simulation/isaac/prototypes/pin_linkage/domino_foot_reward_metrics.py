"""Stateful foot metrics used by Domino's dense locomotion reward."""

from __future__ import annotations

import numpy as np


class DominoFootRewardTracker:
    """Measure contact slip, timing balance, and qualified touchdown cycles."""

    def __init__(
        self,
        num_envs: int,
        num_feet: int,
        *,
        step_dt_s: float,
        min_cycle_air_time_s: float,
        target_cycle_air_time_s: float,
        min_cycle_clearance_m: float,
        min_cycle_body_relative_travel_m: float,
        timing_clip_s: float = 0.5,
    ) -> None:
        self.num_envs = int(num_envs)
        self.num_feet = int(num_feet)
        self.step_dt_s = max(float(step_dt_s), 1.0e-6)
        self.min_cycle_air_time_s = max(float(min_cycle_air_time_s), 0.0)
        self.target_cycle_air_time_s = max(
            float(target_cycle_air_time_s),
            self.min_cycle_air_time_s,
            1.0e-6,
        )
        self.min_cycle_clearance_m = max(float(min_cycle_clearance_m), 0.0)
        self.min_cycle_body_relative_travel_m = max(
            float(min_cycle_body_relative_travel_m),
            0.0,
        )
        self.timing_clip_s = max(float(timing_clip_s), 1.0e-6)

        shape = (self.num_envs, self.num_feet)
        self._initialized = np.zeros(self.num_envs, dtype=bool)
        self._previous_contacts = np.zeros(shape, dtype=bool)
        self._previous_world_positions = np.zeros((*shape, 3), dtype=np.float64)
        self._previous_body_relative_positions = np.zeros((*shape, 3), dtype=np.float64)
        self._current_air_time_s = np.zeros(shape, dtype=np.float64)
        self._current_contact_time_s = np.zeros(shape, dtype=np.float64)
        self._last_air_time_s = np.zeros(shape, dtype=np.float64)
        self._last_contact_time_s = np.zeros(shape, dtype=np.float64)
        self._swing_peak_clearance_m = np.zeros(shape, dtype=np.float64)
        self._swing_body_relative_travel_m = np.zeros(shape, dtype=np.float64)

    def _coerce_inputs(
        self,
        contacts: np.ndarray,
        clearances_m: np.ndarray,
        world_positions: np.ndarray,
        body_relative_positions: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        contacts = np.asarray(contacts, dtype=bool).reshape(-1)
        clearances_m = np.asarray(clearances_m, dtype=np.float64).reshape(-1)
        world_positions = np.asarray(world_positions, dtype=np.float64).reshape(-1, 3)
        body_relative_positions = np.asarray(
            body_relative_positions,
            dtype=np.float64,
        ).reshape(-1, 3)
        expected_vector_shape = (self.num_feet,)
        expected_position_shape = (self.num_feet, 3)
        if contacts.shape != expected_vector_shape:
            raise ValueError(
                f"Expected contacts shaped {expected_vector_shape}, received {contacts.shape}."
            )
        if clearances_m.shape != expected_vector_shape:
            raise ValueError(
                f"Expected clearances shaped {expected_vector_shape}, received {clearances_m.shape}."
            )
        if world_positions.shape != expected_position_shape:
            raise ValueError(
                f"Expected world positions shaped {expected_position_shape}, "
                f"received {world_positions.shape}."
            )
        if body_relative_positions.shape != expected_position_shape:
            raise ValueError(
                f"Expected body-relative positions shaped {expected_position_shape}, "
                f"received {body_relative_positions.shape}."
            )
        return contacts, clearances_m, world_positions, body_relative_positions

    def reset_env(
        self,
        env_index: int,
        contacts: np.ndarray,
        clearances_m: np.ndarray,
        world_positions: np.ndarray,
        body_relative_positions: np.ndarray,
    ) -> None:
        contacts, _, world_positions, body_relative_positions = self._coerce_inputs(
            contacts,
            clearances_m,
            world_positions,
            body_relative_positions,
        )
        self._initialized[env_index] = True
        self._previous_contacts[env_index] = contacts
        self._previous_world_positions[env_index] = world_positions
        self._previous_body_relative_positions[env_index] = body_relative_positions
        self._current_air_time_s[env_index] = 0.0
        self._current_contact_time_s[env_index] = (
            contacts.astype(np.float64) * self.step_dt_s
        )
        self._last_air_time_s[env_index] = 0.0
        self._last_contact_time_s[env_index] = 0.0
        self._swing_peak_clearance_m[env_index] = 0.0
        self._swing_body_relative_travel_m[env_index] = 0.0

    def update_env(
        self,
        env_index: int,
        contacts: np.ndarray,
        clearances_m: np.ndarray,
        world_positions: np.ndarray,
        body_relative_positions: np.ndarray,
        *,
        command_moving: bool,
        suppress_reward: bool = False,
    ) -> dict[str, float]:
        contacts, clearances_m, world_positions, body_relative_positions = self._coerce_inputs(
            contacts,
            clearances_m,
            world_positions,
            body_relative_positions,
        )
        if not self._initialized[env_index]:
            self.reset_env(
                env_index,
                contacts,
                clearances_m,
                world_positions,
                body_relative_positions,
            )
            return {
                "foot_slip_m_s": 0.0,
                "air_contact_time_variance_s2": 0.0,
                "valid_cycle_touchdown": 0.0,
            }

        previous_contacts = self._previous_contacts[env_index].copy()
        world_delta_xy = (
            world_positions[:, :2]
            - self._previous_world_positions[env_index, :, :2]
        )
        foot_planar_speed_m_s = np.linalg.norm(world_delta_xy, axis=1) / self.step_dt_s
        foot_slip_m_s = float(np.sum(foot_planar_speed_m_s * contacts))
        body_relative_step_travel_m = np.linalg.norm(
            body_relative_positions
            - self._previous_body_relative_positions[env_index],
            axis=1,
        )

        valid_cycle_touchdown = 0.0
        for foot_index in range(self.num_feet):
            contact = bool(contacts[foot_index])
            was_in_contact = bool(previous_contacts[foot_index])
            if contact:
                if was_in_contact:
                    self._current_contact_time_s[env_index, foot_index] += self.step_dt_s
                else:
                    air_time_s = float(
                        self._current_air_time_s[env_index, foot_index]
                    )
                    self._last_air_time_s[env_index, foot_index] = air_time_s
                    self._current_air_time_s[env_index, foot_index] = 0.0
                    self._current_contact_time_s[env_index, foot_index] = self.step_dt_s
                    cycle_is_valid = (
                        bool(command_moving)
                        and air_time_s >= self.min_cycle_air_time_s
                        and float(
                            self._swing_peak_clearance_m[env_index, foot_index]
                        )
                        >= self.min_cycle_clearance_m
                        and float(
                            self._swing_body_relative_travel_m[
                                env_index,
                                foot_index,
                            ]
                        )
                        >= self.min_cycle_body_relative_travel_m
                    )
                    if cycle_is_valid:
                        valid_cycle_touchdown += min(
                            air_time_s / self.target_cycle_air_time_s,
                            1.0,
                        )
                    self._swing_peak_clearance_m[env_index, foot_index] = 0.0
                    self._swing_body_relative_travel_m[env_index, foot_index] = 0.0
            else:
                if was_in_contact:
                    self._last_contact_time_s[env_index, foot_index] = float(
                        self._current_contact_time_s[env_index, foot_index]
                    )
                    self._current_contact_time_s[env_index, foot_index] = 0.0
                    self._current_air_time_s[env_index, foot_index] = self.step_dt_s
                    self._swing_peak_clearance_m[env_index, foot_index] = max(
                        float(clearances_m[foot_index]),
                        0.0,
                    )
                    self._swing_body_relative_travel_m[env_index, foot_index] = float(
                        body_relative_step_travel_m[foot_index]
                    )
                else:
                    self._current_air_time_s[env_index, foot_index] += self.step_dt_s
                    self._swing_peak_clearance_m[env_index, foot_index] = max(
                        float(
                            self._swing_peak_clearance_m[env_index, foot_index]
                        ),
                        float(clearances_m[foot_index]),
                    )
                    self._swing_body_relative_travel_m[env_index, foot_index] += float(
                        body_relative_step_travel_m[foot_index]
                    )

        clipped_air_time = np.clip(
            self._last_air_time_s[env_index],
            0.0,
            self.timing_clip_s,
        )
        clipped_contact_time = np.clip(
            self._last_contact_time_s[env_index],
            0.0,
            self.timing_clip_s,
        )
        timing_variance_s2 = float(
            np.var(clipped_air_time) + np.var(clipped_contact_time)
        )

        self._previous_contacts[env_index] = contacts
        self._previous_world_positions[env_index] = world_positions
        self._previous_body_relative_positions[env_index] = body_relative_positions
        if suppress_reward:
            foot_slip_m_s = 0.0
            timing_variance_s2 = 0.0
            valid_cycle_touchdown = 0.0
        return {
            "foot_slip_m_s": foot_slip_m_s,
            "air_contact_time_variance_s2": timing_variance_s2,
            "valid_cycle_touchdown": float(valid_cycle_touchdown),
        }
