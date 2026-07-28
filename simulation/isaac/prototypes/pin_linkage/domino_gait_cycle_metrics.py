"""Contact-event gait metrics for rejecting non-walking policy exploits."""

from __future__ import annotations

from collections import Counter

import numpy as np


class GaitCycleTracker:
    """Track confirmed liftoff, swing, and touchdown cycles for every foot."""

    def __init__(
        self,
        num_envs: int,
        foot_names: list[str],
        *,
        min_air_steps: int,
        touchdown_confirm_steps: int,
        min_clearance_m: float,
        min_body_relative_travel_m: float,
        max_tilt_deg: float,
        min_touchdown_support_feet: int,
    ) -> None:
        self.num_envs = int(num_envs)
        self.foot_names = list(foot_names)
        self.num_feet = len(self.foot_names)
        self.min_air_steps = max(int(min_air_steps), 1)
        self.touchdown_confirm_steps = max(int(touchdown_confirm_steps), 1)
        self.min_clearance_m = max(float(min_clearance_m), 0.0)
        self.min_body_relative_travel_m = max(float(min_body_relative_travel_m), 0.0)
        self.max_tilt_deg = max(float(max_tilt_deg), 0.0)
        self.min_touchdown_support_feet = max(int(min_touchdown_support_feet), 1)

        shape = (self.num_envs, self.num_feet)
        self._initialized = np.zeros(self.num_envs, dtype=bool)
        self._in_swing = np.zeros(shape, dtype=bool)
        self._air_streak = np.zeros(shape, dtype=np.int32)
        self._contact_streak = np.zeros(shape, dtype=np.int32)
        self._liftoff_step = np.full(shape, -1, dtype=np.int32)
        self._liftoff_positions = np.zeros((*shape, 3), dtype=np.float64)
        self._last_positions = np.zeros((*shape, 3), dtype=np.float64)
        self._candidate_positions = np.zeros((*shape, 3), dtype=np.float64)
        self._candidate_steps = np.full(shape, -1, dtype=np.int32)
        self._peak_clearance_m = np.zeros(shape, dtype=np.float64)
        self._body_relative_travel_m = np.zeros(shape, dtype=np.float64)
        self._max_tilt_deg = np.zeros(shape, dtype=np.float64)

        self.completed_cycles = np.zeros(shape, dtype=np.int32)
        self.valid_cycles = np.zeros(shape, dtype=np.int32)
        self.invalid_cycles = np.zeros(shape, dtype=np.int32)
        self.aborted_swings = np.zeros(shape, dtype=np.int32)
        self.airborne_steps = np.zeros(self.num_envs, dtype=np.int32)
        self.observed_steps = np.zeros(self.num_envs, dtype=np.int32)
        self._invalid_reasons: Counter[str] = Counter()
        self._cycle_records: list[dict[str, object]] = []

    def _check_shapes(
        self,
        contacts: np.ndarray,
        clearances_m: np.ndarray,
        body_relative_positions: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        contacts = np.asarray(contacts, dtype=bool).reshape(-1)
        clearances_m = np.asarray(clearances_m, dtype=np.float64).reshape(-1)
        body_relative_positions = np.asarray(body_relative_positions, dtype=np.float64).reshape(-1, 3)
        if contacts.shape != (self.num_feet,):
            raise ValueError(f"Expected {self.num_feet} foot contacts, received {contacts.shape}.")
        if clearances_m.shape != (self.num_feet,):
            raise ValueError(f"Expected {self.num_feet} foot clearances, received {clearances_m.shape}.")
        if body_relative_positions.shape != (self.num_feet, 3):
            raise ValueError(
                f"Expected body-relative foot positions shaped ({self.num_feet}, 3), "
                f"received {body_relative_positions.shape}."
            )
        return contacts, clearances_m, body_relative_positions

    def initialize_env(
        self,
        env_index: int,
        contacts: np.ndarray,
        clearances_m: np.ndarray,
        body_relative_positions: np.ndarray,
    ) -> None:
        contacts, _, body_relative_positions = self._check_shapes(
            contacts,
            clearances_m,
            body_relative_positions,
        )
        self._initialized[env_index] = True
        self._in_swing[env_index] = False
        self._air_streak[env_index] = 0
        self._contact_streak[env_index] = contacts.astype(np.int32)
        self._last_positions[env_index] = body_relative_positions

    def _reset_foot_state(self, env_index: int, foot_index: int) -> None:
        self._in_swing[env_index, foot_index] = False
        self._air_streak[env_index, foot_index] = 0
        self._contact_streak[env_index, foot_index] = 0
        self._liftoff_step[env_index, foot_index] = -1
        self._candidate_steps[env_index, foot_index] = -1
        self._peak_clearance_m[env_index, foot_index] = 0.0
        self._body_relative_travel_m[env_index, foot_index] = 0.0
        self._max_tilt_deg[env_index, foot_index] = 0.0

    def update_env(
        self,
        env_index: int,
        step: int,
        contacts: np.ndarray,
        clearances_m: np.ndarray,
        body_relative_positions: np.ndarray,
        *,
        body_tilt_deg: float,
        done: bool,
    ) -> None:
        contacts, clearances_m, body_relative_positions = self._check_shapes(
            contacts,
            clearances_m,
            body_relative_positions,
        )
        if not self._initialized[env_index]:
            self.initialize_env(env_index, contacts, clearances_m, body_relative_positions)
            return

        self.observed_steps[env_index] += 1
        if not np.any(contacts):
            self.airborne_steps[env_index] += 1

        if done:
            self.aborted_swings[env_index] += self._in_swing[env_index].astype(np.int32)
            for foot_index in range(self.num_feet):
                self._reset_foot_state(env_index, foot_index)
            self._contact_streak[env_index] = contacts.astype(np.int32)
            self._last_positions[env_index] = body_relative_positions
            return

        touchdown_support_feet = int(np.count_nonzero(contacts))
        for foot_index in range(self.num_feet):
            position = body_relative_positions[foot_index]
            travel_increment = float(
                np.linalg.norm(position - self._last_positions[env_index, foot_index])
            )
            contact = bool(contacts[foot_index])

            if not contact:
                self._contact_streak[env_index, foot_index] = 0
                if not self._in_swing[env_index, foot_index]:
                    if self._air_streak[env_index, foot_index] == 0:
                        self._candidate_steps[env_index, foot_index] = int(step)
                        self._candidate_positions[env_index, foot_index] = position
                        self._peak_clearance_m[env_index, foot_index] = float(clearances_m[foot_index])
                        self._body_relative_travel_m[env_index, foot_index] = 0.0
                        self._max_tilt_deg[env_index, foot_index] = float(body_tilt_deg)
                    else:
                        self._peak_clearance_m[env_index, foot_index] = max(
                            self._peak_clearance_m[env_index, foot_index],
                            float(clearances_m[foot_index]),
                        )
                        self._body_relative_travel_m[env_index, foot_index] += travel_increment
                        self._max_tilt_deg[env_index, foot_index] = max(
                            self._max_tilt_deg[env_index, foot_index],
                            float(body_tilt_deg),
                        )
                    self._air_streak[env_index, foot_index] += 1
                    if self._air_streak[env_index, foot_index] >= self.min_air_steps:
                        self._in_swing[env_index, foot_index] = True
                        self._liftoff_step[env_index, foot_index] = self._candidate_steps[
                            env_index, foot_index
                        ]
                        self._liftoff_positions[env_index, foot_index] = self._candidate_positions[
                            env_index, foot_index
                        ]
                else:
                    self._air_streak[env_index, foot_index] += 1
                    self._peak_clearance_m[env_index, foot_index] = max(
                        self._peak_clearance_m[env_index, foot_index],
                        float(clearances_m[foot_index]),
                    )
                    self._body_relative_travel_m[env_index, foot_index] += travel_increment
                    self._max_tilt_deg[env_index, foot_index] = max(
                        self._max_tilt_deg[env_index, foot_index],
                        float(body_tilt_deg),
                    )
            else:
                self._contact_streak[env_index, foot_index] += 1
                if self._in_swing[env_index, foot_index]:
                    self._body_relative_travel_m[env_index, foot_index] += travel_increment
                    self._max_tilt_deg[env_index, foot_index] = max(
                        self._max_tilt_deg[env_index, foot_index],
                        float(body_tilt_deg),
                    )
                    if (
                        self._contact_streak[env_index, foot_index]
                        >= self.touchdown_confirm_steps
                    ):
                        reasons = []
                        if self._peak_clearance_m[env_index, foot_index] < self.min_clearance_m:
                            reasons.append("low_clearance")
                        if (
                            self._body_relative_travel_m[env_index, foot_index]
                            < self.min_body_relative_travel_m
                        ):
                            reasons.append("low_body_relative_travel")
                        if self._max_tilt_deg[env_index, foot_index] > self.max_tilt_deg:
                            reasons.append("excessive_body_tilt")
                        if touchdown_support_feet < self.min_touchdown_support_feet:
                            reasons.append("insufficient_touchdown_support")

                        self.completed_cycles[env_index, foot_index] += 1
                        if reasons:
                            self.invalid_cycles[env_index, foot_index] += 1
                            self._invalid_reasons.update(reasons)
                        else:
                            self.valid_cycles[env_index, foot_index] += 1

                        placement_delta = (
                            position - self._liftoff_positions[env_index, foot_index]
                        )
                        self._cycle_records.append(
                            {
                                "env_index": int(env_index),
                                "foot_index": int(foot_index),
                                "foot_name": self.foot_names[foot_index],
                                "valid": not reasons,
                                "invalid_reasons": reasons,
                                "air_steps": int(
                                    step - self._liftoff_step[env_index, foot_index] + 1
                                ),
                                "peak_clearance_m": float(
                                    self._peak_clearance_m[env_index, foot_index]
                                ),
                                "body_relative_travel_m": float(
                                    self._body_relative_travel_m[env_index, foot_index]
                                ),
                                "body_relative_placement_delta_m": [
                                    float(value) for value in placement_delta.tolist()
                                ],
                                "max_body_tilt_deg": float(
                                    self._max_tilt_deg[env_index, foot_index]
                                ),
                                "touchdown_support_feet": touchdown_support_feet,
                            }
                        )
                        self._reset_foot_state(env_index, foot_index)
                        self._contact_streak[env_index, foot_index] = 1
                else:
                    self._air_streak[env_index, foot_index] = 0

            self._last_positions[env_index, foot_index] = position

    def summary(self) -> dict[str, object]:
        valid_records = [record for record in self._cycle_records if bool(record["valid"])]
        total_completed = int(np.sum(self.completed_cycles))
        total_valid = int(np.sum(self.valid_cycles))
        aggregate_valid_per_foot = np.sum(self.valid_cycles, axis=0)
        max_foot_cycle_share = (
            float(np.max(aggregate_valid_per_foot)) / float(total_valid)
            if total_valid > 0
            else 0.0
        )

        def mean_record_value(key: str) -> float:
            if not valid_records:
                return 0.0
            return float(np.mean([float(record[key]) for record in valid_records]))

        mean_placement_delta = [0.0, 0.0, 0.0]
        if valid_records:
            mean_placement_delta = np.mean(
                np.asarray(
                    [
                        record["body_relative_placement_delta_m"]
                        for record in valid_records
                    ],
                    dtype=np.float64,
                ),
                axis=0,
            ).tolist()

        return {
            "foot_names": self.foot_names,
            "completed_cycles_per_env_foot": self.completed_cycles.tolist(),
            "valid_cycles_per_env_foot": self.valid_cycles.tolist(),
            "invalid_cycles_per_env_foot": self.invalid_cycles.tolist(),
            "aborted_swings_per_env_foot": self.aborted_swings.tolist(),
            "aggregate_valid_cycles_per_foot": aggregate_valid_per_foot.tolist(),
            "min_valid_cycles_per_env_foot": int(np.min(self.valid_cycles)),
            "total_completed_cycles": total_completed,
            "total_valid_cycles": total_valid,
            "valid_cycle_ratio": (
                float(total_valid) / float(total_completed)
                if total_completed > 0
                else 0.0
            ),
            "max_foot_valid_cycle_share": max_foot_cycle_share,
            "invalid_reason_counts": dict(sorted(self._invalid_reasons.items())),
            "airborne_fraction": (
                float(np.sum(self.airborne_steps)) / float(np.sum(self.observed_steps))
                if int(np.sum(self.observed_steps)) > 0
                else 0.0
            ),
            "mean_valid_air_steps": mean_record_value("air_steps"),
            "mean_valid_peak_clearance_m": mean_record_value("peak_clearance_m"),
            "mean_valid_body_relative_travel_m": mean_record_value(
                "body_relative_travel_m"
            ),
            "mean_valid_body_relative_placement_delta_m": [
                float(value) for value in mean_placement_delta
            ],
            "detection_thresholds": {
                "min_air_steps": self.min_air_steps,
                "touchdown_confirm_steps": self.touchdown_confirm_steps,
                "min_clearance_m": self.min_clearance_m,
                "min_body_relative_travel_m": self.min_body_relative_travel_m,
                "max_tilt_deg": self.max_tilt_deg,
                "min_touchdown_support_feet": self.min_touchdown_support_feet,
            },
            "cycles": self._cycle_records,
        }
