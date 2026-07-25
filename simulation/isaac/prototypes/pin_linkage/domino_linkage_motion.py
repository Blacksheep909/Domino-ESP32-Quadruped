"""Measure actual Domino linkage-drive rotation independently of chassis motion."""

from __future__ import annotations

import math

import numpy as np

from domino_action_contract import per_leg_action_layout
from domino_cad_linkage_env import quat_wxyz_to_rotation_matrix, to_numpy


def _body_rotation(env, env_index: int, body_name: str) -> np.ndarray:
    _, orientation = env._body_views_by_env[env_index][body_name].get_world_pose()
    return quat_wxyz_to_rotation_matrix(to_numpy(orientation).astype(np.float64).reshape(-1))


def hip_carriage_relative_actual_cad_visual_feet(env, env_index: int) -> np.ndarray:
    """Return each rendered CAD foot in its own hip-carriage coordinate frame."""
    visual_feet = env._actual_cad_visual_foot_positions(env_index, rendered=True)
    legs = per_leg_action_layout()
    if visual_feet.size == 0:
        return visual_feet.reshape(0, 3).astype(np.float64)
    if visual_feet.shape[0] != len(legs):
        raise RuntimeError(
            f"Expected {len(legs)} rendered Domino feet, found {visual_feet.shape[0]}."
        )

    relative_feet = []
    for foot_position, leg in zip(visual_feet, legs, strict=True):
        carriage_name = f"{leg['leg_id']}_ground"
        carriage_position, carriage_orientation = env._body_views_by_env[env_index][carriage_name].get_world_pose()
        carriage_position = to_numpy(carriage_position).astype(np.float64).reshape(-1)[:3]
        world_from_carriage = quat_wxyz_to_rotation_matrix(
            to_numpy(carriage_orientation).astype(np.float64).reshape(-1)
        )
        relative_feet.append(
            (np.asarray(foot_position, dtype=np.float64) - carriage_position) @ world_from_carriage
        )
    return np.stack(relative_feet).astype(np.float64)


def create_foot_endpoint_motion_tracker(env) -> dict[str, object]:
    names = [str(leg["leg_id"]) for leg in per_leg_action_layout()]
    initial_by_env: list[np.ndarray] = []
    max_displacement_by_env: list[np.ndarray] = []
    range_min_by_env: list[np.ndarray] = []
    range_max_by_env: list[np.ndarray] = []
    for env_index in range(env.num_envs):
        positions = hip_carriage_relative_actual_cad_visual_feet(env, env_index)
        initial_by_env.append(positions.copy())
        max_displacement_by_env.append(np.zeros(len(names), dtype=np.float64))
        range_min_by_env.append(positions.copy())
        range_max_by_env.append(positions.copy())
    return {
        "names": names,
        "initial_by_env": initial_by_env,
        "max_displacement_by_env": max_displacement_by_env,
        "range_min_by_env": range_min_by_env,
        "range_max_by_env": range_max_by_env,
    }


def update_foot_endpoint_motion_tracker(tracker: dict[str, object], env) -> None:
    for env_index in range(env.num_envs):
        current = hip_carriage_relative_actual_cad_visual_feet(env, env_index)
        initial = tracker["initial_by_env"][env_index]
        tracker["max_displacement_by_env"][env_index] = np.maximum(
            tracker["max_displacement_by_env"][env_index],
            np.linalg.norm(current - initial, axis=1),
        )
        tracker["range_min_by_env"][env_index] = np.minimum(
            tracker["range_min_by_env"][env_index], current
        )
        tracker["range_max_by_env"][env_index] = np.maximum(
            tracker["range_max_by_env"][env_index], current
        )


def foot_endpoint_motion_report(tracker: dict[str, object]) -> dict[str, object]:
    names = list(tracker["names"])
    all_values: list[float] = []
    env_rows = []
    for env_index, values in enumerate(tracker["max_displacement_by_env"]):
        values = np.asarray(values, dtype=np.float64)
        all_values.extend(values.tolist())
        ranges = np.asarray(tracker["range_max_by_env"][env_index]) - np.asarray(
            tracker["range_min_by_env"][env_index]
        )
        env_rows.append(
            {
                "env_index": int(env_index),
                "min_each_foot_motion_m": round(float(np.min(values)), 6) if values.size else 0.0,
                "feet": [
                    {
                        "name": name,
                        "max_displacement_m": round(float(values[index]), 6),
                        "range_xyz_m": np.round(ranges[index], 6).tolist(),
                    }
                    for index, name in enumerate(names)
                ],
            }
        )
    return {
        "source": "rendered_actual_cad_foot_in_own_hip_carriage_frame",
        "foot_count": len(names),
        "min_each_foot_motion_m": round(float(min(all_values)), 6) if all_values else 0.0,
        "max_foot_motion_m": round(float(max(all_values)), 6) if all_values else 0.0,
        "envs": env_rows,
    }


def linkage_drive_relative_rotations(env, env_index: int) -> tuple[list[str], np.ndarray]:
    """Return lower/upper driver rotations relative to each leg's hip carriage."""
    names: list[str] = []
    rotations: list[np.ndarray] = []
    for leg in per_leg_action_layout():
        leg_id = str(leg["leg_id"])
        carriage_rotation = _body_rotation(env, env_index, f"{leg_id}_ground")
        for role, body_suffix, action_name in (
            ("lower_linkage_drive", "lower_driver", str(leg["linkage_drives"][0])),
            ("upper_pitch_drive", "upper_driver", str(leg["linkage_drives"][1])),
        ):
            driver_rotation = _body_rotation(env, env_index, f"{leg_id}_{body_suffix}")
            names.append(action_name)
            rotations.append(carriage_rotation.T @ driver_rotation)
    return names, np.stack(rotations).astype(np.float64)


def rotation_delta_deg(initial: np.ndarray, current: np.ndarray) -> float:
    delta = np.asarray(initial, dtype=np.float64).T @ np.asarray(current, dtype=np.float64)
    cosine = max(-1.0, min(1.0, 0.5 * (float(np.trace(delta)) - 1.0)))
    return math.degrees(math.acos(cosine))


def create_linkage_motion_tracker(env) -> dict[str, object]:
    names: list[str] | None = None
    initial_by_env: list[np.ndarray] = []
    max_deg_by_env: list[np.ndarray] = []
    for env_index in range(env.num_envs):
        env_names, rotations = linkage_drive_relative_rotations(env, env_index)
        if names is None:
            names = env_names
        elif env_names != names:
            raise RuntimeError("Domino linkage-drive ordering differs between environments.")
        initial_by_env.append(rotations)
        max_deg_by_env.append(np.zeros(len(env_names), dtype=np.float64))
    return {
        "names": names or [],
        "initial_by_env": initial_by_env,
        "max_deg_by_env": max_deg_by_env,
    }


def update_linkage_motion_tracker(tracker: dict[str, object], env) -> None:
    initial_by_env = tracker["initial_by_env"]
    max_deg_by_env = tracker["max_deg_by_env"]
    for env_index in range(env.num_envs):
        _, current = linkage_drive_relative_rotations(env, env_index)
        initial = initial_by_env[env_index]
        maxima = max_deg_by_env[env_index]
        for drive_index in range(current.shape[0]):
            maxima[drive_index] = max(
                float(maxima[drive_index]),
                rotation_delta_deg(initial[drive_index], current[drive_index]),
            )


def linkage_motion_report(tracker: dict[str, object]) -> dict[str, object]:
    names = list(tracker["names"])
    max_deg_by_env = tracker["max_deg_by_env"]
    env_rows = []
    all_values = []
    for env_index, values in enumerate(max_deg_by_env):
        values = np.asarray(values, dtype=np.float64)
        all_values.extend(values.tolist())
        env_rows.append(
            {
                "env_index": int(env_index),
                "min_each_drive_motion_deg": round(float(np.min(values)), 6) if values.size else 0.0,
                "drives": [
                    {
                        "name": name,
                        "max_relative_rotation_deg": round(float(values[index]), 6),
                    }
                    for index, name in enumerate(names)
                ],
            }
        )
    return {
        "source": "driver_rotation_relative_to_hip_carriage",
        "drive_count": len(names),
        "min_each_drive_motion_deg": round(float(min(all_values)), 6) if all_values else 0.0,
        "max_drive_motion_deg": round(float(max(all_values)), 6) if all_values else 0.0,
        "envs": env_rows,
    }
