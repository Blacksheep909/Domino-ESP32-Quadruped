"""Import-safe scripted reference gait helpers for Domino CAD-linkage runs."""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np

CONTRACT_DIR = Path(__file__).resolve().parents[1] / "quadruped"
if str(CONTRACT_DIR) not in sys.path:
    sys.path.insert(0, str(CONTRACT_DIR))

from domino_action_contract import EXPECTED_ACTION_COUNT  # noqa: E402


PHASE_OFFSETS_RAD = [0.0, math.pi, 0.0, math.pi]
SIDE_SIGNS = [-1.0, 1.0, 1.0, -1.0]
LEG_PHASE_PARAMETER_NAMES = [f"leg_phase_{index}" for index in range(4)]
SHOULDER_SIGN_PARAMETER_NAMES = [f"shoulder_sign_{index}" for index in range(4)]
LOWER_SIGN_PARAMETER_NAMES = [f"lower_sign_{index}" for index in range(4)]
UPPER_SIGN_PARAMETER_NAMES = [f"upper_sign_{index}" for index in range(4)]
REFERENCE_GAIT_PARAMETER_NAMES = [
    "lower_amp",
    "upper_amp",
    "shoulder_amp",
    "lower_bias",
    "upper_bias",
    "shoulder_bias",
    "lower_phase",
    "upper_phase",
    "shoulder_phase",
    "frequency_scale",
    *LEG_PHASE_PARAMETER_NAMES,
    *SHOULDER_SIGN_PARAMETER_NAMES,
    *LOWER_SIGN_PARAMETER_NAMES,
    *UPPER_SIGN_PARAMETER_NAMES,
]


def default_reference_candidate() -> dict[str, float | str]:
    return {
        "name": "baseline_lift_trot",
        "lower_amp": 0.35,
        "upper_amp": 0.55,
        "shoulder_amp": 0.22,
        "lower_bias": -0.55,
        "upper_bias": 0.05,
        "shoulder_bias": 0.0,
        "lower_phase": 0.0,
        "upper_phase": math.pi / 2.0,
        "shoulder_phase": math.pi / 2.0,
        "frequency_scale": 1.0,
        "leg_phase_0": PHASE_OFFSETS_RAD[0],
        "leg_phase_1": PHASE_OFFSETS_RAD[1],
        "leg_phase_2": PHASE_OFFSETS_RAD[2],
        "leg_phase_3": PHASE_OFFSETS_RAD[3],
        "shoulder_sign_0": SIDE_SIGNS[0],
        "shoulder_sign_1": SIDE_SIGNS[1],
        "shoulder_sign_2": SIDE_SIGNS[2],
        "shoulder_sign_3": SIDE_SIGNS[3],
        "lower_sign_0": 1.0,
        "lower_sign_1": 1.0,
        "lower_sign_2": 1.0,
        "lower_sign_3": 1.0,
        "upper_sign_0": 1.0,
        "upper_sign_1": 1.0,
        "upper_sign_2": 1.0,
        "upper_sign_3": 1.0,
    }


def candidate_with_defaults(candidate: dict) -> dict[str, float | str]:
    merged = default_reference_candidate()
    merged.update(candidate)
    for key in REFERENCE_GAIT_PARAMETER_NAMES:
        merged[key] = float(merged[key])
    merged["name"] = str(merged.get("name", "reference_gait"))
    return merged


def is_keyframe_sequence(candidate: dict) -> bool:
    return str(candidate.get("type", "")).lower() == "keyframe_sequence"


def validate_keyframe_sequence(candidate: dict) -> dict:
    if not is_keyframe_sequence(candidate):
        raise ValueError("Reference candidate is not a keyframe sequence.")
    segments = candidate.get("segments")
    if not isinstance(segments, list) or not segments:
        raise ValueError("Keyframe sequence requires at least one segment.")
    normalized_segments = []
    for segment_index, segment in enumerate(segments):
        steps = int(segment.get("steps", 0))
        if steps <= 0:
            raise ValueError(f"Keyframe segment {segment_index} must contain a positive step count.")
        start = np.asarray(segment.get("start_actions"), dtype=np.float32).reshape(-1)
        end = np.asarray(segment.get("end_actions"), dtype=np.float32).reshape(-1)
        if start.shape != (EXPECTED_ACTION_COUNT,) or end.shape != (EXPECTED_ACTION_COUNT,):
            raise ValueError(
                f"Keyframe segment {segment_index} must contain {EXPECTED_ACTION_COUNT} start and end actions."
            )
        swing_leg_index = int(segment.get("swing_leg_index", -1))
        if swing_leg_index not in {-1, 0, 1, 2, 3}:
            raise ValueError(f"Invalid swing leg index in keyframe segment {segment_index}: {swing_leg_index}")
        normalized_segments.append(
            {
                "name": str(segment.get("name", f"segment_{segment_index}")),
                "steps": steps,
                "start_actions": [clamp_action(value) for value in start],
                "end_actions": [clamp_action(value) for value in end],
                "swing_leg_index": swing_leg_index,
            }
        )
    normalized = dict(candidate)
    normalized["name"] = str(candidate.get("name", "keyframe_sequence"))
    normalized["type"] = "keyframe_sequence"
    normalized["loop"] = bool(candidate.get("loop", True))
    normalized["segments"] = normalized_segments
    normalized["total_steps"] = sum(int(segment["steps"]) for segment in normalized_segments)
    return normalized


def load_reference_candidate(path: str | Path) -> dict[str, object]:
    data = json.loads(Path(path).expanduser().resolve().read_text(encoding="utf-8-sig"))
    if isinstance(data, dict) and is_keyframe_sequence(data):
        return validate_keyframe_sequence(data)
    if isinstance(data, dict) and "candidate" in data:
        if isinstance(data["candidate"], dict) and is_keyframe_sequence(data["candidate"]):
            return validate_keyframe_sequence(data["candidate"])
        return candidate_with_defaults(data["candidate"])
    if isinstance(data, dict) and isinstance(data.get("best"), dict) and "candidate" in data["best"]:
        return candidate_with_defaults(data["best"]["candidate"])
    if isinstance(data, dict):
        return candidate_with_defaults(data)
    if isinstance(data, list) and data:
        return candidate_with_defaults(data[0])
    raise ValueError(f"Unsupported reference gait JSON format in {path}.")


def reference_actions_for_steps(candidate: dict, step_indexes: np.ndarray) -> np.ndarray:
    if not is_keyframe_sequence(candidate):
        step_indexes = np.asarray(step_indexes, dtype=np.float64).reshape(-1)
        base_phases = 2.0 * math.pi * step_indexes / max(float(len(step_indexes)), 1.0)
        return reference_actions_for_base_phases(candidate, base_phases)
    candidate = validate_keyframe_sequence(candidate)
    total_steps = int(candidate["total_steps"])
    rows = []
    for raw_step in np.asarray(step_indexes, dtype=np.int64).reshape(-1):
        step = int(raw_step)
        if bool(candidate["loop"]):
            step %= total_steps
        else:
            step = max(0, min(step, total_steps - 1))
        offset = step
        for segment in candidate["segments"]:
            segment_steps = int(segment["steps"])
            if offset < segment_steps:
                alpha = float(offset + 1) / float(segment_steps)
                start = np.asarray(segment["start_actions"], dtype=np.float32)
                end = np.asarray(segment["end_actions"], dtype=np.float32)
                rows.append(np.clip(start + alpha * (end - start), -1.0, 1.0))
                break
            offset -= segment_steps
    return np.asarray(rows, dtype=np.float32).reshape(-1, EXPECTED_ACTION_COUNT)


def reference_desired_stance_for_steps(candidate: dict, step_indexes: np.ndarray) -> np.ndarray:
    candidate = validate_keyframe_sequence(candidate)
    total_steps = int(candidate["total_steps"])
    rows = []
    for raw_step in np.asarray(step_indexes, dtype=np.int64).reshape(-1):
        step = int(raw_step)
        if bool(candidate["loop"]):
            step %= total_steps
        else:
            step = max(0, min(step, total_steps - 1))
        offset = step
        stance = np.ones(4, dtype=np.float32)
        for segment in candidate["segments"]:
            segment_steps = int(segment["steps"])
            if offset < segment_steps:
                swing_leg_index = int(segment["swing_leg_index"])
                if swing_leg_index >= 0:
                    stance[swing_leg_index] = 0.0
                break
            offset -= segment_steps
        rows.append(stance)
    return np.asarray(rows, dtype=np.float32).reshape(-1, 4)


def clamp_action(value: float, lower: float = -1.0, upper: float = 1.0) -> float:
    return max(lower, min(upper, float(value)))


def reference_actions_for_base_phases(candidate: dict, base_phases_rad: np.ndarray) -> np.ndarray:
    candidate = candidate_with_defaults(candidate)
    rows = []
    for base_phase in np.asarray(base_phases_rad, dtype=np.float64).reshape(-1):
        row = []
        scaled_base_phase = float(base_phase) * float(candidate["frequency_scale"])
        for leg_index in range(4):
            phase_offset = float(candidate[f"leg_phase_{leg_index}"])
            shoulder_sign = float(candidate[f"shoulder_sign_{leg_index}"])
            lower_sign = float(candidate[f"lower_sign_{leg_index}"])
            upper_sign = float(candidate[f"upper_sign_{leg_index}"])
            phase = scaled_base_phase + phase_offset
            shoulder = float(candidate["shoulder_bias"]) + shoulder_sign * float(
                candidate["shoulder_amp"]
            ) * math.sin(phase + float(candidate["shoulder_phase"]))
            lower = float(candidate["lower_bias"]) + lower_sign * float(candidate["lower_amp"]) * math.sin(
                phase + float(candidate["lower_phase"])
            )
            upper = float(candidate["upper_bias"]) + upper_sign * float(candidate["upper_amp"]) * math.sin(
                phase + float(candidate["upper_phase"])
            )
            row.extend([clamp_action(shoulder), clamp_action(lower), clamp_action(upper)])
        if len(row) != EXPECTED_ACTION_COUNT:
            raise RuntimeError(f"Expected reference gait row to contain {EXPECTED_ACTION_COUNT} actions, found {len(row)}.")
        rows.append(row)
    actions = np.asarray(rows, dtype=np.float32)
    if actions.shape != (len(rows), EXPECTED_ACTION_COUNT):
        raise RuntimeError(f"Expected reference action array with {EXPECTED_ACTION_COUNT} columns, found {actions.shape}.")
    return actions
