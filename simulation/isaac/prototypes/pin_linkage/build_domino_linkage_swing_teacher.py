"""Build the verified four-leg Domino linkage-swing teacher."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_OUTPUT = REPO_ROOT / "simulation" / "isaac" / "config" / "domino_linkage_swing_cycle_teacher.json"
DEFAULT_LEG_ORDER = [2, 3, 0, 1]
LEG_LABELS = ["front_right", "front_left", "rear_left", "rear_right"]
ACTION_COUNT = 12


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--leg-order", default=",".join(str(value) for value in DEFAULT_LEG_ORDER))
    return parser.parse_args()


def action_row(value: object, label: str) -> list[float]:
    row = [float(item) for item in value] if isinstance(value, list) else []
    if len(row) != ACTION_COUNT:
        raise ValueError(f"{label} must contain exactly {ACTION_COUNT} actions.")
    if any(item < -1.0 or item > 1.0 for item in row):
        raise ValueError(f"{label} contains an action outside [-1, 1].")
    return row


def add_segment(
    segments: list[dict[str, object]],
    name: str,
    steps: int,
    start: list[float],
    end: list[float],
    swing_leg_index: int,
) -> None:
    segments.append(
        {
            "name": name,
            "steps": int(steps),
            "start_actions": start,
            "end_actions": end,
            "swing_leg_index": int(swing_leg_index),
        }
    )


def main() -> None:
    args = parse_args()
    leg_order = [int(value.strip()) for value in str(args.leg_order).split(",") if value.strip()]
    if sorted(leg_order) != [0, 1, 2, 3]:
        raise ValueError("Leg order must contain each index from 0 to 3 exactly once.")
    commands: dict[int, tuple[Path, dict[str, object]]] = {}
    scales = set()
    servo_rates = set()
    for leg_index, label in enumerate(LEG_LABELS):
        path = REPO_ROOT / "simulation" / "isaac" / "config" / f"domino_{label}_swing_hipframe.json"
        command = json.loads(path.read_text(encoding="utf-8-sig"))
        if int(command.get("target_leg_index", -1)) != leg_index:
            raise ValueError(f"Target leg mismatch in {path.name}.")
        metrics = command.get("verified_metrics", {})
        required_gates = (
            "passed",
            "stable_mechanics",
            "drive_gate_passed",
            "airborne_gate_passed",
            "endpoint_motion_gate_passed",
            "support_gate_passed",
        )
        if any(not bool(metrics.get(gate)) for gate in required_gates):
            raise RuntimeError(f"Swing command is not fully verified: {path.name}")
        if float(metrics.get("max_hip_carriage_relative_fore_aft_motion_m", 0.0)) < 0.04:
            raise RuntimeError(f"Swing command lacks verified hip-frame fore/aft motion: {path.name}")
        if float(metrics.get("max_hip_carriage_relative_total_motion_m", 0.0)) < 0.05:
            raise RuntimeError(f"Swing command lacks verified hip-frame endpoint motion: {path.name}")
        for field in ("transfer_actions", "lift_actions", "sweep_actions"):
            action_row(command.get(field), f"{path.name} {field}")
        scales.add(float(command["action_scale_deg"]))
        servo_rates.add(float(command["servo_target_rate_limit_deg_s"]))
        commands[leg_index] = (path, command)
    if len(scales) != 1 or len(servo_rates) != 1:
        raise ValueError("All verified swing commands must use one action scale and servo slew rate.")

    zero = [0.0] * ACTION_COUNT
    segments: list[dict[str, object]] = []
    source_commands: dict[str, str] = {}
    verified_metrics: dict[str, object] = {}
    for leg_index in leg_order:
        path, command = commands[leg_index]
        label = LEG_LABELS[leg_index]
        transfer = action_row(command["transfer_actions"], f"{path.name} transfer_actions")
        lift = action_row(command["lift_actions"], f"{path.name} lift_actions")
        sweep = action_row(command["sweep_actions"], f"{path.name} sweep_actions")
        add_segment(segments, f"{label}_transfer", 60, zero, transfer, -1)
        add_segment(segments, f"{label}_lift", 60, transfer, lift, leg_index)
        add_segment(segments, f"{label}_lift_hold", 20, lift, lift, leg_index)
        add_segment(segments, f"{label}_sweep", 70, lift, sweep, leg_index)
        add_segment(segments, f"{label}_sweep_hold", 20, sweep, sweep, leg_index)
        add_segment(segments, f"{label}_sweep_return", 70, sweep, lift, leg_index)
        add_segment(segments, f"{label}_place", 50, lift, transfer, leg_index)
        add_segment(segments, f"{label}_return", 80, transfer, zero, -1)
        add_segment(segments, f"{label}_neutral", 160, zero, zero, -1)
        source_commands[str(leg_index)] = path.relative_to(REPO_ROOT).as_posix()
        verified_metrics[str(leg_index)] = command["verified_metrics"]

    teacher = {
        "name": "domino_verified_linkage_swing_cycle",
        "type": "keyframe_sequence",
        "loop": True,
        "action_scale_deg": scales.pop(),
        "servo_target_rate_limit_deg_s": servo_rates.pop(),
        "leg_order": leg_order,
        "total_steps": sum(int(segment["steps"]) for segment in segments),
        "source_commands": source_commands,
        "verified_leg_metrics": verified_metrics,
        "segments": segments,
    }
    output_path = args.output.expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(teacher, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"status": "passed", "output": str(output_path), "total_steps": teacher["total_steps"]}, indent=2))


if __name__ == "__main__":
    main()
