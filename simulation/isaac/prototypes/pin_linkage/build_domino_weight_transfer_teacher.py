"""Build a policy teacher from the verified continuous Domino foot-lift cycle."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_REPORT = (
    REPO_ROOT
    / "simulation"
    / "isaac"
    / "out"
    / "cad_identity"
    / "next_policy"
    / "weight_transfer_context_verified_four_foot_cycle_v2.json"
)
DEFAULT_OUTPUT = REPO_ROOT / "simulation" / "isaac" / "config" / "domino_weight_transfer_cycle_teacher.json"
COMMAND_PATHS = {
    0: REPO_ROOT / "simulation" / "isaac" / "config" / "domino_front_right_weight_transfer.json",
    1: REPO_ROOT / "simulation" / "isaac" / "config" / "domino_front_left_weight_transfer.json",
    2: REPO_ROOT / "simulation" / "isaac" / "config" / "domino_rear_left_weight_transfer.json",
    3: REPO_ROOT / "simulation" / "isaac" / "config" / "domino_rear_right_weight_transfer.json",
}
LEG_LABELS = {
    0: "front_right",
    1: "front_left",
    2: "rear_left",
    3: "rear_right",
}
ACTION_COUNT = 12


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cycle-report", type=Path, default=DEFAULT_REPORT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()


def action_row(value: object, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != ACTION_COUNT:
        raise ValueError(f"{label} must contain exactly {ACTION_COUNT} actions.")
    row = [float(item) for item in value]
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
    if steps <= 0:
        raise ValueError(f"Segment {name} must contain at least one step.")
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
    report_path = args.cycle_report.expanduser().resolve()
    output_path = args.output.expanduser().resolve()
    report = json.loads(report_path.read_text(encoding="utf-8-sig"))
    if report.get("status") != "passed" or not bool(report.get("global_mechanics_passed")):
        raise RuntimeError("The source continuous-cycle mechanics report has not passed.")

    leg_order = [int(value) for value in report.get("leg_order", [])]
    if sorted(leg_order) != [0, 1, 2, 3]:
        raise ValueError("The source report must contain each leg exactly once.")

    zero = [0.0] * ACTION_COUNT
    segments: list[dict[str, object]] = []
    source_commands: dict[str, str] = {}
    action_scale = float(report["action_scale_deg"])
    servo_rate = float(report["servo_target_rate_limit_deg_s"])

    for leg_index in leg_order:
        path = COMMAND_PATHS[leg_index]
        command = json.loads(path.read_text(encoding="utf-8-sig"))
        if int(command.get("target_leg_index", -1)) != leg_index:
            raise ValueError(f"Command target mismatch in {path.name}.")
        if not bool(command.get("verified_metrics", {}).get("passed")):
            raise RuntimeError(f"Command is not marked as verified: {path.name}")
        if float(command["action_scale_deg"]) != action_scale:
            raise ValueError(f"Action scale mismatch in {path.name}.")
        if float(command["servo_target_rate_limit_deg_s"]) != servo_rate:
            raise ValueError(f"Servo rate mismatch in {path.name}.")

        transfer = action_row(command["transfer_actions"], f"{path.name} transfer_actions")
        final = action_row(command["final_actions"], f"{path.name} final_actions")
        label = LEG_LABELS[leg_index]
        add_segment(segments, f"{label}_transfer", 60, zero, transfer, -1)
        add_segment(segments, f"{label}_lift", 60, transfer, final, leg_index)
        add_segment(segments, f"{label}_hold", 20, final, final, leg_index)
        add_segment(segments, f"{label}_release", 40, final, transfer, leg_index)
        add_segment(segments, f"{label}_return", 80, transfer, zero, -1)
        add_segment(segments, f"{label}_neutral", 80, zero, zero, -1)
        source_commands[str(leg_index)] = path.relative_to(REPO_ROOT).as_posix()

    teacher = {
        "name": "domino_verified_weight_transfer_cycle",
        "type": "keyframe_sequence",
        "loop": True,
        "action_scale_deg": action_scale,
        "servo_target_rate_limit_deg_s": servo_rate,
        "leg_order": leg_order,
        "total_steps": sum(int(segment["steps"]) for segment in segments),
        "source_cycle_report": report_path.relative_to(REPO_ROOT).as_posix(),
        "source_commands": source_commands,
        "segments": segments,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(teacher, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"status": "passed", "output": str(output_path), "total_steps": teacher["total_steps"]}, indent=2))


if __name__ == "__main__":
    main()
