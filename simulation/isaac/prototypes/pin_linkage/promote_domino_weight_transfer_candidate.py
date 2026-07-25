"""Promote a passing weight-transfer search row into a command config."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


parser = argparse.ArgumentParser(description="Select a verified Domino weight-transfer command.")
parser.add_argument("report", help="Passing weight-transfer search report.")
parser.add_argument("output", help="Command JSON to write.")
parser.add_argument(
    "--selection",
    choices=["score", "minimum-recovery-drift"],
    default="minimum-recovery-drift",
    help="Ranking applied to passing rows.",
)
args = parser.parse_args()


def candidate_rows(report: dict) -> list[dict]:
    rows: list[dict] = []
    if isinstance(report.get("best"), dict):
        rows.append(dict(report["best"]))
    for generation in report.get("generation_reports", []):
        generation_index = int(generation.get("generation", -1))
        for row in generation.get("top", []):
            candidate = dict(row)
            candidate.setdefault("generation", generation_index)
            rows.append(candidate)
    unique = {}
    for row in rows:
        key = (int(row.get("generation", -1)), int(row.get("candidate_index", -1)))
        unique[key] = row
    return [row for row in unique.values() if bool(row.get("passed"))]


def recovery_drift(row: dict) -> float:
    displacement = np.asarray(row.get("body_displacement_m", [np.inf, np.inf, np.inf]), dtype=np.float64)
    return float(np.linalg.norm(displacement[:2]))


report_path = Path(args.report).expanduser().resolve()
output_path = Path(args.output).expanduser().resolve()
report = json.loads(report_path.read_text(encoding="utf-8"))
rows = candidate_rows(report)
if not rows:
    raise SystemExit("The report contains no passing candidate to promote.")

if args.selection == "score":
    selected = max(rows, key=lambda row: float(row.get("score", -np.inf)))
else:
    selected = min(
        rows,
        key=lambda row: (
            recovery_drift(row),
            -float(row.get("target_peak_clearance_m", 0.0)),
            float(row.get("recovery_tilt_deg", np.inf)),
        ),
    )

target_leg_index = int(report["target_leg_index"])
target_leg = report["target_leg"]
command = {
    "name": f"domino_{target_leg['leg_id']}_weight_transfer",
    "selection": str(args.selection),
    "source_generation": int(selected.get("generation", -1)),
    "source_candidate_index": int(selected.get("candidate_index", -1)),
    "target_leg_index": target_leg_index,
    "target_leg": target_leg,
    "action_scale_deg": float(report["action_scale_deg"]),
    "servo_target_rate_limit_deg_s": float(report["servo_target_rate_limit_deg_s"]),
    "transfer_steps": int(report["transfer_steps"]),
    "lift_steps": int(report["lift_steps"]),
    "hold_steps": int(report["hold_steps"]),
    "release_steps": int(report.get("release_steps", 0)),
    "return_steps": int(report.get("return_steps", 0)),
    "neutral_steps": int(report.get("neutral_steps", 0)),
    "transfer_actions": selected["transfer_actions"],
    "final_actions": selected["final_actions"],
    "verified_metrics": selected,
}
output_path.parent.mkdir(parents=True, exist_ok=True)
output_path.write_text(json.dumps(command, indent=2), encoding="utf-8")
print(
    json.dumps(
        {
            "status": "passed",
            "selection": str(args.selection),
            "source_generation": command["source_generation"],
            "source_candidate_index": command["source_candidate_index"],
            "recovery_planar_drift_m": round(recovery_drift(selected), 6),
            "target_peak_clearance_m": selected["target_peak_clearance_m"],
            "output": str(output_path),
        },
        indent=2,
    )
)
