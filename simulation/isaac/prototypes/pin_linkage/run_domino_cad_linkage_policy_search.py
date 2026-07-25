"""Run a small policy-search loop on the Domino CAD-linkage scene.

This runner deliberately avoids Isaac Lab's Gym/RSL-RL stack so it can run
inside the current Isaac Sim install without extra Python packages. It still
uses the same 12-actuator Domino linkage builder and requires the exported
Domino STL visuals by default.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import random
import sys
import traceback

import numpy as np

CONTRACT_DIR = Path(__file__).resolve().parents[1] / "quadruped"
if str(CONTRACT_DIR) not in sys.path:
    sys.path.insert(0, str(CONTRACT_DIR))


parser = argparse.ArgumentParser(description="Train a lightweight Domino CAD-linkage gait policy search.")
parser.add_argument("--generations", type=int, default=2, help="Number of policy-search generations.")
parser.add_argument("--population-size", type=int, default=4, help="Candidates evaluated per generation.")
parser.add_argument("--episode-steps", type=int, default=180, help="Physics steps per candidate episode.")
parser.add_argument("--seed", type=int, default=11, help="Policy-search random seed.")
parser.add_argument("--action-scale-deg", type=float, default=0.35, help="Drive target offset for normalized action=1.")
parser.add_argument("--gait-frequency-hz", type=float, default=1.0, help="Nominal gait phase frequency.")
parser.add_argument("--candidate-noise", type=float, default=0.08, help="Initial perturbation scale.")
parser.add_argument("--learning-rate", type=float, default=0.45, help="Best-candidate blend rate per generation.")
parser.add_argument("--top-k", type=int, default=2, help="Number of best candidates used for the update.")
parser.add_argument("--floating-height-m", type=float, default=0.12, help="Initial floating body height.")
parser.add_argument("--min-body-height-m", type=float, default=0.035, help="Height below which a candidate is penalized.")
parser.add_argument("--report-path", default="", help="Optional JSON report output path.")
parser.add_argument("--policy-output-path", default="", help="Optional JSON path for the best gait policy.")
parser.add_argument("--save-usd", default="", help="Optional path to save the generated USD stage.")
parser.add_argument("--no-print-report", action="store_true", help="Write reports without printing the full JSON.")
parser.add_argument("--actual-cad-mesh-dir", default="", help="Optional override for the Domino STL mesh folder.")
parser.add_argument("--headless", action="store_true", help="Run without an Isaac Sim viewport.")
parser.add_argument("--device", default="cuda:0", help="Simulation device passed to the Isaac core SimulationContext.")
parser.add_argument(
    "--disable-actual-cad-visuals",
    action="store_true",
    help="Use only proxy cube/sphere visuals. Blocked in visible runs unless --allow-proxy-visuals is also set.",
)
parser.add_argument(
    "--allow-proxy-visuals",
    action="store_true",
    help="Allow visible proxy-only debugging. Do not use this for portfolio screenshots.",
)
parser.add_argument(
    "--graceful-close",
    action="store_true",
    help="Call SimulationApp.close() before exit. Disabled by default because it can hang on some Windows setups.",
)
parser.add_argument("--hold-open", action="store_true", help="Keep the visible Isaac Sim window open after training.")
args_cli = parser.parse_args()

if not args_cli.headless and args_cli.disable_actual_cad_visuals and not args_cli.allow_proxy_visuals:
    raise SystemExit(
        "Visible Domino policy-search runs must use the actual exported CAD STL visuals. "
        "Remove --disable-actual-cad-visuals, run headless, or pass --allow-proxy-visuals only for proxy debugging."
    )

os.environ.setdefault("WARP_CACHE_PATH", str((Path.cwd() / "simulation" / "isaac" / "out" / "warp_cache").resolve()))

from isaacsim import SimulationApp  # noqa: E402

launch_config = {"headless": bool(args_cli.headless)}
if sys.platform.startswith("win") and not args_cli.headless:
    launch_config["extra_args"] = ["--/app/vulkan=false"]
simulation_app = SimulationApp(launch_config)

import omni.usd  # noqa: E402
from isaacsim.core.api import SimulationContext  # noqa: E402
from isaacsim.core.prims import SingleRigidPrim  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402

from domino_action_contract import ACTION_JOINT_NAMES, EXPECTED_ACTION_COUNT, action_group_counts, per_leg_action_layout  # noqa: E402
from domino_cad_linkage_builder import (  # noqa: E402
    DominoCadLinkageBuildConfig,
    build_domino_four_12_floating_linkage,
    local_endpoint,
    set_drive_targets_from_actions,
    validate_domino_actual_cad_visuals,
)
from domino_reference_gait import (  # noqa: E402
    REFERENCE_GAIT_PARAMETER_NAMES,
    candidate_with_defaults,
    default_reference_candidate,
    reference_actions_for_base_phases,
)


TRAINED_PARAMETER_NAMES = [
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
    "leg_phase_0",
    "leg_phase_1",
    "leg_phase_2",
    "leg_phase_3",
]


def to_numpy(value) -> np.ndarray:
    if hasattr(value, "detach"):
        return value.detach().cpu().numpy()
    return np.asarray(value)


def tensor_list(value) -> list[float]:
    return [round(float(item), 6) for item in to_numpy(value).reshape(-1)]


def world_endpoint(view: SingleRigidPrim, local_point: Gf.Vec3f) -> np.ndarray:
    position, orientation = view.get_world_pose()
    position = to_numpy(position).astype(np.float64).reshape(3)
    orientation = to_numpy(orientation).astype(np.float64).reshape(4)
    w, x, y, z = [float(v) for v in orientation]
    rotation = np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    local = np.array([float(local_point[0]), float(local_point[1]), float(local_point[2])], dtype=np.float64)
    return position + rotation @ local


def capture_rigid_body_states(views: dict[str, SingleRigidPrim]) -> dict[str, dict[str, np.ndarray]]:
    states = {}
    for name, view in views.items():
        position, orientation = view.get_world_pose()
        states[name] = {
            "position": to_numpy(position).astype(np.float32),
            "orientation": to_numpy(orientation).astype(np.float32),
        }
    return states


def reset_rigid_bodies_to_initial(
    views: dict[str, SingleRigidPrim],
    initial_states: dict[str, dict[str, np.ndarray]],
    drive_specs: list[dict],
) -> None:
    for spec in drive_specs:
        spec["target_attr"].Set(float(spec["center_deg"]))
        spec["current_target_deg"] = float(spec["center_deg"])
    for name, view in views.items():
        state = initial_states[name]
        rigid_backend = view._rigid_prim_view._backend_utils
        rigid_device = view._rigid_prim_view._device
        position = rigid_backend.convert(np.expand_dims(state["position"], axis=0), device=rigid_device)
        orientation = rigid_backend.convert(np.expand_dims(state["orientation"], axis=0), device=rigid_device)
        zero_velocity = rigid_backend.convert(np.zeros((1, 6), dtype=np.float32), device=rigid_device)
        view._rigid_prim_view.set_world_poses(positions=position, orientations=orientation)
        view._rigid_prim_view.set_velocities(zero_velocity)


def clamp_candidate(candidate: dict) -> dict:
    candidate = candidate_with_defaults(candidate)
    for key in ("lower_amp", "upper_amp"):
        candidate[key] = float(np.clip(candidate[key], 0.05, 1.0))
    candidate["shoulder_amp"] = float(np.clip(candidate["shoulder_amp"], 0.0, 0.4))
    for key in ("lower_bias", "upper_bias"):
        candidate[key] = float(np.clip(candidate[key], -0.8, 0.8))
    candidate["shoulder_bias"] = float(np.clip(candidate["shoulder_bias"], -0.3, 0.3))
    candidate["frequency_scale"] = float(np.clip(candidate["frequency_scale"], 0.35, 1.8))
    return candidate


def perturb_candidate(center: dict, rng: random.Random, noise: float) -> dict:
    candidate = dict(center)
    for key in TRAINED_PARAMETER_NAMES:
        scale = float(noise)
        if key.endswith("_phase") or key.startswith("leg_phase_"):
            scale *= math.pi
        candidate[key] = float(candidate[key]) + rng.gauss(0.0, scale)
    candidate["name"] = "policy_search_candidate"
    return clamp_candidate(candidate)


def average_candidates(candidates: list[dict]) -> dict:
    averaged = dict(candidates[0])
    for key in TRAINED_PARAMETER_NAMES:
        averaged[key] = float(np.mean([float(candidate[key]) for candidate in candidates]))
    averaged["name"] = "policy_search_center"
    return clamp_candidate(averaged)


def visible_proxy_count(stage) -> dict[str, int]:
    counts = {
        "visible_robot_proxy_cubes": 0,
        "visible_robot_proxy_spheres": 0,
        "visible_ground_cubes": 0,
        "mesh_count": 0,
    }
    for prim in stage.Traverse():
        type_name = prim.GetTypeName()
        if type_name == "Mesh":
            counts["mesh_count"] += 1
        if type_name not in {"Cube", "Sphere"}:
            continue
        imageable = UsdGeom.Imageable(prim)
        visibility = imageable.ComputeVisibility()
        if visibility == UsdGeom.Tokens.invisible:
            continue
        prim_path = str(prim.GetPath())
        if prim_path.startswith("/World/Ground"):
            if type_name == "Cube":
                counts["visible_ground_cubes"] += 1
            continue
        if type_name == "Cube":
            counts["visible_robot_proxy_cubes"] += 1
        elif type_name == "Sphere":
            counts["visible_robot_proxy_spheres"] += 1
    return counts


def physics_revolute_joint_count(stage) -> int:
    return sum(1 for prim in stage.Traverse() if prim.GetTypeName() == "PhysicsRevoluteJoint")


def body_reference_position(views: dict[str, SingleRigidPrim]) -> np.ndarray:
    position, _ = views["body_reference"].get_world_pose()
    return to_numpy(position).astype(np.float64).reshape(3)


def max_loop_error(linkage: dict, views: dict[str, SingleRigidPrim]) -> float:
    errors = []
    for check in linkage["loop_checks"]:
        pivot = np.array(check["pivot"], dtype=np.float64)
        body_a = linkage["bodies"][check["body_a"]]
        body_b = linkage["bodies"][check["body_b"]]
        world_a = world_endpoint(views[check["body_a"]], local_endpoint(pivot, body_a["center"]))
        world_b = world_endpoint(views[check["body_b"]], local_endpoint(pivot, body_b["center"]))
        errors.append(float(np.linalg.norm(world_a - world_b)))
    return max(errors) if errors else 0.0


def evaluate_candidate(
    candidate: dict,
    linkage: dict,
    views: dict[str, SingleRigidPrim],
    initial_states: dict[str, dict[str, np.ndarray]],
    sim,
    generation: int,
    candidate_index: int,
) -> dict:
    reset_rigid_bodies_to_initial(views, initial_states, linkage["drives"])
    start_position = body_reference_position(views)
    sim_dt = float(sim.get_physics_dt())
    base_phases = []
    actions_by_step = []
    min_body_height = float("inf")
    max_body_speed = 0.0
    max_action_abs = np.zeros(EXPECTED_ACTION_COUNT, dtype=np.float64)
    finite = True
    max_error = 0.0

    for step in range(int(args_cli.episode_steps)):
        time_s = step * sim_dt
        base_phase = 2.0 * math.pi * float(args_cli.gait_frequency_hz) * time_s
        base_phases.append(base_phase)
        actions = reference_actions_for_base_phases(candidate, np.asarray([base_phase], dtype=np.float64))[0]
        actions_by_step.append(actions)
        max_action_abs = np.maximum(max_action_abs, np.abs(actions.astype(np.float64)))
        set_drive_targets_from_actions(linkage["drives"], actions, float(args_cli.action_scale_deg))
        sim.step(render=not bool(args_cli.headless))

        position = body_reference_position(views)
        linear_velocity = to_numpy(views["body_reference"].get_linear_velocity()).astype(np.float64).reshape(3)
        max_body_speed = max(max_body_speed, float(np.linalg.norm(linear_velocity)))
        min_body_height = min(min_body_height, float(position[2]))
        max_error = max(max_error, max_loop_error(linkage, views))
        if not np.isfinite(position).all() or not np.isfinite(linear_velocity).all():
            finite = False
            break

    end_position = body_reference_position(views)
    displacement = end_position - start_position
    actions_np = np.asarray(actions_by_step, dtype=np.float64) if actions_by_step else np.zeros((1, EXPECTED_ACTION_COUNT))
    mean_abs_action = float(np.mean(np.abs(actions_np)))
    height_shortfall = max(0.0, float(args_cli.min_body_height_m) - float(min_body_height))
    score = (
        float(displacement[0])
        - 0.45 * abs(float(displacement[1]))
        - 0.10 * abs(float(displacement[2]))
        - 12.0 * height_shortfall
        - 4.0 * max_error
        - 0.01 * mean_abs_action
    )
    if not finite:
        score -= 100.0

    return {
        "generation": int(generation),
        "candidate_index": int(candidate_index),
        "score": round(float(score), 8),
        "finite": bool(finite),
        "forward_displacement_m": round(float(displacement[0]), 6),
        "lateral_drift_m": round(float(displacement[1]), 6),
        "vertical_drift_m": round(float(displacement[2]), 6),
        "min_body_height_m": round(float(min_body_height), 6),
        "max_body_speed_m_s": round(float(max_body_speed), 6),
        "max_loop_closure_error_m": round(float(max_error), 8),
        "mean_abs_action": round(mean_abs_action, 6),
        "max_abs_action_by_channel": [round(float(value), 6) for value in max_action_abs.tolist()],
        "candidate": candidate,
    }


def main() -> None:
    rng = random.Random(int(args_cli.seed))
    sim = SimulationContext(
        physics_dt=0.005,
        rendering_dt=0.005,
        stage_units_in_meters=1.0,
        device=str(args_cli.device),
    )
    if not args_cli.headless:
        set_camera_view(
            eye=[0.72, -0.68, 0.38],
            target=[0.10, 0.06, 0.05],
            camera_prim_path="/OmniverseKit_Persp",
        )
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    linkage = build_domino_four_12_floating_linkage(
        stage,
        DominoCadLinkageBuildConfig(
            floating_height_m=float(args_cli.floating_height_m),
            include_actual_cad_visuals=not bool(args_cli.disable_actual_cad_visuals),
            hide_proxy_visuals_when_actual_cad=True,
            actual_cad_mesh_dir=str(args_cli.actual_cad_mesh_dir),
        ),
    )
    if not linkage.get("actual_cad_visual") and not args_cli.disable_actual_cad_visuals:
        raise RuntimeError("Domino STL visual attachment failed; refusing to run a proxy-only policy search.")
    if len(linkage["drives"]) != EXPECTED_ACTION_COUNT:
        raise RuntimeError(f"Expected {EXPECTED_ACTION_COUNT} actuator drives, found {len(linkage['drives'])}.")

    if args_cli.save_usd:
        save_path = Path(args_cli.save_usd).expanduser().resolve()
        save_path.parent.mkdir(parents=True, exist_ok=True)
        stage.GetRootLayer().Export(str(save_path))

    views = {
        name: SingleRigidPrim(str(body["path"]), name=f"{name}_view", reset_xform_properties=False)
        for name, body in linkage["bodies"].items()
    }
    sim.reset()
    for view in views.values():
        view.initialize()

    initial_states = capture_rigid_body_states(views)
    center = default_reference_candidate()
    best_result = None
    history = []
    top_k = max(1, min(int(args_cli.top_k), int(args_cli.population_size)))

    for generation in range(max(1, int(args_cli.generations))):
        noise = float(args_cli.candidate_noise) * (0.65**generation)
        candidates = [clamp_candidate(center)]
        while len(candidates) < max(1, int(args_cli.population_size)):
            candidates.append(perturb_candidate(center, rng, noise))

        generation_results = []
        for candidate_index, candidate in enumerate(candidates):
            result = evaluate_candidate(candidate, linkage, views, initial_states, sim, generation, candidate_index)
            generation_results.append(result)
            history.append(result)
            if best_result is None or result["score"] > best_result["score"]:
                best_result = result

        ranked = sorted(generation_results, key=lambda item: float(item["score"]), reverse=True)
        center_update = average_candidates([item["candidate"] for item in ranked[:top_k]])
        blended = dict(center)
        for key in TRAINED_PARAMETER_NAMES:
            blended[key] = ((1.0 - float(args_cli.learning_rate)) * float(center[key])) + (
                float(args_cli.learning_rate) * float(center_update[key])
            )
        blended["name"] = "policy_search_center"
        center = clamp_candidate(blended)

    if best_result is None:
        raise RuntimeError("Policy search did not evaluate any candidates.")

    reset_rigid_bodies_to_initial(views, initial_states, linkage["drives"])
    visual_counts = validate_domino_actual_cad_visuals(stage, linkage)

    report = {
        "status": "passed",
        "runner": "standalone_policy_search",
        "visual_fidelity": linkage.get("visual_fidelity"),
        "actual_cad_visual": linkage.get("actual_cad_visual"),
        "cad_source": linkage.get("cad_source"),
        "actual_cad_visuals": linkage.get("actual_cad_visuals"),
        "visible_geometry_counts": visual_counts,
        "physics_revolute_joint_count": physics_revolute_joint_count(stage),
        "rigid_body_count": len(linkage["bodies"]),
        "action_count": len(linkage["drives"]),
        "action_names": ACTION_JOINT_NAMES,
        "action_group_counts": action_group_counts(),
        "per_leg_action_layout": per_leg_action_layout(),
        "training": {
            "method": "cross_entropy_policy_search_over_reference_gait_parameters",
            "generations": int(args_cli.generations),
            "population_size": int(args_cli.population_size),
            "top_k": top_k,
            "episode_steps": int(args_cli.episode_steps),
            "action_scale_deg": float(args_cli.action_scale_deg),
            "gait_frequency_hz": float(args_cli.gait_frequency_hz),
            "trained_parameters": REFERENCE_GAIT_PARAMETER_NAMES,
        },
        "best_result": best_result,
        "final_center_policy": center,
        "history": history,
    }

    if args_cli.report_path:
        report_path = Path(args_cli.report_path).expanduser().resolve()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    if args_cli.policy_output_path:
        policy_path = Path(args_cli.policy_output_path).expanduser().resolve()
        policy_path.parent.mkdir(parents=True, exist_ok=True)
        policy_path.write_text(json.dumps(best_result["candidate"], indent=2), encoding="utf-8")

    if not args_cli.no_print_report:
        print(json.dumps(report, indent=2), flush=True)

    if args_cli.hold_open:
        print("Policy search complete; keeping Isaac Sim open. Close the Isaac window to exit.", flush=True)
        while simulation_app.is_running():
            simulation_app.update()


if __name__ == "__main__":
    exit_code = 0
    try:
        main()
    except Exception:
        exit_code = 1
        traceback.print_exc()
    finally:
        if args_cli.graceful_close:
            simulation_app.close(wait_for_replicator=False, skip_cleanup=True)
        os._exit(exit_code)
