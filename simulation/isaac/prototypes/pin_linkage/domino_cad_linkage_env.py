"""Isaac Lab DirectRLEnv wrapper for the floating Domino CAD linkage."""

from __future__ import annotations

from collections.abc import Sequence
import json
import math
import time

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass
import omni.usd
from isaacsim.core.prims import SingleRigidPrim
from pxr import Gf, UsdGeom

from domino_cad_linkage_builder import (
    DominoCadLinkageBuildConfig,
    build_domino_four_12_floating_linkage,
    create_static_ground_box,
    create_static_stairs_terrain,
    local_endpoint,
    set_drive_targets_from_actions,
)
from domino_action_contract import (
    EXPECTED_ACTION_COUNT,
    EXPECTED_FOOT_COUNT,
    VALIDATED_INITIAL_POLICY_ACTION_SCALE_DEG,
)
from domino_locomotion_rewards import command_motion_terms
from domino_reference_gait import (
    PHASE_OFFSETS_RAD,
    REFERENCE_GAIT_PARAMETER_NAMES,
    SIDE_SIGNS,
    is_keyframe_sequence,
    reference_actions_for_base_phases,
    reference_actions_for_steps,
    reference_desired_stance_for_steps,
)


CAD_LINKAGE_OBSERVATION_DIM = 61
CAD_LINKAGE_REFERENCE_OBSERVATION_DIM = CAD_LINKAGE_OBSERVATION_DIM + EXPECTED_ACTION_COUNT
DIAGONAL_TROT_PHASE_OFFSETS_RAD = np.array([0.0, math.pi, 0.0, math.pi], dtype=np.float32)


def to_numpy(value) -> np.ndarray:
    if hasattr(value, "detach"):
        return value.detach().cpu().numpy()
    return np.asarray(value)


def quat_wxyz_to_rotation_matrix(orientation: np.ndarray) -> np.ndarray:
    w, x, y, z = [float(value) for value in np.asarray(orientation, dtype=np.float64).reshape(-1)[:4]]
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def yaw_from_quat_wxyz(orientation: np.ndarray) -> float:
    w, x, y, z = [float(value) for value in np.asarray(orientation, dtype=np.float64).reshape(-1)[:4]]
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_angle_rad(value: float) -> float:
    return math.atan2(math.sin(float(value)), math.cos(float(value)))


def projected_gravity_from_quat(orientation: np.ndarray) -> np.ndarray:
    rotation = quat_wxyz_to_rotation_matrix(orientation)
    gravity_world = np.array([0.0, 0.0, -1.0], dtype=np.float64)
    return (rotation.T @ gravity_world).astype(np.float32)


def rotation_delta_deg(rotation_a: np.ndarray, rotation_b: np.ndarray) -> float:
    relative = np.asarray(rotation_a, dtype=np.float64).T @ np.asarray(rotation_b, dtype=np.float64)
    cosine = max(-1.0, min(1.0, 0.5 * (float(np.trace(relative)) - 1.0)))
    return math.degrees(math.acos(cosine))


def calculate_ground_size_m(num_envs: int, env_spacing_m: float, margin_m: float = 2.0, min_size_m: float = 4.0) -> float:
    grid_width = max(1, math.ceil(math.sqrt(max(num_envs, 1))))
    cloned_env_span_m = max(0.0, float(grid_width - 1) * float(env_spacing_m))
    return max(float(min_size_m), cloned_env_span_m + (2.0 * float(margin_m)))


def grid_env_origins_m(num_envs: int, env_spacing_m: float) -> np.ndarray:
    grid_width = max(1, math.ceil(math.sqrt(max(num_envs, 1))))
    origins = []
    for env_index in range(max(num_envs, 1)):
        row = env_index // grid_width
        column = env_index % grid_width
        origins.append((float(column) * float(env_spacing_m), float(row) * float(env_spacing_m), 0.0))
    return np.asarray(origins, dtype=np.float32)


@configclass
class DominoCadLinkageEnvCfg(DirectRLEnvCfg):
    decimation = 6
    episode_length_s = 3.0
    action_scale_deg = VALIDATED_INITIAL_POLICY_ACTION_SCALE_DEG
    servo_target_rate_limit_deg_s = 180.0
    reset_settle_steps = 15
    visible_step_delay_s = 0.0
    enable_gravity = True
    action_space = EXPECTED_ACTION_COUNT
    observation_space = CAD_LINKAGE_OBSERVATION_DIM
    state_space = 0

    # The closed-loop prototype commands raw USD DriveAPI attributes. Fabric
    # does not propagate those live parameter edits into PhysX, so keep USD
    # synchronization enabled until the mechanism is promoted to a tensor-
    # controlled reduced-coordinate articulation.
    sim: SimulationCfg = SimulationCfg(dt=1 / 300, render_interval=decimation, device="cpu", use_fabric=False)
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=1, env_spacing=2.0, replicate_physics=False)

    floating_height_m = 0.12
    min_height_m = 0.02
    target_height_m = 0.129
    max_tilt_deg = 75.0
    command_x_m_s = 0.0
    command_y_m_s = 0.0
    command_yaw_rad_s = 0.0
    gait_frequency_hz = 1.0
    locomotion_command_threshold = 0.01
    reference_action_tracking_reward_scale = 0.0
    reference_action_tracking_sigma = 0.35
    reference_action_mse_reward_scale = 0.0
    include_reference_actions_in_observation = False
    reference_gait_name = ""
    reference_gait_lower_amp = 0.35
    reference_gait_upper_amp = 0.55
    reference_gait_shoulder_amp = 0.22
    reference_gait_lower_bias = -0.55
    reference_gait_upper_bias = 0.05
    reference_gait_shoulder_bias = 0.0
    reference_gait_lower_phase = 0.0
    reference_gait_upper_phase = math.pi / 2.0
    reference_gait_shoulder_phase = math.pi / 2.0
    reference_gait_frequency_scale = 1.0
    reference_gait_leg_phase_0 = PHASE_OFFSETS_RAD[0]
    reference_gait_leg_phase_1 = PHASE_OFFSETS_RAD[1]
    reference_gait_leg_phase_2 = PHASE_OFFSETS_RAD[2]
    reference_gait_leg_phase_3 = PHASE_OFFSETS_RAD[3]
    reference_gait_shoulder_sign_0 = SIDE_SIGNS[0]
    reference_gait_shoulder_sign_1 = SIDE_SIGNS[1]
    reference_gait_shoulder_sign_2 = SIDE_SIGNS[2]
    reference_gait_shoulder_sign_3 = SIDE_SIGNS[3]
    reference_gait_lower_sign_0 = 1.0
    reference_gait_lower_sign_1 = 1.0
    reference_gait_lower_sign_2 = 1.0
    reference_gait_lower_sign_3 = 1.0
    reference_gait_upper_sign_0 = 1.0
    reference_gait_upper_sign_1 = 1.0
    reference_gait_upper_sign_2 = 1.0
    reference_gait_upper_sign_3 = 1.0
    reference_sequence_json = ""
    foot_proxy_radius_m = 0.024
    include_actual_cad_visuals = True
    hide_proxy_visuals_when_actual_cad = True
    fixed_base = False
    closure_model = "passive"
    actual_cad_mesh_dir = ""
    use_actual_cad_foot_collision = True
    foot_contact_mode = "actual_cad_visual_bottom"
    use_calibrated_neutral_pose = True
    align_actual_cad_visual_bottom_to_ground = True
    actual_cad_ground_clearance_m = 0.001
    enable_body_collisions = True
    terminate_on_non_foot_ground_contact = True
    terminate_on_joint_separation = True
    max_joint_separation_m = 0.005
    non_foot_ground_contact_margin_m = 0.001
    foot_contact_epsilon_m = 0.004
    target_swing_clearance_m = 0.012
    use_actual_cad_visual_foot_bottom_for_rewards = False
    ground_size_m = 10.0
    ground_margin_m = 2.0
    ground_thickness_m = 0.05
    terrain_type = "flat"
    stairs_step_count = 7
    stairs_step_depth_m = 0.16
    stairs_step_height_m = 0.018
    stairs_width_m = 1.20
    stairs_start_x_m = 0.45
    stairs_top_platform_length_m = 0.50
    alive_reward_scale = 1.0
    height_reward_scale = -20.0
    flat_orientation_reward_scale = -1.5
    vertical_velocity_reward_scale = -0.05
    angular_velocity_reward_scale = -0.02
    command_velocity_reward_scale = -4.0
    command_velocity_tracking_reward_scale = 1.0
    command_velocity_tracking_sigma = 0.06
    command_progress_reward_scale = 2.0
    command_stagnation_penalty_scale = 0.0
    command_stagnation_speed_m_s = 0.03
    use_displacement_velocity_rewards = True
    lateral_drift_reward_scale = 0.0
    yaw_drift_reward_scale = 0.0
    command_yaw_reward_scale = -1.0
    command_yaw_tracking_reward_scale = 0.25
    command_yaw_tracking_sigma = 0.25
    action_reward_scale = -0.02
    action_rate_reward_scale = -0.02
    foot_contact_reward_scale = 0.15
    gait_contact_reward_scale = 0.25
    stance_contact_reward_scale = 0.20
    swing_contact_penalty_scale = -0.40
    foot_clearance_reward_scale = 0.20


class DominoCadLinkageEnv(DirectRLEnv):
    cfg: DominoCadLinkageEnvCfg

    def __init__(self, cfg: DominoCadLinkageEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        self._body_views_by_env = []
        for env_index, linkage in enumerate(self._linkages):
            env_views = {
                name: SingleRigidPrim(
                    str(body["path"]),
                    name=f"env_{env_index}_{name}_view",
                    reset_xform_properties=False,
                )
                for name, body in linkage["bodies"].items()
            }
            for view in env_views.values():
                view.initialize()
            self._body_views_by_env.append(env_views)
        if len(self._body_views_by_env) != self.num_envs:
            raise RuntimeError(
                "Domino environment construction count mismatch: "
                f"num_envs={self.num_envs}, cfg_num_envs={self.cfg.scene.num_envs}, "
                f"env_origins={len(self._env_origins_np)}, linkages={len(self._linkages)}, "
                f"body_view_groups={len(self._body_views_by_env)}."
            )
        self._body_views = self._body_views_by_env[0]
        self._initial_body_states = [
            self._capture_rigid_body_states(env_index) for env_index in range(self.num_envs)
        ]
        self._apply_actual_cad_visual_lift()
        for env_index, drive_specs in enumerate(self._drive_specs_by_env):
            if len(drive_specs) != EXPECTED_ACTION_COUNT:
                raise RuntimeError(f"Expected {EXPECTED_ACTION_COUNT} actuator drives in env {env_index}, found {len(drive_specs)}.")
        self._actions = torch.zeros(self.num_envs, self.cfg.action_space, device=self.device)
        self._commands = torch.tensor(
            [self.cfg.command_x_m_s, self.cfg.command_y_m_s, self.cfg.command_yaw_rad_s],
            dtype=torch.float32,
            device=self.device,
        ).repeat(self.num_envs, 1)
        self._gait_phase_offsets_rad_np = np.linspace(
            0.0,
            2.0 * math.pi,
            num=self.num_envs,
            endpoint=False,
            dtype=np.float32,
        )
        self._target_deg = torch.tensor(
            [[float(spec["center_deg"]) for spec in drive_specs] for drive_specs in self._drive_specs_by_env],
            dtype=torch.float32,
            device=self.device,
        )
        self._center_deg = self._target_deg.clone()
        self._previous_body_reference_positions = torch.tensor(
            np.vstack([self._body_reference_state(env_index)[0] for env_index in range(self.num_envs)]),
            dtype=torch.float32,
            device=self.device,
        )
        self._previous_actions = torch.zeros_like(self._actions)
        self._reset_settle_steps_remaining = torch.zeros(
            self.num_envs,
            dtype=torch.int64,
            device=self.device,
        )
        self._max_tilt_rad = math.radians(float(self.cfg.max_tilt_deg))
        self._last_reward_terms_unscaled: dict[str, torch.Tensor] = {}
        self._last_reward_terms_scaled: dict[str, torch.Tensor] = {}
        self._last_reward_terms_total = torch.zeros(self.num_envs, dtype=torch.float32, device=self.device)

    def _setup_scene(self) -> None:
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        light_cfg = sim_utils.DomeLightCfg(intensity=1500.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)
        requested_num_envs = int(self.cfg.scene.num_envs)
        if hasattr(self.scene, "env_origins"):
            scene_origins = to_numpy(self.scene.env_origins).astype(np.float32).reshape(-1, 3)
            self._env_origins_np = (
                scene_origins
                if len(scene_origins) == requested_num_envs
                else grid_env_origins_m(requested_num_envs, self.cfg.scene.env_spacing)
            )
        else:
            self._env_origins_np = grid_env_origins_m(requested_num_envs, self.cfg.scene.env_spacing)
        self._env_origins = torch.tensor(self._env_origins_np, dtype=torch.float32, device=self.device)
        ground_size_m = max(
            float(self.cfg.ground_size_m),
            calculate_ground_size_m(self.cfg.scene.num_envs, self.cfg.scene.env_spacing, self.cfg.ground_margin_m),
        )
        ground_center_xy_m = (
            0.5 * float(np.min(self._env_origins_np[:, 0]) + np.max(self._env_origins_np[:, 0])),
            0.5 * float(np.min(self._env_origins_np[:, 1]) + np.max(self._env_origins_np[:, 1])),
        )
        self._ground_box = create_static_ground_box(
            stage,
            "/World/Ground",
            size_m=ground_size_m,
            thickness_m=self.cfg.ground_thickness_m,
            center_xy_m=ground_center_xy_m,
            visible=True,
            collision=True,
        )
        self._terrain_report = {
            "type": str(self.cfg.terrain_type),
            "ground_box": self._ground_box,
            "visible_ground_box": None,
            "stairs": [],
            "visible_stairs": [],
        }
        self._linkages = []
        self._drive_specs_by_env = []
        for env_index, origin in enumerate(self._env_origins_np):
            env_root = f"/World/envs/env_{env_index}/DominoFour12FloatingBody"
            UsdGeom.Xform.Define(stage, f"/World/envs/env_{env_index}")
            if str(self.cfg.terrain_type).lower() == "stairs":
                self._terrain_report["stairs"].append(
                    create_static_stairs_terrain(
                        stage,
                        f"/World/terrain/env_{env_index}/stairs",
                        tuple(float(value) for value in origin),
                        step_count=int(self.cfg.stairs_step_count),
                        step_depth_m=float(self.cfg.stairs_step_depth_m),
                        step_height_m=float(self.cfg.stairs_step_height_m),
                        width_m=float(self.cfg.stairs_width_m),
                        start_x_m=float(self.cfg.stairs_start_x_m),
                        top_platform_length_m=float(self.cfg.stairs_top_platform_length_m),
                        visible=True,
                        collision=True,
                    )
                )
            linkage = build_domino_four_12_floating_linkage(
                stage,
                DominoCadLinkageBuildConfig(
                    root_prim_path=env_root,
                    world_translation_m=tuple(float(value) for value in origin),
                    floating_height_m=self.cfg.floating_height_m,
                    enable_gravity=bool(self.cfg.enable_gravity),
                    foot_proxy_radius_m=self.cfg.foot_proxy_radius_m,
                    include_actual_cad_visuals=bool(self.cfg.include_actual_cad_visuals),
                    hide_proxy_visuals_when_actual_cad=bool(self.cfg.hide_proxy_visuals_when_actual_cad),
                    fixed_base=bool(self.cfg.fixed_base),
                    closure_model=str(self.cfg.closure_model),
                    actual_cad_mesh_dir=str(self.cfg.actual_cad_mesh_dir),
                    use_actual_cad_foot_collision=bool(self.cfg.use_actual_cad_foot_collision),
                    foot_contact_mode=str(self.cfg.foot_contact_mode),
                    use_calibrated_neutral_pose=bool(self.cfg.use_calibrated_neutral_pose),
                    align_actual_cad_visual_bottom_to_ground=bool(self.cfg.align_actual_cad_visual_bottom_to_ground),
                    actual_cad_ground_clearance_m=float(self.cfg.actual_cad_ground_clearance_m),
                    enable_body_collisions=bool(self.cfg.enable_body_collisions),
                    include_ground=False,
                ),
            )
            self._linkages.append(linkage)
            self._drive_specs_by_env.append(linkage["drives"])
        self._linkage = self._linkages[0]
        self._drive_specs = self._drive_specs_by_env[0]
        if self._linkage.get("target_height_m") is not None:
            self.cfg.target_height_m = float(self._linkage["target_height_m"])
        self._foot_tracks_by_env = [
            [
                {
                    "name": leg["id"],
                    "body": leg.get("foot_proxy_body", f"{leg['id']}_lower_closure"),
                    "pivot": np.array(
                        leg.get("foot_proxy", {}).get("center_m", linkage["points"][f"{leg['id']}_lower_closure"]),
                        dtype=np.float64,
                    ),
                    "dynamic_local": np.array(
                        leg.get("foot_proxy", {}).get(
                            "local_center_m",
                            local_endpoint(
                                leg.get("foot_proxy", {}).get(
                                    "center_m",
                                    linkage["points"][f"{leg['id']}_lower_closure"],
                                ),
                                linkage["bodies"][
                                    leg.get("foot_proxy_body", f"{leg['id']}_lower_closure")
                                ]["center"],
                            ),
                        ),
                        dtype=np.float64,
                    ),
                    "path": str(leg.get("foot_proxy", {}).get("path", "")),
                    "radius_m": float(leg.get("foot_proxy", {}).get("radius_m", self.cfg.foot_proxy_radius_m)),
                }
                for leg in linkage["legs"]
            ]
            for linkage in self._linkages
        ]
        self._actual_cad_visual_foot_tracks_by_env = [
            [
                {
                    "name": leg["id"],
                    "body": leg.get("actual_cad_visual_foot_body", leg.get("foot_proxy_body", f"{leg['id']}_lower_closure")),
                    "pivot": np.array(leg["actual_cad_visual_foot_center_m"], dtype=np.float64),
                    "dynamic_local": np.array(leg["actual_cad_visual_foot_center_local_m"], dtype=np.float64),
                    "radius_m": float(leg["actual_cad_visual_foot_radius_m"]),
                }
                for leg in linkage["legs"]
                if leg.get("actual_cad_visual_foot_center_m") is not None
            ]
            for linkage in self._linkages
        ]
        for env_index, foot_tracks in enumerate(self._foot_tracks_by_env):
            if len(foot_tracks) != EXPECTED_FOOT_COUNT:
                raise RuntimeError(f"Expected {EXPECTED_FOOT_COUNT} CAD foot proxies in env {env_index}, found {len(foot_tracks)}.")

    def _capture_rigid_body_states(self, env_index: int) -> dict[str, dict[str, np.ndarray]]:
        states = {}
        for name, view in self._body_views_by_env[env_index].items():
            position, orientation = view.get_world_pose()
            states[name] = {
                "position": to_numpy(position).astype(np.float32),
                "orientation": to_numpy(orientation).astype(np.float32),
            }
        return states

    def _env_id_list(self, env_ids: Sequence[int] | torch.Tensor | None) -> list[int]:
        if env_ids is None:
            return list(range(self.num_envs))
        if isinstance(env_ids, torch.Tensor):
            return [int(value) for value in env_ids.detach().cpu().flatten().tolist()]
        return [int(value) for value in env_ids]

    @staticmethod
    def _clone_if_inference_tensor(value: torch.Tensor) -> torch.Tensor:
        if hasattr(value, "is_inference") and value.is_inference():
            return value.clone()
        return value

    def _apply_actual_cad_visual_lift(self, env_ids: Sequence[int] | torch.Tensor | None = None) -> None:
        if not hasattr(self, "_body_views_by_env") or not self._body_views_by_env:
            return
        stage = omni.usd.get_context().get_stage()
        for env_index in self._env_id_list(env_ids):
            linkage = self._linkages[env_index]
            lift_m = float(linkage.get("actual_cad_visual_lift_m", 0.0) or 0.0)
            if lift_m <= 0.0:
                continue
            corrector_paths = linkage.get("actual_cad_visual_corrector_paths") or {}
            if not corrector_paths:
                actual_cad_visuals = linkage.get("actual_cad_visuals") or {}
                corrector_paths = actual_cad_visuals.get("corrector_paths") or {}
            if not corrector_paths:
                continue
            lift_world = np.array([0.0, 0.0, lift_m], dtype=np.float64)
            for body_name, corrector_path in corrector_paths.items():
                view = self._body_views_by_env[env_index].get(body_name)
                if view is None:
                    continue
                _, orientation = view.get_world_pose()
                local_lift = quat_wxyz_to_rotation_matrix(to_numpy(orientation)).T @ lift_world
                prim = stage.GetPrimAtPath(str(corrector_path))
                if prim.IsValid():
                    UsdGeom.XformCommonAPI(prim).SetTranslate(
                        Gf.Vec3d(float(local_lift[0]), float(local_lift[1]), float(local_lift[2]))
                    )

    def _reset_rigid_bodies_to_initial(self, env_ids: Sequence[int] | torch.Tensor | None = None) -> None:
        for env_index in self._env_id_list(env_ids):
            for spec in self._drive_specs_by_env[env_index]:
                spec["target_attr"].Set(float(spec["center_deg"]))
                spec["current_target_deg"] = float(spec["center_deg"])
            for name, view in self._body_views_by_env[env_index].items():
                state = self._initial_body_states[env_index][name]
                zero_velocity = view._backend_utils.convert(np.zeros((1, 6), dtype=np.float32), device=view._device)
                view.set_world_pose(position=state["position"], orientation=state["orientation"])
                if not (bool(self.cfg.fixed_base) and name == "body_reference"):
                    view._rigid_prim_view.set_velocities(zero_velocity)
        self._apply_actual_cad_visual_lift(env_ids)
        self.sim.forward()

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        if actions.ndim != 2 or actions.shape[0] != self.num_envs:
            raise ValueError(f"Expected action tensor shaped ({self.num_envs}, {EXPECTED_ACTION_COUNT}), received {tuple(actions.shape)}.")
        if actions.shape[1] != EXPECTED_ACTION_COUNT:
            raise ValueError(f"Expected {EXPECTED_ACTION_COUNT} actions per env, received {actions.shape[1]}.")
        self._previous_actions = self._actions.clone()
        self._actions = torch.clamp(actions.clone(), -1.0, 1.0)
        settling = self._reset_settle_steps_remaining > 0
        if bool(torch.any(settling)):
            self._actions[settling] = 0.0
            self._reset_settle_steps_remaining = torch.clamp(
                self._reset_settle_steps_remaining - settling.to(dtype=torch.int64),
                min=0,
            )
        target_rows = []
        max_target_delta_deg = None
        if float(self.cfg.servo_target_rate_limit_deg_s) > 0.0:
            max_target_delta_deg = float(self.cfg.servo_target_rate_limit_deg_s) * float(self.step_dt)
        for env_index, drive_specs in enumerate(self._drive_specs_by_env):
            targets = set_drive_targets_from_actions(
                drive_specs,
                self._actions[env_index].detach().cpu().tolist(),
                self.cfg.action_scale_deg,
                max_target_delta_deg=max_target_delta_deg,
            )
            target_rows.append(targets)
        self._target_deg = torch.tensor(target_rows, dtype=torch.float32, device=self.device)

    def _apply_action(self) -> None:
        for env_index, drive_specs in enumerate(self._drive_specs_by_env):
            for spec, target in zip(drive_specs, self._target_deg[env_index].detach().cpu().tolist()):
                spec["target_attr"].Set(float(target))
                spec["current_target_deg"] = float(target)

    def _body_reference_state(self, env_index: int = 0) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        view = self._body_views_by_env[env_index]["body_reference"]
        position, orientation = view.get_world_pose()
        linear_velocity = view.get_linear_velocity()
        angular_velocity = view.get_angular_velocity()
        return (
            to_numpy(position).astype(np.float32).reshape(-1),
            to_numpy(orientation).astype(np.float32).reshape(-1),
            to_numpy(linear_velocity).astype(np.float32).reshape(-1),
            to_numpy(angular_velocity).astype(np.float32).reshape(-1),
        )

    def _rigid_body_pose(self, env_index: int, body_name: str) -> tuple[np.ndarray, np.ndarray]:
        position, orientation = self._body_views_by_env[env_index][body_name].get_world_pose()
        return (
            to_numpy(position).astype(np.float64).reshape(-1),
            to_numpy(orientation).astype(np.float64).reshape(-1),
        )

    def _body_world_endpoint(self, env_index: int, body_name: str, pivot: np.ndarray) -> np.ndarray:
        body = self._linkages[env_index]["bodies"][body_name]
        view = self._body_views_by_env[env_index][body_name]
        position, orientation = view.get_world_pose()
        local = np.array(local_endpoint(pivot, body["center"]), dtype=np.float64)
        return to_numpy(position).astype(np.float64).reshape(-1) + quat_wxyz_to_rotation_matrix(to_numpy(orientation)) @ local

    def _track_world_endpoint(self, env_index: int, track: dict[str, object]) -> np.ndarray:
        body_name = str(track["body"])
        dynamic_local = track.get("dynamic_local")
        if dynamic_local is None:
            return self._body_world_endpoint(env_index, body_name, np.asarray(track["pivot"], dtype=np.float64))
        view = self._body_views_by_env[env_index][body_name]
        position, orientation = view.get_world_pose()
        local = np.asarray(dynamic_local, dtype=np.float64).reshape(3)
        return to_numpy(position).astype(np.float64).reshape(-1) + quat_wxyz_to_rotation_matrix(to_numpy(orientation)) @ local

    def _foot_positions(self, env_index: int) -> np.ndarray:
        positions = [
            self._track_world_endpoint(env_index, track)
            for track in self._foot_tracks_by_env[env_index]
        ]
        return np.vstack(positions).astype(np.float32)

    def _actual_cad_visual_foot_positions(self, env_index: int, rendered: bool = True) -> np.ndarray:
        tracks = self._actual_cad_visual_foot_tracks_by_env[env_index]
        if not tracks:
            return np.zeros((0, 3), dtype=np.float32)
        positions = [
            self._track_world_endpoint(env_index, track)
            for track in tracks
        ]
        rows = np.vstack(positions).astype(np.float32)
        rows[:, 2] -= np.asarray(
            [float(track["radius_m"]) for track in tracks],
            dtype=np.float32,
        )
        if rendered:
            lift_m = float(self._linkages[env_index].get("actual_cad_visual_lift_m", 0.0) or 0.0)
            rows[:, 2] += lift_m
        return rows

    def foot_contact_alignment_report(
        self,
        tolerance_m: float = 0.001,
        penetration_tolerance_m: float = 0.0005,
    ) -> dict[str, object]:
        """Compare each rendered CAD foot bottom with its physics contact bottom."""
        env_rows = []
        max_error_m = 0.0
        min_ground_clearance_m = float("inf")
        for env_index in range(self.num_envs):
            support_centers = self._foot_positions(env_index).astype(np.float64)
            support_radii = np.asarray(
                [float(track["radius_m"]) for track in self._foot_tracks_by_env[env_index]],
                dtype=np.float64,
            )
            support_bottoms = support_centers.copy()
            support_bottoms[:, 2] -= support_radii
            terrain_heights = self._terrain_heights_np(support_centers, env_index=env_index).astype(np.float64)
            ground_clearances = support_bottoms[:, 2] - terrain_heights
            min_ground_clearance_m = min(
                min_ground_clearance_m,
                float(np.min(ground_clearances, initial=float("inf"))),
            )
            visual_bottoms = self._actual_cad_visual_foot_positions(env_index, rendered=True).astype(np.float64)
            if visual_bottoms.shape != support_bottoms.shape:
                raise RuntimeError(
                    "CAD foot/contact count mismatch: "
                    f"visual={visual_bottoms.shape}, support={support_bottoms.shape}"
                )
            errors = np.linalg.norm(visual_bottoms - support_bottoms, axis=1)
            max_error_m = max(max_error_m, float(np.max(errors, initial=0.0)))
            env_rows.append(
                {
                    "env_index": int(env_index),
                    "feet": [
                        {
                            "name": str(track["name"]),
                            "visual_bottom_m": self._rounded_vector(visual_bottoms[index]),
                            "collision_bottom_m": self._rounded_vector(support_bottoms[index]),
                            "alignment_error_m": round(float(errors[index]), 8),
                            "terrain_height_m": round(float(terrain_heights[index]), 8),
                            "ground_clearance_m": round(float(ground_clearances[index]), 8),
                            "radius_m": round(float(support_radii[index]), 8),
                        }
                        for index, track in enumerate(self._foot_tracks_by_env[env_index])
                    ],
                }
            )
        return {
            "passed": bool(
                max_error_m <= float(tolerance_m)
                and min_ground_clearance_m >= -float(penetration_tolerance_m)
            ),
            "tolerance_m": float(tolerance_m),
            "penetration_tolerance_m": float(penetration_tolerance_m),
            "max_alignment_error_m": round(max_error_m, 8),
            "min_ground_clearance_m": round(min_ground_clearance_m, 8),
            "envs": env_rows,
        }

    @staticmethod
    def _rounded_vector(values: np.ndarray) -> list[float]:
        return [round(float(value), 6) for value in np.asarray(values, dtype=np.float64).reshape(-1)]

    def leg_start_stance_report(self) -> dict[str, object]:
        """Report per-leg hip-to-foot geometry at the current pose."""
        env_rows = []
        max_visual_reach_spread = 0.0
        max_support_reach_spread = 0.0
        for env_index, linkage in enumerate(self._linkages):
            legs = list(linkage.get("legs", []))
            origin = self._env_origins_np[env_index].astype(np.float64)
            body_position, body_orientation, _, _ = self._body_reference_state(env_index)
            world_from_body = quat_wxyz_to_rotation_matrix(body_orientation)
            hip_points = np.vstack(
                [
                    self._body_world_endpoint(
                        env_index,
                        f"{leg['id']}_ground",
                        np.asarray(linkage["points"][f"{leg['id']}_hip_origin"], dtype=np.float64),
                    )
                    for leg in legs
                ]
            )
            local_hip_points = (
                (hip_points - body_position.reshape(1, 3)) @ world_from_body
                if len(hip_points)
                else np.zeros((0, 3), dtype=np.float64)
            )
            local_hip_center = local_hip_points.mean(axis=0) if len(local_hip_points) else np.zeros(3)
            support_positions = self._foot_positions(env_index).astype(np.float64)
            visual_positions = self._actual_cad_visual_foot_positions(env_index, rendered=True).astype(np.float64)
            rows = []
            support_reaches = []
            visual_reaches = []
            for leg_index, leg in enumerate(legs):
                leg_id = str(leg["id"])
                hip = hip_points[leg_index]
                local_hip = local_hip_points[leg_index]
                fore_aft = "front" if local_hip[0] >= local_hip_center[0] else "rear"
                side = "left" if local_hip[1] >= local_hip_center[1] else "right"

                def endpoint_metrics(endpoint: np.ndarray | None) -> dict[str, object] | None:
                    if endpoint is None:
                        return None
                    vector = np.asarray(endpoint, dtype=np.float64) - hip
                    planar_reach = float(np.linalg.norm(vector[:2]))
                    return {
                        "position_m": self._rounded_vector(endpoint),
                        "hip_to_foot_m": self._rounded_vector(vector),
                        "planar_reach_m": round(planar_reach, 6),
                        "x_reach_m": round(float(vector[0]), 6),
                        "y_reach_m": round(float(vector[1]), 6),
                        "z_drop_m": round(float(-vector[2]), 6),
                    }

                support_endpoint = support_positions[leg_index] if leg_index < len(support_positions) else None
                visual_endpoint = visual_positions[leg_index] if leg_index < len(visual_positions) else None
                support = endpoint_metrics(support_endpoint)
                visual = endpoint_metrics(visual_endpoint)
                if support is not None:
                    support_reaches.append(float(support["planar_reach_m"]))
                if visual is not None:
                    visual_reaches.append(float(visual["planar_reach_m"]))
                visual_minus_support = None
                if support_endpoint is not None and visual_endpoint is not None:
                    visual_minus_support = self._rounded_vector(visual_endpoint - support_endpoint)
                rows.append(
                    {
                        "leg_index": int(leg_index),
                        "leg_id": leg_id,
                        "pose_label": f"{fore_aft}_{side}",
                        "label_assumption": "x_positive_front_y_positive_left",
                        "hip_position_m": self._rounded_vector(hip),
                        "hip_position_local_m": self._rounded_vector(local_hip),
                        "support_endpoint": support,
                        "rendered_visual_endpoint": visual,
                        "rendered_visual_minus_support_m": visual_minus_support,
                    }
                )
            support_spread = max(support_reaches) - min(support_reaches) if support_reaches else 0.0
            visual_spread = max(visual_reaches) - min(visual_reaches) if visual_reaches else 0.0
            max_support_reach_spread = max(max_support_reach_spread, float(support_spread))
            max_visual_reach_spread = max(max_visual_reach_spread, float(visual_spread))
            env_rows.append(
                {
                    "env_index": int(env_index),
                    "support_planar_reach_spread_m": round(float(support_spread), 6),
                    "rendered_visual_planar_reach_spread_m": round(float(visual_spread), 6),
                    "legs": rows,
                }
            )
        return {
            "label_assumption": "x_positive_front_y_positive_left",
            "max_support_planar_reach_spread_m": round(float(max_support_reach_spread), 6),
            "max_rendered_visual_planar_reach_spread_m": round(float(max_visual_reach_spread), 6),
            "envs": env_rows,
        }

    def front_rear_kinematic_symmetry_report(self) -> dict[str, object]:
        """Compare each front mechanism with its rear-derived counterpart."""
        pair_specs = (
            ("right", "dom_p_4_1", "dom_p_21_1"),
            ("left", "dom_p_12_1", "dom_p_25_1"),
        )
        body_suffixes = (
            "ground",
            "lower_driver",
            "coupler",
            "lower_diagonal",
            "lower_closure",
            "upper_driver",
            "upper_closure",
        )
        env_rows = []
        global_max_position_error = 0.0
        global_max_rotation_error = 0.0
        for env_index in range(self.num_envs):
            pair_rows = []
            for side, front_id, rear_id in pair_specs:
                front_ground_position, front_ground_orientation = self._rigid_body_pose(
                    env_index, f"{front_id}_ground"
                )
                rear_ground_position, rear_ground_orientation = self._rigid_body_pose(
                    env_index, f"{rear_id}_ground"
                )
                world_from_front_ground = quat_wxyz_to_rotation_matrix(front_ground_orientation)
                world_from_rear_ground = quat_wxyz_to_rotation_matrix(rear_ground_orientation)
                body_rows = []
                pair_max_position_error = 0.0
                pair_max_rotation_error = 0.0
                for suffix in body_suffixes:
                    front_name = f"{front_id}_{suffix}"
                    rear_name = f"{rear_id}_{suffix}"
                    front_position, front_orientation = self._rigid_body_pose(env_index, front_name)
                    rear_position, rear_orientation = self._rigid_body_pose(env_index, rear_name)
                    front_local_position = world_from_front_ground.T @ (
                        front_position - front_ground_position
                    )
                    rear_local_position = world_from_rear_ground.T @ (
                        rear_position - rear_ground_position
                    )
                    front_local_rotation = (
                        world_from_front_ground.T
                        @ quat_wxyz_to_rotation_matrix(front_orientation)
                    )
                    rear_local_rotation = (
                        world_from_rear_ground.T
                        @ quat_wxyz_to_rotation_matrix(rear_orientation)
                    )
                    position_error = float(np.linalg.norm(front_local_position - rear_local_position))
                    rotation_error = rotation_delta_deg(front_local_rotation, rear_local_rotation)
                    pair_max_position_error = max(pair_max_position_error, position_error)
                    pair_max_rotation_error = max(pair_max_rotation_error, rotation_error)
                    body_rows.append(
                        {
                            "suffix": suffix,
                            "front_body": front_name,
                            "rear_body": rear_name,
                            "front_local_position_m": self._rounded_vector(front_local_position),
                            "rear_local_position_m": self._rounded_vector(rear_local_position),
                            "position_error_m": round(position_error, 6),
                            "rotation_error_deg": round(rotation_error, 6),
                        }
                    )
                global_max_position_error = max(global_max_position_error, pair_max_position_error)
                global_max_rotation_error = max(global_max_rotation_error, pair_max_rotation_error)
                pair_rows.append(
                    {
                        "side": side,
                        "front_leg_id": front_id,
                        "rear_leg_id": rear_id,
                        "max_body_position_error_m": round(pair_max_position_error, 6),
                        "max_body_rotation_error_deg": round(pair_max_rotation_error, 6),
                        "bodies": body_rows,
                    }
                )
            env_rows.append({"env_index": int(env_index), "pairs": pair_rows})
        return {
            "comparison_frame": "each_leg_hip_carriage",
            "max_body_position_error_m": round(global_max_position_error, 6),
            "max_body_rotation_error_deg": round(global_max_rotation_error, 6),
            "envs": env_rows,
        }

    def _reward_foot_positions_and_radii(self, env_index: int) -> tuple[np.ndarray, np.ndarray]:
        visual_positions = self._actual_cad_visual_foot_positions(env_index, rendered=True)
        if (
            bool(self.cfg.use_actual_cad_visual_foot_bottom_for_rewards)
            and visual_positions.shape[0] == EXPECTED_FOOT_COUNT
        ):
            return visual_positions, np.zeros(EXPECTED_FOOT_COUNT, dtype=np.float32)
        return self._foot_positions(env_index), np.asarray(
            [track.get("radius_m", self.cfg.foot_proxy_radius_m) for track in self._foot_tracks_by_env[env_index]],
            dtype=np.float32,
        )

    def _reward_foot_positions(self, env_index: int) -> np.ndarray:
        return self._reward_foot_positions_and_radii(env_index)[0]

    def _joint_separation_rows(self, env_index: int) -> list[dict[str, object]]:
        rows = []
        linkage = self._linkages[env_index]
        leg_ids = [str(leg["id"]) for leg in linkage.get("legs", [])]
        for check in linkage.get("joint_checks", []):
            pivot = np.asarray(check["pivot"], dtype=np.float64)
            body_a = str(check["body_a"])
            body_b = str(check["body_b"])
            leg_id = next(
                (candidate for candidate in leg_ids if body_a.startswith(f"{candidate}_")),
                "",
            )
            point_a = self._body_world_endpoint(env_index, body_a, pivot)
            point_b = self._body_world_endpoint(env_index, body_b, pivot)
            separation = float(np.linalg.norm(point_a - point_b))
            rows.append(
                {
                    "env_index": int(env_index),
                    "leg_id": leg_id,
                    "name": str(check["name"]),
                    "role": str(check["role"]),
                    "body_a": body_a,
                    "body_b": body_b,
                    "separation_m": separation,
                }
            )
        return rows

    def joint_separation_report(self, worst_count: int = 12) -> dict[str, object]:
        rows = []
        for env_index in range(self.num_envs):
            rows.extend(self._joint_separation_rows(env_index))
        rows.sort(key=lambda row: float(row["separation_m"]), reverse=True)
        role_max: dict[str, float] = {}
        for row in rows:
            role = str(row["role"])
            role_max[role] = max(role_max.get(role, 0.0), float(row["separation_m"]))
        return {
            "joint_count": len(rows),
            "max_separation_m": round(float(rows[0]["separation_m"]), 6) if rows else 0.0,
            "max_by_role_m": {
                role: round(value, 6)
                for role, value in sorted(role_max.items())
            },
            "worst": [
                {
                    "env_index": int(row["env_index"]),
                    "leg_id": row["leg_id"],
                    "name": row["name"],
                    "role": row["role"],
                    "body_a": row["body_a"],
                    "body_b": row["body_b"],
                    "separation_m": round(float(row["separation_m"]), 6),
                }
                for row in rows[: max(int(worst_count), 0)]
            ],
        }

    def _terrain_heights_np(self, world_positions: np.ndarray, env_index: int | None = None) -> np.ndarray:
        positions = np.asarray(world_positions, dtype=np.float32).reshape(-1, 3)
        heights = np.zeros(len(positions), dtype=np.float32)
        if str(self.cfg.terrain_type).lower() != "stairs":
            return heights
        origins = (
            self._env_origins_np[[int(env_index)]]
            if env_index is not None
            else self._env_origins_np
        )
        step_depth = max(float(self.cfg.stairs_step_depth_m), 1e-6)
        step_height = float(self.cfg.stairs_step_height_m)
        step_count = int(self.cfg.stairs_step_count)
        start_x = float(self.cfg.stairs_start_x_m)
        half_width = 0.5 * float(self.cfg.stairs_width_m)
        top_start = start_x + (float(step_count) * step_depth)
        top_end = top_start + float(self.cfg.stairs_top_platform_length_m)
        for point_index, position in enumerate(positions):
            max_height = 0.0
            for origin in origins:
                local_x = float(position[0] - origin[0])
                local_y = float(position[1] - origin[1])
                if abs(local_y) > half_width or local_x < start_x:
                    continue
                if local_x < top_start:
                    step_index = int((local_x - start_x) // step_depth)
                    step_index = max(0, min(step_count - 1, step_index))
                    max_height = max(max_height, float(step_index + 1) * step_height)
                elif local_x <= top_end:
                    max_height = max(max_height, float(step_count) * step_height)
            heights[point_index] = max_height
        return heights

    def _foot_ground_clearance_np(
        self,
        foot_positions: np.ndarray,
        env_index: int | None = None,
        radii_m: np.ndarray | None = None,
    ) -> np.ndarray:
        terrain_heights = self._terrain_heights_np(foot_positions, env_index=env_index)
        if radii_m is not None:
            radii = np.asarray(radii_m, dtype=np.float32).reshape(-1)
        elif env_index is not None:
            radii = np.asarray(
                [track.get("radius_m", self.cfg.foot_proxy_radius_m) for track in self._foot_tracks_by_env[int(env_index)]],
                dtype=np.float32,
            )
        else:
            radii = np.full(len(terrain_heights), float(self.cfg.foot_proxy_radius_m), dtype=np.float32)
        return foot_positions[:, 2] - (
            terrain_heights + radii + float(self.cfg.foot_contact_epsilon_m)
        )

    def _foot_contact_flags_np(
        self,
        foot_positions: np.ndarray,
        env_index: int | None = None,
        radii_m: np.ndarray | None = None,
    ) -> np.ndarray:
        return (self._foot_ground_clearance_np(foot_positions, env_index=env_index, radii_m=radii_m) <= 0.0).astype(np.float32)

    def _reward_foot_contact_flags_np(self, env_index: int) -> np.ndarray:
        foot_positions, radii = self._reward_foot_positions_and_radii(env_index)
        return self._foot_contact_flags_np(foot_positions, env_index=env_index, radii_m=radii)

    def _reward_foot_ground_clearance_np(self, env_index: int) -> np.ndarray:
        foot_positions, radii = self._reward_foot_positions_and_radii(env_index)
        return self._foot_ground_clearance_np(foot_positions, env_index=env_index, radii_m=radii)

    def _gait_phase_np(self) -> np.ndarray:
        if hasattr(self, "episode_length_buf"):
            episode_steps = to_numpy(self.episode_length_buf).astype(np.float32).reshape(-1)
        else:
            episode_steps = np.zeros(self.num_envs, dtype=np.float32)
        phase = (
            2.0 * math.pi * float(self.cfg.gait_frequency_hz) * episode_steps * float(self.step_dt)
            + self._gait_phase_offsets_rad_np
        )
        return np.stack([np.sin(phase), np.cos(phase)], axis=1).astype(np.float32)

    def _gait_phase_angle_np(self) -> np.ndarray:
        if hasattr(self, "episode_length_buf"):
            episode_steps = to_numpy(self.episode_length_buf).astype(np.float32).reshape(-1)
        else:
            episode_steps = np.zeros(self.num_envs, dtype=np.float32)
        return (
            2.0 * math.pi * float(self.cfg.gait_frequency_hz) * episode_steps * float(self.step_dt)
            + self._gait_phase_offsets_rad_np
        ).astype(np.float32)

    def _desired_stance_np(self, env_index: int, command: np.ndarray) -> np.ndarray:
        candidate = self._reference_gait_candidate()
        if is_keyframe_sequence(candidate):
            step = int(to_numpy(self.episode_length_buf[env_index]).reshape(-1)[0])
            return reference_desired_stance_for_steps(candidate, np.asarray([step], dtype=np.int64))[0]
        planar_command = float(np.linalg.norm(command[:2]))
        yaw_command = abs(float(command[2]))
        if planar_command + yaw_command < float(self.cfg.locomotion_command_threshold):
            return np.ones(EXPECTED_FOOT_COUNT, dtype=np.float32)
        phase = float(self._gait_phase_angle_np()[env_index])
        candidate = self._reference_gait_candidate()
        if not is_keyframe_sequence(candidate):
            phase *= float(candidate.get("frequency_scale", 1.0))
            phase_offsets = np.asarray(
                [
                    float(candidate.get(f"leg_phase_{leg_index}", DIAGONAL_TROT_PHASE_OFFSETS_RAD[leg_index]))
                    for leg_index in range(EXPECTED_FOOT_COUNT)
                ],
                dtype=np.float32,
            )
        else:
            phase_offsets = DIAGONAL_TROT_PHASE_OFFSETS_RAD
        return (np.cos(phase + phase_offsets) > 0.0).astype(np.float32)

    def _reference_gait_candidate(self) -> dict[str, object]:
        if str(self.cfg.reference_sequence_json).strip():
            return json.loads(str(self.cfg.reference_sequence_json))
        candidate = {"name": str(self.cfg.reference_gait_name or "reference_gait")}
        for key in REFERENCE_GAIT_PARAMETER_NAMES:
            candidate[key] = float(getattr(self.cfg, f"reference_gait_{key}"))
        return candidate

    def _reference_actions_np(self) -> np.ndarray:
        candidate = self._reference_gait_candidate()
        if is_keyframe_sequence(candidate):
            episode_steps = to_numpy(self.episode_length_buf).astype(np.int64).reshape(-1)
            return reference_actions_for_steps(candidate, episode_steps)
        return reference_actions_for_base_phases(candidate, self._gait_phase_angle_np())

    def last_reward_terms_report(self) -> dict[str, object]:
        def mean_dict(terms: dict[str, torch.Tensor]) -> dict[str, float]:
            return {
                name: round(float(torch.mean(value.detach()).cpu()), 6)
                for name, value in sorted(terms.items())
            }

        scaled_mean = mean_dict(self._last_reward_terms_scaled)
        dominant_scaled = [
            {"name": name, "mean": value}
            for name, value in sorted(scaled_mean.items(), key=lambda item: abs(item[1]), reverse=True)
            if abs(value) > 1e-6
        ]
        return {
            "unscaled_mean": mean_dict(self._last_reward_terms_unscaled),
            "scaled_mean": scaled_mean,
            "dominant_scaled_mean": dominant_scaled[:8],
            "total_mean": round(float(torch.mean(self._last_reward_terms_total.detach()).cpu()), 6),
        }

    def _get_observations(self) -> dict:
        self._apply_actual_cad_visual_lift()
        obs_rows = []
        actions_np = self._actions.detach().cpu().numpy().astype(np.float32)
        commands_np = self._commands.detach().cpu().numpy().astype(np.float32)
        gait_phase_np = self._gait_phase_np()
        reference_actions_np = None
        if bool(self.cfg.include_reference_actions_in_observation):
            reference_actions_np = self._reference_actions_np()
        for env_index, drive_specs in enumerate(self._drive_specs_by_env):
            position, orientation, linear_velocity, angular_velocity = self._body_reference_state(env_index)
            local_position = position - self._env_origins_np[env_index]
            projected_gravity = projected_gravity_from_quat(orientation)
            reward_foot_positions, reward_foot_radii = self._reward_foot_positions_and_radii(env_index)
            foot_positions = reward_foot_positions - self._env_origins_np[env_index]
            foot_contacts = self._foot_contact_flags_np(
                reward_foot_positions,
                env_index,
                radii_m=reward_foot_radii,
            )
            normalized_targets = []
            for spec in drive_specs:
                lower_deg, upper_deg = spec["target_limit_deg"]
                span = max(float(upper_deg) - float(lower_deg), 1e-6)
                # Actions are expressed in the CAD command convention. Mirror the
                # PhysX target back into that convention before exposing it to the
                # policy, otherwise negative-axis legs report the opposite sign.
                target_sign = float(spec.get("target_sign", 1.0))
                command_target_deg = target_sign * float(spec["current_target_deg"])
                command_center_deg = float(spec.get("command_center_deg", spec["center_deg"]))
                normalized_targets.append((command_target_deg - command_center_deg) / span)
            obs_parts = [
                local_position,
                orientation,
                linear_velocity,
                angular_velocity,
                projected_gravity,
                commands_np[env_index],
                gait_phase_np[env_index],
                foot_positions.reshape(-1),
                foot_contacts,
                actions_np[env_index],
                np.asarray(normalized_targets, dtype=np.float32),
            ]
            if reference_actions_np is not None:
                obs_parts.append(reference_actions_np[env_index])
            obs_np = np.concatenate(obs_parts)
            if obs_np.shape[0] != int(self.cfg.observation_space):
                raise RuntimeError(f"Expected {self.cfg.observation_space} observations, found {obs_np.shape[0]}.")
            obs_rows.append(obs_np)
        obs = torch.tensor(np.vstack(obs_rows), dtype=torch.float32, device=self.device)
        if float(self.cfg.visible_step_delay_s) > 0.0:
            time.sleep(float(self.cfg.visible_step_delay_s))
        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        height_errors = []
        flat_orientation_terms = []
        vertical_velocity_terms = []
        angular_velocity_terms = []
        command_velocity_terms = []
        command_velocity_tracking_terms = []
        command_progress_terms = []
        command_stagnation_terms = []
        lateral_drift_terms = []
        yaw_drift_terms = []
        command_yaw_terms = []
        command_yaw_tracking_terms = []
        foot_contact_terms = []
        gait_contact_terms = []
        stance_contact_terms = []
        swing_contact_terms = []
        foot_clearance_terms = []
        reference_action_tracking_terms = []
        reference_action_mse_terms = []
        actions_np = self._actions.detach().cpu().numpy().astype(np.float32)
        commands_np = self._commands.detach().cpu().numpy().astype(np.float32)
        reference_actions_np = None
        previous_positions_np = self._previous_body_reference_positions.detach().cpu().numpy().astype(np.float32)
        if hasattr(self, "episode_length_buf"):
            episode_steps_np = to_numpy(self.episode_length_buf).astype(np.float32).reshape(-1)
        else:
            episode_steps_np = np.zeros(self.num_envs, dtype=np.float32)
        current_positions = []
        if (
            abs(float(self.cfg.reference_action_tracking_reward_scale)) > 0.0
            or abs(float(self.cfg.reference_action_mse_reward_scale)) > 0.0
        ):
            reference_actions_np = self._reference_actions_np()
        for env_index in range(self.num_envs):
            position, orientation, linear_velocity, angular_velocity = self._body_reference_state(env_index)
            current_positions.append(position)
            command = commands_np[env_index]
            height_errors.append(float(position[2] - self._env_origins_np[env_index][2]) - float(self.cfg.target_height_m))
            projected_gravity = projected_gravity_from_quat(orientation)
            flat_orientation_terms.append(float(np.sum(np.square(projected_gravity[:2]))))
            vertical_velocity_terms.append(float(linear_velocity[2] * linear_velocity[2]))
            angular_velocity_terms.append(float(np.sum(np.square(angular_velocity))))
            displacement_velocity = (position - previous_positions_np[env_index]) / max(float(self.step_dt), 1e-6)
            reward_velocity = displacement_velocity if bool(self.cfg.use_displacement_velocity_rewards) else linear_velocity
            command_motion = command_motion_terms(
                reward_velocity[:2],
                command[:2],
                tracking_sigma_m_s=float(self.cfg.command_velocity_tracking_sigma),
                stagnation_speed_m_s=float(self.cfg.command_stagnation_speed_m_s),
            )
            if int(self._reset_settle_steps_remaining[env_index].item()) > 0:
                command_motion = {
                    "velocity_error_sq": 0.0,
                    "velocity_tracking": 0.0,
                    "directional_progress_m_s": 0.0,
                    "stagnation": 0.0,
                }
            command_velocity_terms.append(command_motion["velocity_error_sq"])
            command_velocity_tracking_terms.append(command_motion["velocity_tracking"])
            command_progress_terms.append(command_motion["directional_progress_m_s"])
            command_stagnation_terms.append(command_motion["stagnation"])
            initial_body_position = self._initial_body_states[env_index]["body_reference"]["position"].reshape(-1)
            initial_body_orientation = self._initial_body_states[env_index]["body_reference"]["orientation"].reshape(-1)
            lateral_drift_terms.append(float((position[1] - initial_body_position[1]) ** 2))
            current_yaw = yaw_from_quat_wxyz(orientation)
            initial_yaw = yaw_from_quat_wxyz(initial_body_orientation)
            target_yaw = initial_yaw + float(command[2]) * float(episode_steps_np[env_index]) * float(self.step_dt)
            yaw_drift_terms.append(float(wrap_angle_rad(current_yaw - target_yaw) ** 2))
            yaw_error = float((angular_velocity[2] - command[2]) ** 2)
            command_yaw_terms.append(yaw_error)
            sigma_yaw = max(float(self.cfg.command_yaw_tracking_sigma), 1e-6)
            command_yaw_tracking_terms.append(float(math.exp(-yaw_error / (sigma_yaw * sigma_yaw))))
            foot_positions, foot_radii = self._reward_foot_positions_and_radii(env_index)
            foot_contacts_np = self._foot_contact_flags_np(foot_positions, env_index, radii_m=foot_radii)
            foot_contact_terms.append(float(np.mean(foot_contacts_np)))
            desired_stance = self._desired_stance_np(env_index, command)
            gait_contact_terms.append(float(np.mean(1.0 - np.abs(foot_contacts_np - desired_stance))))
            desired_swing = 1.0 - desired_stance
            stance_count = max(float(np.sum(desired_stance)), 1.0)
            swing_count = max(float(np.sum(desired_swing)), 1.0)
            stance_contact_terms.append(float(np.sum(foot_contacts_np * desired_stance) / stance_count))
            swing_contact_terms.append(float(np.sum(foot_contacts_np * desired_swing) / swing_count))
            if float(np.sum(desired_swing)) > 0.0:
                ground_clearance = np.maximum(
                    0.0,
                    self._foot_ground_clearance_np(foot_positions, env_index, radii_m=foot_radii),
                )
                normalized_clearance = np.clip(ground_clearance / max(float(self.cfg.target_swing_clearance_m), 1e-6), 0.0, 1.0)
                foot_clearance_terms.append(float(np.sum(normalized_clearance * desired_swing) / np.sum(desired_swing)))
            else:
                foot_clearance_terms.append(0.0)
            if reference_actions_np is not None:
                reference_error = float(np.mean(np.square(actions_np[env_index] - reference_actions_np[env_index])))
                sigma_action = max(float(self.cfg.reference_action_tracking_sigma), 1e-6)
                reference_action_tracking_terms.append(float(math.exp(-reference_error / (sigma_action * sigma_action))))
                reference_action_mse_terms.append(reference_error)
            else:
                reference_action_tracking_terms.append(0.0)
                reference_action_mse_terms.append(0.0)
        height_error = torch.tensor(height_errors, dtype=torch.float32, device=self.device)
        flat_orientation = torch.tensor(flat_orientation_terms, dtype=torch.float32, device=self.device)
        vertical_velocity = torch.tensor(vertical_velocity_terms, dtype=torch.float32, device=self.device)
        angular_velocity = torch.tensor(angular_velocity_terms, dtype=torch.float32, device=self.device)
        command_velocity = torch.tensor(command_velocity_terms, dtype=torch.float32, device=self.device)
        command_velocity_tracking = torch.tensor(command_velocity_tracking_terms, dtype=torch.float32, device=self.device)
        command_progress = torch.tensor(command_progress_terms, dtype=torch.float32, device=self.device)
        command_stagnation = torch.tensor(command_stagnation_terms, dtype=torch.float32, device=self.device)
        lateral_drift = torch.tensor(lateral_drift_terms, dtype=torch.float32, device=self.device)
        yaw_drift = torch.tensor(yaw_drift_terms, dtype=torch.float32, device=self.device)
        command_yaw = torch.tensor(command_yaw_terms, dtype=torch.float32, device=self.device)
        command_yaw_tracking = torch.tensor(command_yaw_tracking_terms, dtype=torch.float32, device=self.device)
        foot_contact = torch.tensor(foot_contact_terms, dtype=torch.float32, device=self.device)
        gait_contact = torch.tensor(gait_contact_terms, dtype=torch.float32, device=self.device)
        stance_contact = torch.tensor(stance_contact_terms, dtype=torch.float32, device=self.device)
        swing_contact = torch.tensor(swing_contact_terms, dtype=torch.float32, device=self.device)
        foot_clearance = torch.tensor(foot_clearance_terms, dtype=torch.float32, device=self.device)
        reference_action_tracking = torch.tensor(reference_action_tracking_terms, dtype=torch.float32, device=self.device)
        reference_action_mse = torch.tensor(reference_action_mse_terms, dtype=torch.float32, device=self.device)
        action_penalty = torch.sum(torch.square(self._actions), dim=1)
        action_rate_penalty = torch.sum(torch.square(self._actions - self._previous_actions), dim=1)
        reward_terms = {
            "alive": torch.ones(self.num_envs, dtype=torch.float32, device=self.device),
            "height_error_sq": torch.square(height_error),
            "flat_orientation": flat_orientation,
            "vertical_velocity_sq": vertical_velocity,
            "angular_velocity_sq": angular_velocity,
            "command_velocity_error_sq": command_velocity,
            "command_velocity_tracking": command_velocity_tracking,
            "command_progress": command_progress,
            "command_stagnation": command_stagnation,
            "lateral_drift_sq": lateral_drift,
            "yaw_drift_sq": yaw_drift,
            "command_yaw_error_sq": command_yaw,
            "command_yaw_tracking": command_yaw_tracking,
            "action_sq": action_penalty,
            "action_rate_sq": action_rate_penalty,
            "foot_contact": foot_contact,
            "gait_contact_match": gait_contact,
            "stance_contact": stance_contact,
            "swing_contact": swing_contact,
            "foot_clearance": foot_clearance,
            "reference_action_tracking": reference_action_tracking,
            "reference_action_mse": reference_action_mse,
        }
        reward_scales = {
            "alive": float(self.cfg.alive_reward_scale),
            "height_error_sq": float(self.cfg.height_reward_scale),
            "flat_orientation": float(self.cfg.flat_orientation_reward_scale),
            "vertical_velocity_sq": float(self.cfg.vertical_velocity_reward_scale),
            "angular_velocity_sq": float(self.cfg.angular_velocity_reward_scale),
            "command_velocity_error_sq": float(self.cfg.command_velocity_reward_scale),
            "command_velocity_tracking": float(self.cfg.command_velocity_tracking_reward_scale),
            "command_progress": float(self.cfg.command_progress_reward_scale),
            "command_stagnation": float(self.cfg.command_stagnation_penalty_scale),
            "lateral_drift_sq": float(self.cfg.lateral_drift_reward_scale),
            "yaw_drift_sq": float(self.cfg.yaw_drift_reward_scale),
            "command_yaw_error_sq": float(self.cfg.command_yaw_reward_scale),
            "command_yaw_tracking": float(self.cfg.command_yaw_tracking_reward_scale),
            "action_sq": float(self.cfg.action_reward_scale),
            "action_rate_sq": float(self.cfg.action_rate_reward_scale),
            "foot_contact": float(self.cfg.foot_contact_reward_scale),
            "gait_contact_match": float(self.cfg.gait_contact_reward_scale),
            "stance_contact": float(self.cfg.stance_contact_reward_scale),
            "swing_contact": float(self.cfg.swing_contact_penalty_scale),
            "foot_clearance": float(self.cfg.foot_clearance_reward_scale),
            "reference_action_tracking": float(self.cfg.reference_action_tracking_reward_scale),
            "reference_action_mse": float(self.cfg.reference_action_mse_reward_scale),
        }
        scaled_reward_terms = {
            name: reward_scales[name] * value
            for name, value in reward_terms.items()
        }
        reward = torch.zeros(self.num_envs, dtype=torch.float32, device=self.device)
        for value in scaled_reward_terms.values():
            reward = reward + value
        self._last_reward_terms_unscaled = reward_terms
        self._last_reward_terms_scaled = scaled_reward_terms
        self._last_reward_terms_total = reward
        self._previous_body_reference_positions = torch.tensor(
            np.vstack(current_positions), dtype=torch.float32, device=self.device
        )
        return reward

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        died_values = []
        below_height_values = []
        above_tilt_values = []
        non_foot_contact_values = []
        joint_separation_values = []
        max_joint_separation_values = []
        done_diagnostics = []
        for env_index in range(self.num_envs):
            position, orientation, _, _ = self._body_reference_state(env_index)
            projected_gravity = projected_gravity_from_quat(orientation)
            tilt = math.acos(max(-1.0, min(1.0, -float(projected_gravity[2]))))
            body_height = float(position[2] - self._env_origins_np[env_index][2])
            below_height = body_height < float(self.cfg.min_height_m)
            above_tilt = tilt > self._max_tilt_rad
            non_foot_contacts = self._non_foot_ground_contact_rows(env_index)
            non_foot_ground_contact = bool(
                self.cfg.terminate_on_non_foot_ground_contact
                and any(bool(row["contact"]) for row in non_foot_contacts)
            )
            joint_separation_rows = self._joint_separation_rows(env_index)
            max_joint_separation_m = max(
                (float(row["separation_m"]) for row in joint_separation_rows),
                default=0.0,
            )
            joint_separation = bool(
                self.cfg.terminate_on_joint_separation
                and max_joint_separation_m > float(self.cfg.max_joint_separation_m)
            )
            below_height_values.append(bool(below_height))
            above_tilt_values.append(bool(above_tilt))
            non_foot_contact_values.append(non_foot_ground_contact)
            joint_separation_values.append(joint_separation)
            max_joint_separation_values.append(max_joint_separation_m)
            died_values.append(
                below_height
                or above_tilt
                or non_foot_ground_contact
                or joint_separation
            )
            done_diagnostics.append(
                {
                    "env_index": env_index,
                    "body_height_m": body_height,
                    "tilt_deg": math.degrees(tilt),
                    "below_min_height": bool(below_height),
                    "above_max_tilt": bool(above_tilt),
                    "non_foot_ground_contact": non_foot_ground_contact,
                    "joint_separation": joint_separation,
                    "max_joint_separation_m": round(max_joint_separation_m, 6),
                    "joint_separation_limit_m": float(self.cfg.max_joint_separation_m),
                    "separated_joints": [
                        {
                            "leg_id": str(row["leg_id"]),
                            "name": str(row["name"]),
                            "role": str(row["role"]),
                            "separation_m": round(float(row["separation_m"]), 6),
                        }
                        for row in joint_separation_rows
                        if float(row["separation_m"]) > float(self.cfg.max_joint_separation_m)
                    ],
                    "non_foot_min_ground_clearance_m": round(
                        min(
                            (float(row["ground_clearance_m"]) for row in non_foot_contacts),
                            default=float("inf"),
                        ),
                        6,
                    ),
                    "non_foot_contact_bodies": [
                        str(row["body"])
                        for row in non_foot_contacts
                        if bool(row["contact"])
                    ],
                }
            )
        self._last_done_diagnostics = done_diagnostics
        died = torch.tensor(died_values, dtype=torch.bool, device=self.device)
        log_terms = {
            "Termination/below_height_rate": torch.tensor(
                below_height_values,
                dtype=torch.float32,
                device=self.device,
            ),
            "Termination/above_tilt_rate": torch.tensor(
                above_tilt_values,
                dtype=torch.float32,
                device=self.device,
            ),
            "Termination/non_foot_ground_contact_rate": torch.tensor(
                non_foot_contact_values,
                dtype=torch.float32,
                device=self.device,
            ),
            "Termination/joint_separation_rate": torch.tensor(
                joint_separation_values,
                dtype=torch.float32,
                device=self.device,
            ),
            "Mechanism/max_joint_separation_m": torch.tensor(
                max_joint_separation_values,
                dtype=torch.float32,
                device=self.device,
            ),
            "Termination/timeout_rate": time_out.to(dtype=torch.float32),
        }
        for name, value in self._last_reward_terms_scaled.items():
            log_terms[f"Reward/{name}"] = value.detach()
        log_terms["Reward/total"] = self._last_reward_terms_total.detach()
        log_terms["Motion/action_rms_normalized"] = torch.sqrt(
            torch.mean(torch.square(self._actions), dim=1)
        )
        log_terms["Motion/action_max_abs_normalized"] = torch.max(
            torch.abs(self._actions),
            dim=1,
        ).values
        log_terms["Motion/target_offset_rms_deg"] = torch.sqrt(
            torch.mean(torch.square(self._target_deg - self._center_deg), dim=1)
        )
        for env_index in range(self.num_envs):
            log_terms[f"Termination/non_foot_env_{env_index}"] = torch.tensor(
                float(non_foot_contact_values[env_index]),
                dtype=torch.float32,
                device=self.device,
            )
        protected_body_names = sorted(
            {
                str(body_name)
                for linkage in self._linkages
                for body_name, body in linkage["bodies"].items()
                if body.get("collision_half_extents_m") is not None
            }
        )
        for body_name in protected_body_names:
            log_terms[f"Termination/non_foot_body/{body_name}"] = torch.tensor(
                [
                    float(body_name in diagnostic["non_foot_contact_bodies"])
                    for diagnostic in done_diagnostics
                ],
                dtype=torch.float32,
                device=self.device,
            )
        self.extras["log"] = log_terms
        return died, time_out

    def _non_foot_ground_contact_rows(self, env_index: int) -> list[dict[str, object]]:
        rows = []
        linkage = self._linkages[env_index]
        margin_m = float(self.cfg.non_foot_ground_contact_margin_m)
        for body_name, body in linkage["bodies"].items():
            # Distal lower-leg members remain physical colliders, but ground
            # contact there is part of normal foot/ankle operation rather than
            # a chassis or upper-linkage fall.
            if "_lower_" in str(body_name):
                continue
            half_extents = body.get("collision_half_extents_m")
            if half_extents is None:
                continue
            position, orientation = self._rigid_body_pose(env_index, body_name)
            rotation = quat_wxyz_to_rotation_matrix(orientation)
            half_extents_np = np.asarray(half_extents, dtype=np.float64)
            local_center = np.asarray(
                body.get("collision_local_center_m", (0.0, 0.0, 0.0)),
                dtype=np.float64,
            )
            collision_center = position + rotation @ local_center
            projected_half_height_m = float(np.dot(np.abs(rotation[2, :]), half_extents_np))
            lowest_z_m = float(collision_center[2] - projected_half_height_m)
            terrain_height_m = float(
                self._terrain_heights_np(collision_center.reshape(1, 3), env_index=env_index)[0]
            )
            ground_clearance_m = lowest_z_m - terrain_height_m
            rows.append(
                {
                    "env_index": int(env_index),
                    "body": str(body_name),
                    "lowest_z_m": lowest_z_m,
                    "terrain_height_m": terrain_height_m,
                    "ground_clearance_m": ground_clearance_m,
                    "contact": bool(ground_clearance_m <= margin_m),
                    "collision_path": str(body.get("ground_collision_path", "")),
                    "collision_fit_source": str(body.get("ground_collision_fit_source", "")),
                }
            )
        return rows

    def non_foot_ground_contact_report(self) -> dict[str, object]:
        rows = [
            row
            for env_index in range(self.num_envs)
            for row in self._non_foot_ground_contact_rows(env_index)
        ]
        contacts = [row for row in rows if bool(row["contact"])]
        return {
            "enabled": bool(self.cfg.enable_body_collisions),
            "terminate_on_contact": bool(self.cfg.terminate_on_non_foot_ground_contact),
            "contact_margin_m": float(self.cfg.non_foot_ground_contact_margin_m),
            "protected_body_count": len(rows),
            "contact_count": len(contacts),
            "min_ground_clearance_m": round(
                min((float(row["ground_clearance_m"]) for row in rows), default=float("inf")),
                6,
            ),
            "contacts": [
                {
                    **row,
                    "lowest_z_m": round(float(row["lowest_z_m"]), 6),
                    "terrain_height_m": round(float(row["terrain_height_m"]), 6),
                    "ground_clearance_m": round(float(row["ground_clearance_m"]), 6),
                }
                for row in contacts
            ],
        }

    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, dtype=torch.int64, device=self.device)
        super()._reset_idx(env_ids)
        if not hasattr(self, "_body_views"):
            return
        env_id_list = self._env_id_list(env_ids)
        self._actions = self._clone_if_inference_tensor(self._actions)
        self._previous_actions = self._clone_if_inference_tensor(self._previous_actions)
        self._target_deg = self._clone_if_inference_tensor(self._target_deg)
        self._previous_body_reference_positions = self._clone_if_inference_tensor(self._previous_body_reference_positions)
        self._reset_settle_steps_remaining = self._clone_if_inference_tensor(
            self._reset_settle_steps_remaining
        )
        self._actions[env_id_list] = 0.0
        self._previous_actions[env_id_list] = 0.0
        self._reset_settle_steps_remaining[env_id_list] = max(
            int(self.cfg.reset_settle_steps),
            0,
        )
        for env_index in env_id_list:
            self._target_deg[env_index] = torch.tensor(
                [float(spec["center_deg"]) for spec in self._drive_specs_by_env[env_index]],
                dtype=torch.float32,
                device=self.device,
            )
        self._reset_rigid_bodies_to_initial(env_id_list)
        for env_index in env_id_list:
            self._previous_body_reference_positions[env_index] = torch.tensor(
                self._body_reference_state(env_index)[0],
                dtype=torch.float32,
                device=self.device,
            )
