"""Domino velocity + ride-height locomotion task (Isaac Lab manager-based).

Actor observations are restricted to what the ESP32 really has at 50 Hz:
MPU6050 gyro and gravity direction (with history), the remote's commands,
its own previous foot commands and the gait clock. The hobby servos give no
position feedback, so joint angles and body velocity are critic-only
(asymmetric actor-critic).
"""

from __future__ import annotations

import math

import isaaclab.sim as sim_utils
import isaaclab.terrains as terrain_gen
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import UniformNoiseCfg as Unoise  # additive by default

try:  # Isaac Lab 3.x
    import isaaclab_tasks.core.velocity.mdp as vel_mdp
except ImportError:  # Isaac Lab 2.x
    import isaaclab_tasks.manager_based.locomotion.velocity.mdp as vel_mdp

from . import config as domino_config
from . import geometry as g
from . import mdp
from .assets import domino_cfg

CFG = domino_config.load()
# Free-gait policies (gait.clock_obs false) get no gait clock; the policy finds its
# own rhythm. Older configs without the key keep the clock.
GAIT_CLOCK = bool(CFG["gait"].get("clock_obs", True))
FEET = SceneEntityCfg("contact_forces", body_names=mdp.FOOT_BODIES, preserve_order=True)
ACTOR_HISTORY = 3


def terrain_generator(t: dict) -> terrain_gen.TerrainGeneratorCfg:
    sub = {"flat": terrain_gen.MeshPlaneTerrainCfg(proportion=0.25 if t["rough"] else 1.0)}
    if t["rough"]:
        sub["rough"] = terrain_gen.HfRandomUniformTerrainCfg(
            proportion=0.35, noise_range=(0.0, t["max_noise_m"]), noise_step=0.005, border_width=0.25
        )
        sub["slope_up"] = terrain_gen.HfPyramidSlopedTerrainCfg(
            proportion=0.2, slope_range=(0.0, t["max_slope"]), platform_width=1.0, border_width=0.25
        )
        sub["slope_down"] = terrain_gen.HfInvertedPyramidSlopedTerrainCfg(
            proportion=0.2, slope_range=(0.0, t["max_slope"]), platform_width=1.0, border_width=0.25
        )
    return terrain_gen.TerrainGeneratorCfg(
        size=(5.0, 5.0),
        border_width=10.0,
        num_rows=8,
        num_cols=16,
        horizontal_scale=0.05,
        vertical_scale=0.005,
        slope_threshold=0.75,
        use_cache=False,
        sub_terrains=sub,
    )


@configclass
class DominoSceneCfg(InteractiveSceneCfg):
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="generator",
        terrain_generator=terrain_generator(CFG["terrain"]),
        max_init_terrain_level=2,
        collision_group=-1,
        # The TPE foot's softness lives on the ground side of the contact:
        # Isaac Lab's per-robot friction randomisation replaces the robot's
        # shape materials and drops their compliance, while the ground
        # material is never randomised. Friction stays 1.0 (multiply), so the
        # robot side sets it.
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            compliant_contact_stiffness=CFG["robot"]["foot_contact_stiffness_n_per_m"],
            compliant_contact_damping=CFG["robot"]["foot_contact_damping_n_s_per_m"],
        ),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.55, 0.56, 0.58)),
        debug_vis=False,
    )
    robot: ArticulationCfg = domino_cfg(CFG["robot"]).replace(prim_path="{ENV_REGEX_NS}/Robot")
    contact_forces = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/.*", history_length=3, track_air_time=True)
    light = AssetBaseCfg(
        prim_path="/World/light", spawn=sim_utils.DistantLightCfg(color=(0.9, 0.9, 0.9), intensity=2500.0)
    )
    sky = AssetBaseCfg(prim_path="/World/sky", spawn=sim_utils.DomeLightCfg(color=(0.5, 0.55, 0.6), intensity=600.0))


@configclass
class CommandsCfg:
    base_velocity = vel_mdp.UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=tuple(CFG["commands"]["resample_s"]),
        rel_standing_envs=CFG["commands"]["standing_fraction"],
        rel_heading_envs=0.0,
        heading_command=False,
        debug_vis=True,
        ranges=vel_mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=tuple(CFG["commands"]["lin_vel_x"]),
            lin_vel_y=tuple(CFG["commands"]["lin_vel_y"]),
            ang_vel_z=tuple(CFG["commands"]["ang_vel_z"]),
        ),
    )
    body_height = mdp.BodyHeightCommandCfg(
        resampling_time_range=tuple(CFG["commands"]["resample_s"]),
        range_m=(CFG["commands"]["height_mm"][0] / 1000.0, CFG["commands"]["height_mm"][1] / 1000.0),
        slew_m_per_s=CFG["commands"].get("height_slew_mm_s", 0.0) / 1000.0,
    )


def _dr_on(key, default):
    dr = CFG["domain_randomization"]
    return dr.get(key, default) if dr["enabled"] else default


@configclass
class ActionsCfg:
    feet = mdp.FootSpaceActionCfg(
        delay_steps=tuple(_dr_on("action_delay_steps", (0, 0))),
        calibration_error_deg=_dr_on("joint_calibration_error_deg", 0.0),
        foot_x_offset_mm=_dr_on("foot_x_offset_mm", 0.0),
        quantization_deg=CFG["robot"]["pwm_quantization_deg"],
        clock_hz_min=CFG["gait"]["clock_hz_min"],
        clock_hz_max=CFG["gait"]["clock_hz_max"],
        action_filter_alpha=CFG["robot"].get("action_filter_alpha", 1.0),
    )


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        """Order and scaling here IS the firmware contract (see policy_runner.cpp)."""

        gyro = ObsTerm(func=mdp.imu_gyro, scale=0.25, noise=Unoise(n_min=-0.15, n_max=0.15), history_length=ACTOR_HISTORY)
        gravity = ObsTerm(func=mdp.imu_gravity, noise=Unoise(n_min=-0.05, n_max=0.05), history_length=ACTOR_HISTORY)
        actions = ObsTerm(func=vel_mdp.last_action, history_length=ACTOR_HISTORY)
        velocity_cmd = ObsTerm(func=vel_mdp.generated_commands, params={"command_name": "base_velocity"})
        height_cmd = ObsTerm(func=mdp.height_command_obs)
        clock = ObsTerm(func=mdp.gait_clock) if GAIT_CLOCK else None

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class CriticCfg(ObsGroup):
        gyro = ObsTerm(func=mdp.imu_gyro, scale=0.25)
        gravity = ObsTerm(func=mdp.imu_gravity)
        actions = ObsTerm(func=vel_mdp.last_action)
        velocity_cmd = ObsTerm(func=vel_mdp.generated_commands, params={"command_name": "base_velocity"})
        height_cmd = ObsTerm(func=mdp.height_command_obs)
        clock = ObsTerm(func=mdp.gait_clock) if GAIT_CLOCK else None
        base_lin_vel = ObsTerm(func=vel_mdp.base_lin_vel)
        joint_pos = ObsTerm(func=vel_mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=vel_mdp.joint_vel_rel, scale=0.05)
        feet_contact = ObsTerm(func=mdp.foot_contacts, params={"sensor_cfg": FEET})
        body_height = ObsTerm(func=mdp.measured_body_height)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()


def _events_cfg():
    dr = CFG["domain_randomization"]
    on = dr["enabled"]

    @configclass
    class EventCfg:
        # TPE foot balls on floors from polished tile to rubber mat; dynamic
        # friction kept below static. The ground material is 1.0 (multiply).
        foot_material = EventTerm(
            func=vel_mdp.randomize_rigid_body_material,
            mode="startup",
            params={
                "asset_cfg": SceneEntityCfg("robot", body_names=".*_foot"),
                "static_friction_range": tuple(dr["foot_friction"]) if on else (0.85, 0.85),
                "dynamic_friction_range": (dr["foot_friction"][0] * 0.75, dr["foot_friction"][1] * 0.9) if on else (0.7, 0.7),
                "restitution_range": tuple(dr["foot_restitution"]) if on else (0.1, 0.1),
                "num_buckets": 64,
                "make_consistent": True,
            },
        )
        # Printed PLA / carbon parts (shank, thigh, hip block, body) when they touch.
        body_material = EventTerm(
            func=vel_mdp.randomize_rigid_body_material,
            mode="startup",
            params={
                "asset_cfg": SceneEntityCfg("robot", body_names="(base|.*_(hip|thigh|arm|pushrod|crank|rod|shank))"),
                "static_friction_range": tuple(dr["body_friction"]) if on else (0.45, 0.45),
                "dynamic_friction_range": tuple(dr["body_friction"]) if on else (0.4, 0.4),
                "restitution_range": (0.0, 0.1),
                "num_buckets": 16,
                "make_consistent": True,
            },
        )
        reset_base = EventTerm(
            func=vel_mdp.reset_root_state_uniform,
            mode="reset",
            params={
                "pose_range": {"x": (-0.3, 0.3), "y": (-0.3, 0.3), "yaw": (-3.14, 3.14)},
                "velocity_range": {k: (-0.15, 0.15) for k in ("x", "y", "z", "roll", "pitch", "yaw")},
            },
        )
        reset_joints = EventTerm(
            func=vel_mdp.reset_joints_by_offset,
            mode="reset",
            # Zero offsets: the leg is a closed linkage, so a random joint offset
            # would pull pins apart at reset. Variety comes from the DR instead.
            params={"position_range": (0.0, 0.0), "velocity_range": (0.0, 0.0)},
        )

    ev = EventCfg()
    if on:
        ev.add_base_mass = EventTerm(
            func=vel_mdp.randomize_rigid_body_mass,
            mode="startup",
            params={
                "asset_cfg": SceneEntityCfg("robot", body_names="base"),
                "mass_distribution_params": tuple(dr["added_body_mass_kg"]),
                "operation": "add",
            },
        )
        ev.leg_mass = EventTerm(
            func=vel_mdp.randomize_rigid_body_mass,
            mode="startup",
            params={
                "asset_cfg": SceneEntityCfg("robot", body_names=".*_(hip|thigh|shank)"),
                "mass_distribution_params": tuple(dr["leg_mass_scale"]),
                "operation": "scale",
            },
        )
        cx, cy, cz = dr["body_com_offset_m"]
        ev.base_com = EventTerm(
            func=vel_mdp.randomize_rigid_body_com,
            mode="startup",
            params={
                "asset_cfg": SceneEntityCfg("robot", body_names="base"),
                "com_range": {"x": (-cx, cx), "y": (-cy, cy), "z": (-cz, cz)},
            },
        )
        ev.servo_gains = EventTerm(
            func=vel_mdp.randomize_actuator_gains,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=".*_(hip|thigh|arm)_joint"),
                "stiffness_distribution_params": tuple(dr["servo_stiffness_scale"]),
                "damping_distribution_params": tuple(dr["servo_damping_scale"]),
                "operation": "scale",
            },
        )
        ev.joint_friction = EventTerm(
            func=vel_mdp.randomize_joint_parameters,
            mode="startup",
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=".*_(hip|thigh|arm)_joint"),
                "friction_distribution_params": tuple(dr["joint_friction"]),
                "operation": "abs",
            },
        )
        ev.pin_friction = EventTerm(
            func=vel_mdp.randomize_joint_parameters,
            mode="startup",
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=".*_(crank|pushrod|rod|knee)_joint"),
                "friction_distribution_params": tuple(dr["pin_friction_nm"]),
                "operation": "abs",
            },
        )
        ev.servo_effort = EventTerm(
            func=mdp.randomize_servo_strength,
            mode="reset",
            params={"asset_cfg": SceneEntityCfg("robot"), "scale_range": tuple(dr["servo_effort_scale"])},
        )
        ev.imu_bias = EventTerm(
            func=mdp.resample_imu_bias,
            mode="reset",
            params={"gyro_bias": dr["imu_gyro_bias_rad_s"], "tilt_bias_deg": dr["imu_gravity_tilt_bias_deg"]},
        )
        pv = dr["push_velocity_m_s"]
        ev.push_robot = EventTerm(
            func=vel_mdp.push_by_setting_velocity,
            mode="interval",
            interval_range_s=tuple(dr["push_interval_s"]),
            params={"velocity_range": {"x": (-pv, pv), "y": (-pv, pv)}},
        )
    return ev


def _rewards_cfg():
    w = CFG["rewards"]

    @configclass
    class RewardsCfg:
        track_lin_vel_xy = RewTerm(
            func=mdp.track_lin_vel_xy_yaw_frame_exp,
            weight=w["track_lin_vel_xy"],
            params={"command_name": "base_velocity", "std": w["lin_vel_tracking_std"]},
        )
        track_ang_vel_z = RewTerm(
            func=mdp.track_ang_vel_z_world_exp,
            weight=w["track_ang_vel_z"],
            params={"command_name": "base_velocity", "std": w["yaw_rate_tracking_std"]},
        )
        track_height = RewTerm(func=mdp.track_body_height_exp, weight=w["track_height"], params={"std": 0.02})
        gait_schedule = RewTerm(
            func=mdp.trot_schedule, weight=w["gait_schedule"], params={"sensor_cfg": FEET}
        )
        swing_height = RewTerm(
            func=mdp.swing_height_tracking, weight=w["swing_height"], params={"height_m": CFG["gait"]["swing_height_m"]}
        )
        feet_air_time = RewTerm(
            func=vel_mdp.feet_air_time,
            weight=w["feet_air_time"],
            params={"sensor_cfg": FEET, "command_name": "base_velocity", "threshold": w.get("feet_air_time_threshold_s", 0.25)},
        )
        foot_clearance = RewTerm(
            func=mdp.foot_clearance_penalty,
            weight=w["foot_clearance"],
            params={"target_m": w["foot_clearance_target_m"], "command_name": "base_velocity"},
        )
        lin_vel_z = RewTerm(func=vel_mdp.lin_vel_z_l2, weight=w["lin_vel_z"])
        ang_vel_xy = RewTerm(func=vel_mdp.ang_vel_xy_l2, weight=w["ang_vel_xy"])
        flat_orientation = RewTerm(func=vel_mdp.flat_orientation_l2, weight=w["flat_orientation"])
        action_rate = RewTerm(func=vel_mdp.action_rate_l2, weight=w["action_rate"])
        action_smoothness = RewTerm(func=mdp.action_smoothness_l2, weight=w["action_smoothness"])
        joint_torques = RewTerm(func=vel_mdp.joint_torques_l2, weight=w["joint_torques"])
        feet_slide = RewTerm(
            func=vel_mdp.feet_slide,
            weight=w["feet_slide"],
            params={
                "sensor_cfg": FEET,
                "asset_cfg": SceneEntityCfg("robot", body_names=mdp.FOOT_BODIES, preserve_order=True),
            },
        )
        undesired_contacts = RewTerm(
            func=vel_mdp.undesired_contacts,
            weight=w["undesired_contacts"],
            params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_(thigh|shank|hip)"), "threshold": 1.0},
        )
        stand_still = RewTerm(func=mdp.stand_still_actions, weight=w["stand_still"])
        foot_hover = RewTerm(func=mdp.foot_hover_penalty, weight=w.get("foot_hover", 0.0),
                             params={"sensor_cfg": FEET, "max_air_s": w.get("foot_hover_max_s", 0.5)})
        # Spot-style: level, gliding body and soft footfalls
        base_heave_accel = RewTerm(func=mdp.base_heave_accel, weight=w.get("base_heave_accel", 0.0))
        foot_touchdown = RewTerm(
            func=mdp.foot_touchdown_speed, weight=w.get("foot_touchdown", 0.0), params={"sensor_cfg": FEET}
        )
        body_level = RewTerm(func=mdp.body_level_exp, weight=w.get("body_level", 0.0), params={"std_rad": 0.05})

    return RewardsCfg()


@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=vel_mdp.time_out, time_out=True)
    body_contact = DoneTerm(
        func=vel_mdp.illegal_contact,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names="base"), "threshold": 1.0},
    )
    bad_orientation = DoneTerm(func=vel_mdp.bad_orientation, params={"limit_angle": math.radians(60.0)})


@configclass
class CurriculumCfg:
    terrain_levels = CurrTerm(func=vel_mdp.terrain_levels_vel)


@configclass
class DominoWalkEnvCfg(ManagerBasedRLEnvCfg):
    scene: DominoSceneCfg = DominoSceneCfg(num_envs=CFG["training"]["num_envs"], env_spacing=1.5)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards: object = _rewards_cfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: object = _events_cfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        self.decimation = 4  # 200 Hz physics, 50 Hz policy = firmware kControlIntervalMs 20
        self.episode_length_s = 20.0
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        self.sim.physics_material = self.scene.terrain.physics_material
        if hasattr(self.sim, "physx"):  # Isaac Lab 2.x
            self.sim.physx.gpu_max_rigid_patch_count = 10 * 2**15
        else:  # Isaac Lab 3.x: PhysX is one of several backends; select it explicitly
            # (the closed linkage loops are PhysX spherical joints).
            from isaaclab_physx.physics import PhysxCfg

            self.sim.physics = PhysxCfg(gpu_max_rigid_patch_count=10 * 2**15)
            # Our servo model (ServoActuator) runs in Isaac Lab, not as a native actuator.
            self.sim.use_newton_actuators = False
        self.scene.contact_forces.update_period = self.sim.dt
        if not CFG["terrain"]["rough"] or not CFG["terrain"].get("curriculum", True):
            # Random difficulty per tile. Isaac Lab's velocity curriculum scores
            # distance walked per episode, which our mid-episode command changes
            # (direction and speed every 4-8 s) make meaningless.
            self.curriculum.terrain_levels = None
            self.scene.terrain.terrain_generator.curriculum = False
            self.scene.terrain.max_init_terrain_level = None
        else:
            self.scene.terrain.terrain_generator.curriculum = True
        self.viewer.eye = (1.2, 1.2, 0.6)
        self.viewer.origin_type = "asset_root"
        self.viewer.asset_name = "robot"


@configclass
class DominoWalkEnvCfg_PLAY(DominoWalkEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 16
        self.scene.env_spacing = 1.0
        self.scene.terrain.max_init_terrain_level = None
        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.num_rows = 3
            self.scene.terrain.terrain_generator.num_cols = 3
            self.scene.terrain.terrain_generator.curriculum = False
        self.observations.policy.enable_corruption = False
        self.events.push_robot = None
