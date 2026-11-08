"""Record headless Isaac physics for offline rendering (tools/render.py).

Works without the Isaac Sim viewport, so it runs on drivers where the RTX
renderer crashes. Writes an .npz with every link's world pose per control step,
plus a caption per step.

    python scripts/record.py legtest --out legtest.npz
    python scripts/record.py policy --checkpoint runs/<run>/model_3000.pt --out walk.npz

Scenarios
  legtest  scripted actions on the neutral stand: lift each foot in turn,
           shift it forward and outward, then lower and raise the body.
           No policy: checks that each action moves the intended foot.
  policy   a checkpoint driven through the same command script as play.py.
"""

from __future__ import annotations

import argparse
import os
import sys

import h5py  # noqa: F401  # isort: skip  (see train.py: load before Kit)
from isaaclab.app import AppLauncher

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from domino_lab import compat as _compat  # noqa: E402  (no Isaac imports)

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

parser = argparse.ArgumentParser()
parser.add_argument("scenario", choices=["legtest", "standtest", "policy"])
parser.add_argument("--checkpoint", default=None)
parser.add_argument("--out", required=True)
parser.add_argument("--num_envs", type=int, default=4)
parser.add_argument("--flat", action="store_true", help="flat ground instead of the play terrain")
parser.add_argument("--no_dr", action="store_true")
parser.add_argument("--script", default="tour", help="policy command script: tour | forward | turn | height")
parser.add_argument("--stochastic", action="store_true",
                    help="sample actions with the policy's exploration noise, as during training")
parser.add_argument("--keep_resets", action="store_true",
                    help="keep fall terminations and 6 s episodes, as in training (robots respawn when they fall)")
_compat.add_launcher_args(parser)
args = parser.parse_args()
from domino_lab import config as _domino_config  # noqa: E402
_domino_config.use_checkpoint_config(getattr(args, "checkpoint", None))
args.headless = True
app = AppLauncher(_compat.launcher_args(args)).app

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry  # noqa: E402

from isaaclab.terrains import TerrainImporter  # noqa: E402

import domino_lab  # noqa: E402,F401
from domino_lab import geometry as g  # noqa: E402
from domino_lab.compat import T, quat_wxyz  # noqa: E402,F401

# Keep the generated terrain mesh so the renderer can draw bumps and slopes.
_TERRAIN = {}
_import_mesh = TerrainImporter.import_mesh


def _capture_mesh(self, name, mesh):
    _TERRAIN[name] = mesh
    return _import_mesh(self, name, mesh)


TerrainImporter.import_mesh = _capture_mesh
from domino_lab import mdp  # noqa: E402

# (seconds, vx, vy, wz, height_m, label)
SCRIPTS = {
    "tour": [
        (3, 0.0, 0.0, 0.0, 0.28, "stand 280 mm"),
        (5, 0.25, 0.0, 0.0, 0.28, "forward 0.25 m/s"),
        (4, 0.0, 0.0, 0.6, 0.28, "turn left 0.6 rad/s"),
        (4, 0.0, 0.0, -0.6, 0.28, "turn right 0.6 rad/s"),
        (4, 0.0, 0.12, 0.0, 0.26, "sideways left 0.12 m/s"),
        (4, -0.18, 0.0, 0.0, 0.25, "reverse 0.18 m/s"),
        (5, 0.2, 0.0, 0.3, 0.23, "forward + turn, low 230 mm"),
        (3, 0.0, 0.0, 0.0, 0.22, "stand 220 mm"),
        (5, 0.45, 0.0, 0.0, 0.28, "fast forward 0.45 m/s"),
        (4, 0.0, 0.0, 1.3, 0.28, "fast turn 1.3 rad/s"),
        (4, 0.0, -0.22, 0.0, 0.27, "fast sideways right 0.22 m/s"),
        (3, 0.0, 0.0, 0.0, 0.28, "stand"),
    ],
    "forward": [(2, 0, 0, 0, 0.28, "stand"), (8, 0.3, 0, 0, 0.28, "forward 0.30 m/s")],
    "turn": [(2, 0, 0, 0, 0.28, "stand"), (6, 0, 0, 0.75, 0.28, "turn 0.75 rad/s")],
    "height": [(3, 0, 0, 0, 0.28, "280"), (3, 0, 0, 0, 0.22, "220"), (3, 0, 0, 0, 0.28, "280")],
}


def legtest_actions(t: float) -> tuple[np.ndarray, float, str]:
    """Scripted 12-dim action and ride height for the leg test."""
    a = np.zeros(12)
    names = g.LEG_NAMES
    seg = 2.0
    i = int(t // seg)
    phase = (t % seg) / seg
    bump = np.sin(np.pi * phase)  # 0 -> 1 -> 0
    if i < 4:  # lift each foot by 60 mm (dz = -1.5 * 40 mm)
        a[3 * i + 2] = -1.5 * bump
        return a, 0.28, f"lift {names[i]} foot 60 mm"
    if i < 8:  # same foot forward 40 mm and outward 20 mm while lifted
        k = i - 4
        a[3 * k + 0] = 1.0 * bump
        a[3 * k + 1] = 1.0 * bump
        a[3 * k + 2] = -1.0 * bump
        return a, 0.28, f"{names[k]} foot forward + outward"
    if i < 10:  # body down to 220 and back via the height command
        h = 0.28 - 0.06 * (bump if i == 8 else 0.0)
        return a, h, f"ride height {h * 1000:.0f} mm"
    return a, 0.28, "stand"


def main():
    task = "Domino-Walk-Play-v0"
    cfg = load_cfg_from_registry(task, "env_cfg_entry_point")
    cfg.scene.num_envs = args.num_envs
    cfg.commands.base_velocity.resampling_time_range = (1e9, 1e9)
    cfg.commands.base_velocity.rel_standing_envs = 0.0
    cfg.commands.body_height.resampling_time_range = (1e9, 1e9)
    if args.keep_resets:
        cfg.episode_length_s = 6.0
    else:
        cfg.episode_length_s = 1e6
        cfg.terminations.bad_orientation = None
        cfg.terminations.body_contact = None
    if args.scenario == "legtest":
        # Pin the body 45 cm up so the feet hang free: checks action -> foot only.
        cfg.scene.robot.spawn.articulation_props.fix_root_link = True
        cfg.scene.robot.init_state.pos = (0.0, 0.0, 0.45)
    if args.flat or args.scenario in ("legtest", "standtest"):
        cfg.scene.terrain.terrain_type = "plane"
        cfg.scene.terrain.terrain_generator = None
        cfg.curriculum.terrain_levels = None
    if args.no_dr or args.scenario in ("legtest", "standtest"):
        for name in ("add_base_mass", "leg_mass", "base_com", "servo_gains", "joint_friction", "servo_effort", "imu_bias"):
            if hasattr(cfg.events, name):
                setattr(cfg.events, name, None)
        cfg.events.reset_base.params["pose_range"] = {"yaw": (0.0, 0.0)}
        cfg.events.reset_base.params["velocity_range"] = {}
        # Start exactly on the neutral stand (joint defaults), no random offset.
        cfg.events.reset_joints.params["position_range"] = (0.0, 0.0)
        cfg.actions.feet.delay_steps = (0, 0)
        cfg.actions.feet.calibration_error_deg = 0.0
        cfg.actions.feet.foot_x_offset_mm = 0.0
    env = gym.make(task, cfg=cfg)
    base = env.unwrapped
    robot = base.scene["robot"]
    names = robot.body_names
    vel = base.command_manager.get_term("base_velocity")
    height = base.command_manager.get_term("body_height")
    contact = base.scene.sensors["contact_forces"]
    foot_ids, _ = robot.find_bodies(mdp.FOOT_BODIES, preserve_order=True)
    foot_ids_sensor, _ = contact.find_bodies(mdp.FOOT_BODIES, preserve_order=True)

    policy = None
    if args.scenario == "policy":
        from rsl_rl.runners import OnPolicyRunner

        from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper

        agent_cfg = load_cfg_from_registry(task, "rsl_rl_cfg_entry_point")
        wrapped = RslRlVecEnvWrapper(env)
        runner = OnPolicyRunner(wrapped, _compat.runner_cfg(agent_cfg), log_dir=None, device=agent_cfg.device)
        _compat.load_checkpoint(runner, args.checkpoint, optimizer=False, iteration=False)
        policy = runner.get_inference_policy(device=base.device)
        if args.stochastic:
            policy = _compat.stochastic_policy(runner)
        obs = wrapped.get_observations()
        script = SCRIPTS[args.script]
        total = sum(s[0] for s in script)
    else:
        env.reset()
        total = 22.0 if args.scenario == "legtest" else 12.0
    dt = base.step_dt
    steps = int(total / dt)
    P, Q, INFO, FEET, CONTACT, CMD, VEL, H = [], [], [], [], [], [], [], []
    for k in range(steps):
        t = k * dt
        if args.scenario == "standtest":
            act = np.zeros(12)
            h = 0.28 if t < 3 else (0.22 if t < 7 else 0.28)
            label = f"stand, ride height command {h * 1000:.0f} mm"
        if args.scenario == "legtest":
            act, h, label = legtest_actions(t)
        if args.scenario in ("legtest", "standtest"):
            vel.vel_command_b[:] = 0.0
            getattr(height, "target", height.height)[:, 0] = h
            with torch.inference_mode():
                env.step(torch.tensor(act, dtype=torch.float32, device=base.device).repeat(base.num_envs, 1))
            cmd = (0.0, 0.0, 0.0, h)
        else:
            acc = 0.0
            for seg in script:
                if t < acc + seg[0]:
                    break
                acc += seg[0]
            _, vx, vy, wz, h, label = seg
            vel.vel_command_b[:, 0] = vx
            vel.vel_command_b[:, 1] = vy
            vel.vel_command_b[:, 2] = wz
            getattr(height, "target", height.height)[:, 0] = h
            with torch.inference_mode():
                obs, _, _, _ = wrapped.step(policy(obs))
            cmd = (vx, vy, wz, h)
        P.append(T(robot.data.body_pos_w).cpu().numpy() - base.scene.env_origins.cpu().numpy()[:, None])
        Q.append(quat_wxyz(robot.data.body_quat_w).cpu().numpy())  # recordings stay (w, x, y, z)
        FEET.append(T(robot.data.body_pos_w)[:, foot_ids].cpu().numpy() - base.scene.env_origins.cpu().numpy()[:, None])
        f = torch.linalg.norm(T(contact.data.net_forces_w)[:, foot_ids_sensor], dim=-1) > 1.0
        CONTACT.append(f.cpu().numpy())
        CMD.append(cmd)
        VEL.append(T(robot.data.root_lin_vel_b).cpu().numpy())
        H.append(mdp.measured_body_height(base).cpu().numpy()[:, 0])
        v = VEL[-1]
        INFO.append([
            f"t {t:5.2f}s  {label}\n"
            f"cmd  vx {cmd[0]:+.2f}  vy {cmd[1]:+.2f}  wz {cmd[2]:+.2f}  h {cmd[3] * 1000:.0f} mm\n"
            f"real vx {v[e, 0]:+.2f}  vy {v[e, 1]:+.2f}  body {H[-1][e] * 1000:.0f} mm\n"
            f"feet down {''.join('X' if c else '.' for c in CONTACT[-1][e])} (FL FR BL BR)"
            for e in range(base.num_envs)
        ])
    extra = {}
    if "terrain" in _TERRAIN:
        # Crop to 6 x 6 m around robot 0 and express it in robot 0's env frame.
        mesh = _TERRAIN["terrain"]
        origin = base.scene.env_origins[0].cpu().numpy()
        v = np.asarray(mesh.vertices) - origin
        f = np.asarray(mesh.faces)
        keep = (np.abs(v[f][..., :2]) < 3.0).all(axis=(1, 2))
        f = f[keep]
        used, inv = np.unique(f, return_inverse=True)
        extra = {"terrain_vertices": v[used].astype(np.float32), "terrain_faces": inv.reshape(-1, 3).astype(np.int32)}
    np.savez_compressed(
        args.out,
        **extra,
        link_names=np.array(names),
        pos=np.asarray(P, np.float32),
        quat=np.asarray(Q, np.float32),
        feet=np.asarray(FEET, np.float32),
        contacts=np.asarray(CONTACT),
        cmd=np.asarray(CMD, np.float32),
        vel=np.asarray(VEL, np.float32),
        body_height=np.asarray(H, np.float32),
        info=np.array(INFO, dtype=object),
        fps=1.0 / dt,
        env_origins=base.scene.env_origins.cpu().numpy().astype(np.float32),
    )
    print("RECORDED", args.out, len(P), "steps", flush=True)
    env.close()


if __name__ == "__main__":
    main()
    app.close()
