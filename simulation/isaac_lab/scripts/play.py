"""Watch or drive a trained Domino policy in Isaac Sim (visible window).

    run-train.ps1 -Play -Checkpoint runs/<run>/model_3000.pt          # scripted demo, 4 robots (real time)
    run-train.ps1 -Play -Drive -Checkpoint runs/<run>/model_3000.pt   # drive it yourself

Demo mode cycles stand / forward / turn / sideways / reverse at two ride
heights. Drive mode puts one robot under your control: plug the Radiomaster
Boxer in over USB (EdgeTX "USB Joystick" mode) and it behaves like the real
dog: SB fully down runs the policy, sticks drive, CH3 sets ride height. With
no radio, use the keyboard (see domino_lab/remote.py). Domain randomisation
stays on (except pushes) so the robots are as varied as the real dog.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

# Load h5py (and its HDF5 DLLs) before Isaac Sim starts: in a visible window
# Kit loads its own HDF5 first and h5py then fails with "DLL load failed".
import h5py  # noqa: F401,E402  isort: skip
from isaaclab.app import AppLauncher

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from domino_lab import compat as _compat  # noqa: E402  (no Isaac imports)

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

parser = argparse.ArgumentParser()
parser.add_argument("--checkpoint", type=str, required=True)
parser.add_argument("--num_envs", type=int, default=None, help="default 4 (demo, real time) or 1 (drive); 8 runs ~0.9x, 16 ~0.7x")
parser.add_argument("--render_every", type=int, default=2, help="draw every Nth policy step (1 = 50 fps)")
parser.add_argument("--follow", action="store_true", help="camera tracks robot 0 (default when driving)")
parser.add_argument("--free_camera", action="store_true", help="free mouse camera even when driving")
parser.add_argument("--scene", default="showroom", choices=["showroom", "terrain"], help="showroom: grid floor with wooden props; terrain: the rough training terrain")
parser.add_argument("--screenshot", default=None, help="save the Isaac viewport to this PNG (at --screenshot_at seconds)")
parser.add_argument("--screenshot_at", type=float, default=6.0)
parser.add_argument("--arrows", action="store_true", help="show the command arrows (broken on Isaac Sim 6.2)")
parser.add_argument("--seconds", type=float, default=0.0, help="0 = until the window is closed")
parser.add_argument("--drive", action="store_true", help="drive with the Boxer radio or the keyboard")
parser.add_argument("--video", action="store_true")
_compat.add_launcher_args(parser)
args = parser.parse_args()
from domino_lab import config as _domino_config  # noqa: E402
_domino_config.use_checkpoint_config(getattr(args, "checkpoint", None))
args.device_set = any(a == "--device" or a.startswith("--device=") for a in sys.argv)
app = AppLauncher(_compat.launcher_args(args)).app

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402
from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry  # noqa: E402

import domino_lab  # noqa: E402,F401
from domino_lab import remote  # noqa: E402
from domino_lab.compat import T  # noqa: E402

# (vx, vy, wz, height m) - the scripted "remote"
SCRIPT = [
    (0.0, 0.0, 0.0, 0.28),
    (0.25, 0.0, 0.0, 0.28),
    (0.25, 0.0, 0.5, 0.28),
    (0.0, 0.0, -0.7, 0.28),
    (0.0, 0.12, 0.0, 0.25),
    (-0.18, 0.0, 0.0, 0.25),
    (0.2, 0.0, 0.0, 0.22),
    (0.0, 0.0, 0.0, 0.22),
]
SEGMENT_S = 4.0


class KeyboardDriver:
    """Keyboard fallback, read from the Isaac Sim window."""

    def __init__(self):
        import carb.input
        import omni.appwindow

        self._carb = carb.input
        self.held: set[str] = set()
        self.height_m = remote.RIDE_MAX_MM / 1000.0
        self.policy_on = True
        self.reset_requested = False
        window = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._sub = self._input.subscribe_to_keyboard_events(window.get_keyboard(), self._on_key)

    def _on_key(self, event, *_):
        name = event.input.name
        if event.type == self._carb.KeyboardEventType.KEY_PRESS:
            self.held.add(name)
            if name == "P":
                self.policy_on = not self.policy_on
            elif name == "BACKSPACE":
                self.reset_requested = True
        elif event.type == self._carb.KeyboardEventType.KEY_RELEASE:
            self.held.discard(name)
        return True

    def command(self, dt: float):
        k = self.held
        axis = lambda pos, neg: (1.0 if pos in k else 0.0) - (1.0 if neg in k else 0.0)  # noqa: E731
        forward, lateral, turn = axis("W", "S"), axis("A", "D"), axis("Q", "E")
        self.height_m += axis("R", "F") * 0.04 * dt
        self.height_m = min(remote.RIDE_MAX_MM, max(remote.RIDE_MIN_MM, self.height_m * 1000)) / 1000
        vx = forward * (remote.MAX_FORWARD if forward >= 0 else remote.MAX_REVERSE)
        return vx, lateral * remote.MAX_LATERAL, turn * remote.MAX_YAW, self.height_m, self.policy_on


# Wooden props around the demo robots (static colliders), Isaac-showroom style.
# (x, y, size_x, size_y, height, yaw_deg); robots start inside |x|,|y| < 1 m.
PROPS = [
    (2.6, 0.0, 0.9, 0.9, 0.10, 0.0), (2.4, 1.6, 0.6, 0.6, 0.20, 25.0), (2.2, -1.7, 0.7, 0.5, 0.06, -15.0),
    (-2.4, 1.2, 0.8, 0.8, 0.14, 10.0), (-2.6, -1.3, 1.0, 0.6, 0.08, -30.0), (0.3, 2.6, 1.2, 0.5, 0.05, 0.0),
    (-0.8, -2.5, 0.6, 0.6, 0.25, 40.0), (1.0, -2.6, 0.9, 0.7, 0.12, 0.0),
]
RAMPS = [(3.4, -0.2, 1.2, 0.8, 0.10, 0.0), (-3.3, 0.0, 1.0, 0.8, 0.14, 180.0)]  # x, y, length, width, rise, yaw


def _showroom(env_cfg):
    """Flat Isaac grid floor, soft sky light and wooden boxes / ramps to look at."""
    import math

    import isaaclab.sim as sim_utils
    from isaaclab.assets import AssetBaseCfg

    env_cfg.scene.terrain.terrain_type = "plane"
    env_cfg.scene.terrain.terrain_generator = None
    env_cfg.scene.terrain.max_init_terrain_level = None
    wood = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.74, 0.55, 0.36), roughness=0.65)
    wood_dark = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.62, 0.45, 0.29), roughness=0.7)

    def quat_z(deg):
        h = math.radians(deg) / 2.0
        return (math.cos(h), 0.0, 0.0, math.sin(h))

    def rot(qwxyz):
        # Isaac Lab 3 orders quaternions (x, y, z, w).
        return (qwxyz[1], qwxyz[2], qwxyz[3], qwxyz[0]) if _compat.LAB3 else qwxyz

    for i, (x, y, sx, sy, h, yaw) in enumerate(PROPS):
        setattr(env_cfg.scene, f"prop_box_{i}", AssetBaseCfg(
            prim_path=f"/World/Props/box_{i}",
            spawn=sim_utils.CuboidCfg(size=(sx, sy, h), visual_material=wood if i % 2 else wood_dark,
                                      collision_props=sim_utils.CollisionPropertiesCfg()),
            init_state=AssetBaseCfg.InitialStateCfg(pos=(x, y, h / 2.0), rot=rot(quat_z(yaw))),
        ))
    for i, (x, y, length, width, rise, yaw) in enumerate(RAMPS):
        pitch = math.atan2(rise, length)
        hp, hy = pitch / 2.0, math.radians(yaw) / 2.0
        # yaw then pitch (about the ramp's own y axis)
        q = (math.cos(hy) * math.cos(hp), -math.sin(hy) * math.sin(hp), math.cos(hy) * math.sin(hp), math.sin(hy) * math.cos(hp))
        setattr(env_cfg.scene, f"prop_ramp_{i}", AssetBaseCfg(
            prim_path=f"/World/Props/ramp_{i}",
            spawn=sim_utils.CuboidCfg(size=(math.hypot(length, rise), width, 0.02), visual_material=wood,
                                      collision_props=sim_utils.CollisionPropertiesCfg()),
            init_state=AssetBaseCfg.InitialStateCfg(pos=(x, y, rise / 2.0), rot=rot(q)),
        ))


def main():
    num_envs = args.num_envs or (1 if args.drive else 4)  # 4 robots run at real time in the viewer
    env_cfg = load_cfg_from_registry("Domino-Walk-Play-v0", "env_cfg_entry_point")
    agent_cfg = load_cfg_from_registry("Domino-Walk-Play-v0", "rsl_rl_cfg_entry_point")
    env_cfg.scene.num_envs = num_envs
    # Watching a few robots: PhysX on the CPU is ~5x faster per step than the GPU
    # pipeline (whose fixed per-call cost only pays off with thousands of robots).
    device = args.device if args.device_set else ("cpu" if num_envs <= 64 else "cuda:0")
    env_cfg.sim.device = device
    agent_cfg.device = device
    # Rewards are only for training; skip them when watching.
    for name in list(vars(env_cfg.rewards)):
        if not name.startswith("_"):
            setattr(env_cfg.rewards, name, None)
    env_cfg.curriculum.terrain_levels = None
    if args.scene == "showroom":
        _showroom(env_cfg)
    # Draw every Nth policy step (50 Hz policy: 2 -> 25 fps viewport).
    env_cfg.sim.render_interval = env_cfg.decimation * max(1, args.render_every)
    env_cfg.commands.base_velocity.resampling_time_range = (1e9, 1e9)
    env_cfg.commands.base_velocity.rel_standing_envs = 0.0
    env_cfg.commands.body_height.resampling_time_range = (1e9, 1e9)
    env_cfg.episode_length_s = 1e6
    # Isaac Lab's velocity arrows render broken (huge, at the origin) on Isaac Sim 6.2's
    # Fabric point instancer; keep them off until that is fixed upstream.
    env_cfg.commands.base_velocity.debug_vis = args.arrows
    if args.follow or args.drive and not args.free_camera:
        # Chase camera on robot 0 (re-aimed every frame, so mouse orbit is overridden).
        env_cfg.viewer.origin_type = "asset_root"
        env_cfg.viewer.eye = (0.9, 0.9, 0.5) if args.drive else env_cfg.viewer.eye
    else:
        # Free camera: orbit / pan / zoom with the mouse as usual in Isaac Sim.
        env_cfg.viewer.origin_type = "world"
        env_cfg.viewer.eye = (2.5, 2.5, 1.4)
        env_cfg.viewer.lookat = (0.0, 0.0, 0.2)
    env = gym.make("Domino-Walk-Play-v0", cfg=env_cfg, render_mode="rgb_array" if args.video else None)
    if args.video:
        env = gym.wrappers.RecordVideo(
            env, os.path.join(os.path.dirname(args.checkpoint), "videos"), step_trigger=lambda s: s == 0,
            video_length=int(len(SCRIPT) * SEGMENT_S / 0.02), disable_logger=True,
        )
    env = RslRlVecEnvWrapper(env)
    runner = OnPolicyRunner(env, _compat.runner_cfg(agent_cfg), log_dir=None, device=agent_cfg.device)
    _compat.load_checkpoint(runner, args.checkpoint, optimizer=False, iteration=False)
    policy = runner.get_inference_policy(device=env.unwrapped.device)
    base = env.unwrapped
    vel = base.command_manager.get_term("base_velocity")
    height = base.command_manager.get_term("body_height")
    obs = env.get_observations()
    dt = base.step_dt

    radio = remote.BoxerReader() if args.drive else None
    keys = KeyboardDriver() if args.drive else None
    if args.drive:
        print("[drive] Boxer: SB down = policy, sticks drive, CH3 height. "
              "Keyboard: W/S A/D Q/E, R/F height, P policy on/off, Backspace reset.", flush=True)
    policy_on = True
    step = 0
    last_print = 0.0
    rate_t0, rate_step = time.time(), 0
    play_t0 = time.time()
    shot_taken = False
    while app.is_running():
        if args.drive:
            if radio.fresh:
                vx, vy, wz, h, policy_on = remote.channels_to_command(radio.channels, policy_on)
                source = radio.name
            else:
                vx, vy, wz, h, policy_on = keys.command(dt)
                source = "keyboard"
            if keys.reset_requested:
                keys.reset_requested = False
                obs, _ = env.reset()
        else:
            vx, vy, wz, h = SCRIPT[int(step * dt / SEGMENT_S) % len(SCRIPT)]
        vel.vel_command_b[:, 0] = vx
        vel.vel_command_b[:, 1] = vy
        vel.vel_command_b[:, 2] = wz
        getattr(height, "target", height.height)[:, 0] = h
        with torch.inference_mode():
            actions = policy(obs)
            if args.drive and not policy_on:
                # SB not down on the real dog = plain stand at the ride height.
                actions = torch.zeros_like(actions)
            obs, _, _, _ = env.step(actions)
        step += 1
        if not args.video:  # real-time pacing when the sim runs faster than the clock
            ahead = step * dt - (time.time() - play_t0)
            if ahead > 0:
                time.sleep(ahead)
        if time.time() - rate_t0 > 3.0:
            rtf = (step - rate_step) * dt / (time.time() - rate_t0)
            print(f"[play] speed {rtf:.2f}x real time ({(step - rate_step) / (time.time() - rate_t0):.0f} policy steps/s)", flush=True)
            rate_t0, rate_step = time.time(), step
        if args.drive and time.time() - last_print > 0.5:
            last_print = time.time()
            v = T(base.scene["robot"].data.root_lin_vel_b)[0]
            print(f"[drive] {source:>12} | policy {'ON ' if policy_on else 'off'} | cmd vx {vx:+.2f} vy {vy:+.2f} "
                  f"wz {wz:+.2f} h {h * 1000:.0f} mm | actual vx {v[0]:+.2f} vy {v[1]:+.2f}", flush=True)
        if args.screenshot and not shot_taken and step * dt >= args.screenshot_at:
            shot_taken = True
            try:
                from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

                capture_viewport_to_file(get_active_viewport(), args.screenshot)
                print(f"[play] screenshot -> {args.screenshot}", flush=True)
            except Exception as exc:  # noqa: BLE001
                print(f"[play] screenshot failed: {exc}", flush=True)
        if args.seconds and step * dt >= args.seconds:
            break
        if args.video and step * dt >= len(SCRIPT) * SEGMENT_S:
            break
    if radio:
        radio.stop()
    env.close()


if __name__ == "__main__":
    main()
    app.close()
