"""Summarise a scripts/record.py policy recording: per command segment, how
well the robots followed it and how they moved their feet.

    python tools/analyze_recording.py runs/visual/walk.npz
"""

import sys

import numpy as np


def quat_yaw(q):
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def tilt_deg(q):
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    up_z = 1 - 2 * (x * x + y * y)
    return np.degrees(np.arccos(np.clip(up_z, -1, 1)))


def main(path):
    r = np.load(path, allow_pickle=True)
    names = list(r["link_names"])
    b = names.index("base")
    cmd = r["cmd"]
    pos = r["pos"][:, :, b]
    quat = r["quat"][:, :, b]
    vel = r["vel"]
    c = r["contacts"]
    h = r["body_height"]
    feet = r["feet"]
    dt = 1.0 / float(r["fps"])
    n_env = pos.shape[1]
    # segment boundaries where the command changes
    change = np.r_[0, np.where(np.any(np.diff(cmd, axis=0) != 0, axis=1))[0] + 1, len(cmd)]
    yaw = np.unwrap(quat_yaw(quat), axis=0)
    print(f"{path}: {n_env} robots, {len(cmd) * dt:.0f} s")
    print(f"{'command':34s} {'vx real':>8s} {'vy real':>8s} {'wz real':>8s} {'height':>7s} {'tilt':>5s} {'step Hz':>7s} {'clear mm':>8s} {'4-feet%':>7s} {'fell':>4s}")
    for s0, s1 in zip(change[:-1], change[1:]):
        a = s0 + min(int(1.0 / dt), (s1 - s0) // 3)  # skip the first second (transient)
        seg = slice(a, s1)
        vx, vy, wz, hh = cmd[s0]
        rv = vel[seg].mean(axis=(0, 1))
        rwz = (yaw[s1 - 1] - yaw[a]).mean() / ((s1 - 1 - a) * dt)
        touchdowns = np.logical_and(c[seg][1:], ~c[seg][:-1]).sum(axis=0).mean()  # per env and foot
        step_hz = touchdowns / ((s1 - a) * dt)  # touchdowns per foot per second
        clearance = (feet[seg][..., 2].max(axis=0) - feet[seg][..., 2].min(axis=0)).mean() * 1000
        all4 = c[seg].all(axis=2).mean() * 100
        fell = (tilt_deg(quat[seg]) > 50).any(axis=0).sum()
        label = f"vx {vx:+.2f} vy {vy:+.2f} wz {wz:+.2f} h {hh * 1000:.0f}"
        print(f"{label:34s} {rv[0]:+8.2f} {rv[1]:+8.2f} {rwz:+8.2f} {h[seg].mean() * 1000:7.0f} {tilt_deg(quat[seg]).mean():5.1f} "
              f"{step_hz:7.2f} {clearance:8.0f} {all4:7.0f} {fell:4d}")


if __name__ == "__main__":
    main(sys.argv[1])


def per_foot(path, t0=None, t1=None):
    """Per-foot gait quality while walking: swing lift, dragging and timing.

    lift      median of the peak height of each swing above the lowest foot (mm)
    drag      share of time the foot slides (> 5 cm/s) while within 10 mm of
              the ground - scuffing / dragging
    steps/s   touchdowns per second
    stance    share of time on the ground
    """
    r = np.load(path, allow_pickle=True)
    dt = 1.0 / float(r["fps"])
    cmd = r["cmd"]
    moving = (np.abs(cmd[:, 0]) + np.abs(cmd[:, 1]) + 0.3 * np.abs(cmd[:, 2])) > 0.05
    sl = slice(t0, t1)
    feet = r["feet"][sl]  # (T, N, 4, 3)
    c = r["contacts"][sl]
    mv = moving[sl]
    z = feet[..., 2]
    lift = z - z.min(axis=2, keepdims=True)
    vxy = np.linalg.norm(np.diff(feet[..., :2], axis=0), axis=-1) / dt
    names = ["FL", "FR", "BL", "BR"]
    print(f"{'foot':5s} {'lift mm':>8s} {'drag %':>7s} {'steps/s':>8s} {'stance %':>9s}")
    for i, n in enumerate(names):
        lifts = []
        for e in range(z.shape[1]):
            air = ~c[:, e, i] & mv
            # split into swings
            idx = np.where(air)[0]
            if len(idx) == 0:
                continue
            breaks = np.where(np.diff(idx) > 1)[0]
            for seg in np.split(idx, breaks + 1):
                if len(seg) >= 3:
                    lifts.append(lift[seg, e, i].max() * 1000)
        low = lift[1:, :, i] < 0.010
        drag = ((vxy[:, :, i] > 0.05) & low & mv[1:, None]).sum() / max(1, (mv[1:, None] & np.ones_like(low)).sum())
        td = np.logical_and(c[1:, :, i], ~c[:-1, :, i])[mv[1:]].sum() / max(1e-9, mv.sum() * dt * z.shape[1])
        stance = c[mv][:, :, i].mean()
        print(f"{n:5s} {np.median(lifts) if lifts else 0:8.1f} {drag * 100:7.1f} {td:8.2f} {stance * 100:9.0f}")


if __name__ == "__main__" and len(sys.argv) > 2 and sys.argv[2] == "--feet":
    per_foot(sys.argv[1])
