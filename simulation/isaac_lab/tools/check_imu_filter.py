"""Sim-to-real check of the ESP32 gravity estimate.

The policy is trained on the true gravity direction (plus noise). On the robot,
src/policy_runner.cpp estimates it from the MPU6050: gyro integration pulled
toward the accelerometer at kAccelBlendPerSecond. While trotting, the
accelerometer also measures the body's own accelerations, so the estimate can
be off. This replays a recorded walk (scripts/record.py) through the same
filter, using ideal gyro and accelerometer signals derived from the physics,
and reports the error in degrees.

    python tools/check_imu_filter.py runs/visual/walk.npz [blend_per_second ...]
"""

import sys

import numpy as np


def quat_to_R(q):
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    R = np.empty(q.shape[:-1] + (3, 3))
    R[..., 0, 0] = 1 - 2 * (y * y + z * z)
    R[..., 0, 1] = 2 * (x * y - w * z)
    R[..., 0, 2] = 2 * (x * z + w * y)
    R[..., 1, 0] = 2 * (x * y + w * z)
    R[..., 1, 1] = 1 - 2 * (x * x + z * z)
    R[..., 1, 2] = 2 * (y * z - w * x)
    R[..., 2, 0] = 2 * (x * z - w * y)
    R[..., 2, 1] = 2 * (y * z + w * x)
    R[..., 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def run(path, blends):
    r = np.load(path, allow_pickle=True)
    b = list(r["link_names"]).index("base")
    dt = 1.0 / float(r["fps"])
    pos = r["pos"][:, :, b].astype(np.float64)  # (T, N, 3) world
    R = quat_to_R(r["quat"][:, :, b].astype(np.float64))  # body -> world
    g_world = np.array([0.0, 0.0, -9.81])
    # True gravity direction in the body frame.
    g_true = np.einsum("tnji,j->tni", R, g_world / 9.81)
    # Ideal gyro: body angular velocity from consecutive rotations.
    dR = np.einsum("tnji,tnjk->tnik", R[:-1], R[1:])  # R_t^T R_{t+1}
    w = np.stack([dR[..., 2, 1] - dR[..., 1, 2], dR[..., 0, 2] - dR[..., 2, 0], dR[..., 1, 0] - dR[..., 0, 1]], -1) / (2 * dt)
    # Ideal accelerometer: specific force (a - g) in the body frame, in g.
    acc = np.gradient(np.gradient(pos, dt, axis=0), dt, axis=0)
    f_body = np.einsum("tnji,tnj->tni", R, acc - g_world) / 9.81
    print(f"{path}: {pos.shape[1]} robots, {pos.shape[0] * dt:.0f} s")
    print(f"  raw accelerometer alone: mean error {angle(-f_body, g_true).mean():.1f} deg, 95th pct {np.percentile(angle(-f_body, g_true), 95):.1f} deg")
    for blend in blends:
        est = np.empty_like(g_true)
        g = g_true[0].copy()
        for t in range(len(g_true)):
            if t > 0:
                om = w[t - 1]
                g = g + np.cross(g, om) * dt
                a = -f_body[t] / np.linalg.norm(f_body[t], axis=-1, keepdims=True)
                alpha = min(1.0, blend * dt)
                g = (1 - alpha) * g + alpha * a
                g /= np.linalg.norm(g, axis=-1, keepdims=True)
            est[t] = g
        err = angle(est, g_true)
        print(f"  filter blend {blend:4.1f}/s: mean error {err.mean():.2f} deg, 95th pct {np.percentile(err, 95):.2f} deg, max {err.max():.2f} deg")


def angle(a, b):
    a = a / np.linalg.norm(a, axis=-1, keepdims=True)
    b = b / np.linalg.norm(b, axis=-1, keepdims=True)
    return np.degrees(np.arccos(np.clip((a * b).sum(-1), -1, 1)))


if __name__ == "__main__":
    blends = [float(x) for x in sys.argv[2:]] or [0.5, 2.0, 5.0, 20.0]
    run(sys.argv[1], blends)
