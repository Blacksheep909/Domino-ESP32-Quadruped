"""Export an RSL-RL checkpoint to the ESP32 firmware.

    python export_policy.py runs/<run>/model_3000.pt [out_dir] [--install]

Writes
    out_dir/policy_weights.h   C header: const float weights (flash/rodata),
                               observation normaliser folded in, a self-test
                               vector and the policy contract constants.
    out_dir/policy.json        Contract + size/latency budget for Studio.
--install also copies the header to <repo>/src/policy_weights.h so the
normal Studio firmware build flashes it.

Only needs torch + numpy (no Isaac Sim), so Studio can run it with any
Python that has torch; train.py calls it automatically at the end of a run.
"""

from __future__ import annotations

import datetime as _dt
import hashlib
import json
import os
import re
import shutil
import sys

import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(ROOT))
sys.path.insert(0, ROOT)

from domino_lab import geometry as g  # noqa: E402

# ESP32-D0WD (DevKitC V4, WROOM-32E): 240 MHz Xtensa LX6 with a single
# precision FPU, 520 KB SRAM (no PSRAM), 4 MB flash with a 32 KB flash cache.
ESP32_BUDGET = {
    "cpu_mhz": 240,
    "sram_kb": 520,
    "free_heap_kb_typical": 180,
    "flash_app_partition_kb": 3072,
    "flash_cache_kb": 32,
    "control_period_ms": 20,
    # Measured-order estimate for float MAC with weights streamed from flash.
    "ns_per_mac_estimate": 25,
}

OBS_LAYOUT = [
    # name, size, history, scale/notes - MUST match env_cfg.ObservationsCfg.PolicyCfg
    ("gyro", 3, 3, "body rad/s x0.25 (+x fwd, +y left, +z up)"),
    ("gravity", 3, 3, "unit gravity in body frame (level = 0,0,-1)"),
    ("actions", 12, 3, "previous raw policy outputs"),
    ("velocity_cmd", 3, 1, "vx m/s, vy m/s, wz rad/s"),
    ("height_cmd", 1, 1, "(h - 0.25 m) / 0.03 m"),
    ("clock", 2, 1, "sin, cos of the gait phase; rate clock_hz_min..max by command level"),
]


def _actor_layers(sd: dict):
    idx = sorted({int(m.group(1)) for k in sd if (m := re.match(r"actor\.(\d+)\.weight$", k))})
    return [(sd[f"actor.{i}.weight"].float().cpu().numpy(), sd[f"actor.{i}.bias"].float().cpu().numpy()) for i in idx]


def _forward(layers, mean, inv_std, obs):
    x = (obs - mean) * inv_std
    for i, (w, b) in enumerate(layers):
        x = x @ w.T + b
        if i < len(layers) - 1:
            x = np.where(x > 0, x, np.expm1(np.minimum(x, 0)))  # ELU
    return x


def _cf(value: float) -> str:
    """C float literal that is always valid (2 -> 2.0f, 1e-05 -> 1e-05f)."""
    text = f"{float(value):.9g}"
    if not any(ch in text for ch in ".eEn"):
        text += ".0"
    return text + "f"


def _c_array(name, arr, per_line=8):
    flat = np.asarray(arr, dtype=np.float32).reshape(-1)
    body = ",\n  ".join(
        ", ".join(_cf(v) for v in flat[i : i + per_line]) for i in range(0, len(flat), per_line)
    )
    return f"static const float {name}[{len(flat)}] = {{\n  {body}\n}};\n"


def export(checkpoint: str, out_dir: str, install: bool = False) -> dict:
    ckpt = torch.load(checkpoint, map_location="cpu", weights_only=False)
    if "model_state_dict" in ckpt:  # rsl-rl 3.x (Isaac Lab 2.x)
        sd = ckpt["model_state_dict"]
    else:  # rsl-rl 5.x (Isaac Lab 3.x): separate actor model -> same names as 3.x
        sd = {}
        for k, v in ckpt["actor_state_dict"].items():
            if k.startswith("mlp."):
                sd["actor." + k[4:]] = v
            elif k.startswith("obs_normalizer."):
                sd["actor_obs_normalizer." + k[len("obs_normalizer."):]] = v
    layers = _actor_layers(sd)
    obs_dim = layers[0][0].shape[1]
    act_dim = layers[-1][0].shape[0]
    run_dir = os.path.dirname(os.path.abspath(checkpoint))
    run_cfg = {}
    if os.path.exists(os.path.join(run_dir, "run.json")):
        run_cfg = json.load(open(os.path.join(run_dir, "run.json"), encoding="utf-8")).get("config", {})
    uses_clock = bool(run_cfg.get("gait", {}).get("clock_obs", True))
    layout = [row for row in OBS_LAYOUT if uses_clock or row[0] != "clock"]
    height_slew = float(run_cfg.get("commands", {}).get("height_slew_mm_s", 80.0) or 80.0)
    expected = sum(n * h for _, n, h, _ in layout)
    if obs_dim != expected or act_dim != 12:
        raise ValueError(f"checkpoint has obs {obs_dim} / act {act_dim}; firmware contract is {expected} / 12")
    if "actor_obs_normalizer._mean" in sd:
        mean = sd["actor_obs_normalizer._mean"].float().reshape(-1).numpy()
        std = sd["actor_obs_normalizer._std"].float().reshape(-1).numpy()
        inv_std = 1.0 / (std + 1e-2)
    else:
        mean = np.zeros(obs_dim, np.float32)
        inv_std = np.ones(obs_dim, np.float32)

    gait = run_cfg.get("gait", {})
    clock_min = float(gait.get("clock_hz_min", gait.get("clock_hz", 2.0)))
    clock_max = float(gait.get("clock_hz_max", gait.get("clock_hz", 2.0)))

    rng = np.random.default_rng(1234)
    test_obs = (mean + rng.normal(0, 1, obs_dim) / inv_std).astype(np.float32)
    test_out = _forward(layers, mean, inv_std, test_obs.astype(np.float64)).astype(np.float32)

    digest = hashlib.sha1(b"".join(w.tobytes() + b.tobytes() for w, b in layers)).hexdigest()[:8]
    dims = [obs_dim] + [w.shape[0] for w, _ in layers]
    params = sum(w.size + b.size for w, b in layers)
    macs = sum(w.size for w, _ in layers)
    flash_bytes = 4 * (params + 2 * obs_dim + obs_dim + act_dim)
    scratch_bytes = 4 * 2 * max(dims)
    history_bytes = 4 * obs_dim
    est_us = macs * ESP32_BUDGET["ns_per_mac_estimate"] / 1000.0
    run_name = os.path.basename(run_dir)
    iteration = int(ckpt.get("iter", 0))

    os.makedirs(out_dir, exist_ok=True)
    h = []
    h.append("// Generated by simulation/isaac_lab/scripts/export_policy.py - do not edit.\n")
    h.append(f"// Run {run_name}, iteration {iteration}, exported {_dt.datetime.now().isoformat(timespec='seconds')}.\n")
    h.append("#pragma once\n\n#include <stdint.h>\n\n")
    h.append("#define DOMINO_POLICY_AVAILABLE 1\n")
    h.append(f"#define DOMINO_POLICY_GAIT_CLOCK {1 if uses_clock else 0}\n")
    h.append(f"#define DOMINO_POLICY_HEIGHT_SLEW_MM_S {_cf(height_slew)}\n")
    h.append(f'#define DOMINO_POLICY_ID "{digest}"\n')
    h.append(f'#define DOMINO_POLICY_RUN "{run_name}"\n')
    h.append(f"#define DOMINO_POLICY_ITERATION {iteration}\n")
    h.append(f"constexpr int kPolicyObsDim = {obs_dim};\n")
    h.append(f"constexpr int kPolicyActDim = {act_dim};\n")
    h.append(f"constexpr int kPolicyLayerCount = {len(layers)};\n")
    h.append(f"constexpr int kPolicyLayerDims[{len(dims)}] = {{{', '.join(map(str, dims))}}};\n")
    h.append(f"constexpr int kPolicyMaxWidth = {max(dims)};\n")
    h.append(f"constexpr float kPolicyClockHzMin = {_cf(clock_min)};\n")
    h.append(f"constexpr float kPolicyClockHzMax = {_cf(clock_max)};\n")
    h.append(f"constexpr float kPolicyFullCommand[3] = {{{', '.join(_cf(v) for v in g.GAIT_FULL_COMMAND)}}};\n")
    h.append(f"constexpr float kPolicyActionScaleMm[3] = {{{', '.join(_cf(v) for v in g.ACTION_SCALE_MM)}}};\n")
    h.append("constexpr float kPolicyActionClip = 1.5f;\n")
    alpha = float(run_cfg.get("robot", {}).get("action_filter_alpha", 1.0))
    h.append(f"constexpr float kPolicyActionFilterAlpha = {_cf(alpha)};\n\n")
    for i, (w, b) in enumerate(layers):
        h.append(_c_array(f"kPolicyW{i}", w))
        h.append(_c_array(f"kPolicyB{i}", b))
    h.append(f"static const float* const kPolicyWeights[{len(layers)}] = {{{', '.join(f'kPolicyW{i}' for i in range(len(layers)))}}};\n")
    h.append(f"static const float* const kPolicyBiases[{len(layers)}] = {{{', '.join(f'kPolicyB{i}' for i in range(len(layers)))}}};\n")
    h.append(_c_array("kPolicyObsMean", mean))
    h.append(_c_array("kPolicyObsInvStd", inv_std))
    h.append(_c_array("kPolicySelfTestObs", test_obs))
    h.append(_c_array("kPolicySelfTestAction", test_out))
    header = "".join(h)
    with open(os.path.join(out_dir, "policy_weights.h"), "w", newline="\n", encoding="utf-8") as f:
        f.write(header)

    info = {
        "policy_id": digest,
        "run": run_name,
        "iteration": iteration,
        "checkpoint": os.path.abspath(checkpoint),
        "exported": _dt.datetime.now().isoformat(timespec="seconds"),
        "layer_dims": dims,
        "activation": "elu",
        "params": int(params),
        "macs": int(macs),
        "flash_bytes": int(flash_bytes),
        "ram_bytes": int(scratch_bytes + history_bytes),
        "estimated_inference_us": round(est_us, 1),
        "control_budget_us": ESP32_BUDGET["control_period_ms"] * 1000,
        "esp32": ESP32_BUDGET,
        "observation_layout": [
            {"name": n, "size": s, "history": hist, "notes": note} for n, s, hist, note in layout
        ],
        "action": {
            "dim": 12,
            "order": [f"{leg}_{ax}" for leg in g.LEG_NAMES for ax in ("dx", "dy", "dz")],
            "scale_mm": list(g.ACTION_SCALE_MM),
            "neutral_mm": [g.NEUTRAL_X_MM, g.NEUTRAL_Y_MM, "ride_height"],
            "clip": 1.5,
        },
        "clock_hz": [clock_min, clock_max],
        "installed": False,
    }
    if install:
        shutil.copyfile(os.path.join(out_dir, "policy_weights.h"), os.path.join(REPO, "src", "policy_weights.h"))
        info["installed"] = True
        info["installed_to"] = os.path.join(REPO, "src", "policy_weights.h")
    with open(os.path.join(out_dir, "policy.json"), "w", encoding="utf-8") as f:
        json.dump(info, f, indent=2)
    return info


if __name__ == "__main__":
    argv = [a for a in sys.argv[1:] if not a.startswith("--")]
    if not argv:
        print(__doc__)
        sys.exit(2)
    ckpt = argv[0]
    out = argv[1] if len(argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(ckpt)), "policy")
    print(json.dumps(export(ckpt, out, install="--install" in sys.argv), indent=2))
