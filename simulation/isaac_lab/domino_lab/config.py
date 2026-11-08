"""Load the Studio-editable training config (config/training.json).

The env reads it at construction time, so edit the JSON (or use the Policy
Lab page in Studio) and start a new run; no Python changes needed. Set
DOMINO_TRAINING_CONFIG to point at a different file.
"""

from __future__ import annotations

import copy
import json
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_PATH = os.path.join(ROOT, "config", "training.json")


def _merge(base: dict, override: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _merge(out[k], v)
        else:
            out[k] = v
    return out


def load(path: str | None = None) -> dict:
    with open(DEFAULT_PATH, encoding="utf-8") as f:
        cfg = json.load(f)
    path = path or os.environ.get("DOMINO_TRAINING_CONFIG")
    if path and os.path.abspath(path) != os.path.abspath(DEFAULT_PATH):
        with open(path, encoding="utf-8") as f:
            cfg = _merge(cfg, json.load(f))
    return cfg


def use_checkpoint_config(checkpoint: str | None) -> str | None:
    """Point DOMINO_TRAINING_CONFIG at the config a checkpoint was trained with
    (its run.json), so playback / recording build the same observation layout.
    Call before the env config is imported. Returns the path used, if any."""
    if not checkpoint:
        return None
    run_json = os.path.join(os.path.dirname(os.path.abspath(checkpoint)), "run.json")
    if not os.path.exists(run_json):
        return None
    with open(run_json, encoding="utf-8") as f:
        cfg = json.load(f).get("config")
    if not cfg:
        return None
    # Runs from before these keys existed: the policy observed the gait clock.
    cfg.setdefault("gait", {}).setdefault("clock_obs", True)
    out = os.path.join(os.path.dirname(os.path.abspath(checkpoint)), "_playback_config.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2)
    os.environ["DOMINO_TRAINING_CONFIG"] = out
    return out
