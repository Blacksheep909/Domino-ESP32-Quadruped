"""Domino layout and the policy's action contract (shared by the env, the
exporter, the tests and the firmware docs).

The leg itself (thigh, bell crank, push rod, parallel rod, shank) is modelled
in domino_lab/linkage.py, a port of the firmware's CAD linkage solver, and in
the pinned URDF/USD built by tools/build_urdf.py. This module only holds what
the policy and the firmware agree on:

* Hip layout (`BODY_HALF_LENGTH_X`, `BODY_HALF_WIDTH_Y` in src/main.cpp).
* The policy acts in *foot space*, in the firmware's `moveLeg(x, y, z)`
  command frame (mm, per leg): +x forward, +y outward, +z down. The neutral
  stand command is (-15.75, 38, ride_height). On the robot each policy step
  becomes four `moveLeg` calls; in sim the same targets go through the ported
  solver to the three servo axes.

Functions take an array namespace (`numpy` or `torch`).
"""

from __future__ import annotations

HIP_X = 0.1675            # BODY_HALF_LENGTH_X
HIP_Y = 0.062375          # BODY_HALF_WIDTH_Y
FOOT_RADIUS = 0.012

# Leg order used everywhere (actions, firmware kMoveLegFns, exported header).
LEG_NAMES = ("FL", "FR", "BL", "BR")
LEG_SIDE = (+1, -1, +1, -1)        # +1 = left (+y), -1 = right
LEG_FRONT = (+1, +1, -1, -1)       # +1 = front (+x)

# Firmware command frame neutral (mm) and the ride-height band the remote's
# left stick spans (kRideHeightMinMm .. kRideHeightMaxMm).
NEUTRAL_X_MM = -15.75
NEUTRAL_Y_MM = 38.0
RIDE_HEIGHT_MIN_MM = 220.0
RIDE_HEIGHT_MAX_MM = 280.0
# Firmware command z = 280 puts the foot centre 275.99 mm below the hip axis
# (CAD neutral foot), so measured height = command - 4.0 mm.
NEUTRAL_FOOT_BELOW_HIP_M = 0.275993
COMMAND_TO_FOOT_Z_M = 0.280 - NEUTRAL_FOOT_BELOW_HIP_M

# Policy action -> foot offset scale (mm per unit action) and hard clamps that
# the firmware applies too. Keep in sync with `src/policy_runner.cpp`.
ACTION_SCALE_MM = (40.0, 20.0, 40.0)
FOOT_DX_LIMIT_MM = (-80.0, 80.0)
FOOT_Y_LIMIT_MM = (8.0, 78.0)
FOOT_Z_LIMIT_MM = (165.0, 300.0)


# Gait clock: its rate follows the stick, from GAIT_HZ_MIN when walking
# slowly to GAIT_HZ_MAX at full command. The ESP32 integrates the same phase
# (src/policy_runner.cpp), so the policy's clock input means the same thing.
GAIT_FULL_COMMAND = (0.5, 0.25, 1.5)  # vx m/s, vy m/s, wz rad/s at full stick


def gait_frequency_hz(xp, vx, vy, wz, f_min, f_max):
    a = xp.abs
    level = xp.maximum(xp.maximum(a(vx) / GAIT_FULL_COMMAND[0], a(vy) / GAIT_FULL_COMMAND[1]), a(wz) / GAIT_FULL_COMMAND[2])
    level = xp.clip(level, 0.0, 1.0) if xp.__name__ == "numpy" else xp.clamp(level, 0.0, 1.0)
    return f_min + (f_max - f_min) * level


def hip_origin(leg: int) -> tuple[float, float, float]:
    return (LEG_FRONT[leg] * HIP_X, LEG_SIDE[leg] * HIP_Y, 0.0)


def clamp_foot_command_mm(xp, x, y, z):
    x = xp.clip(x, NEUTRAL_X_MM + FOOT_DX_LIMIT_MM[0], NEUTRAL_X_MM + FOOT_DX_LIMIT_MM[1])
    y = xp.clip(y, FOOT_Y_LIMIT_MM[0], FOOT_Y_LIMIT_MM[1])
    z = xp.clip(z, FOOT_Z_LIMIT_MM[0], FOOT_Z_LIMIT_MM[1])
    return x, y, z


def actions_to_foot_mm(xp, actions, height_mm):
    """actions: (..., 12) in leg order FL,FR,BL,BR x (dx, dy, dz).

    height_mm broadcasts against (...,). Returns x, y_outward, z each (..., 4)
    in mm, already clamped to the firmware-safe box.
    """
    a = actions.reshape(*actions.shape[:-1], 4, 3)
    h = height_mm
    if hasattr(h, "shape") and len(getattr(h, "shape", ())) > 0:
        h = h[..., None]
    x = NEUTRAL_X_MM + ACTION_SCALE_MM[0] * a[..., 0]
    y = NEUTRAL_Y_MM + ACTION_SCALE_MM[1] * a[..., 1]
    z = h + ACTION_SCALE_MM[2] * a[..., 2]
    return clamp_foot_command_mm(xp, x, y, z)
