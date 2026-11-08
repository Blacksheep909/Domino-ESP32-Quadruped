"""Domino's real leg linkage, shared by the sim, the exporter and the tests.

This is a batched PyTorch port of the firmware's CAD linkage solver
(src/leg_controller.cpp: solveCadPose / cadFootForServoDeltas /
solveCadPlanar / solveCadEndpoint), using the pin centres Studio's Virtual Lab
measured from the CAD (simulation/standalone/web/src/domino-config.js, copied
to assets/domino/studio_linkage.json). The firmware's kRightCad / kLeftCad
constants are the same numbers.

Per leg, in the planar x-z frame of the leg (x forward, z up, origin at the
hip abduction axis, mm):

    lower drive  (servo)  turns the thigh L2           pin lower_drive -> knee P6
    upper drive  (servo)  turns the arm                pin upper_drive -> P2
    push rod                                           P2 -> P3
    bell crank   (coaxial with the lower drive)        P1 -> P3 and P1 -> P5
    parallel rod (parallel to the thigh)               P5 -> P4
    shank L3 + bracket + foot, pivots on the knee      P6 -> P4, foot

Firmware names (kept here) differ from Studio's mesh names for two points:
firmware lowerClosureDriver = knee P6 (Studio "lower_closure_diagonal"),
firmware lowerClosureDiagonal = P4 (Studio "lower_closure_driver").

All angles follow the firmware: servo deltas in degrees, the solver rotates the
thigh by (-lowerDelta + neutralLowerOffset) and the arm by -upperDelta in the
mathematical x-z sense.
"""

from __future__ import annotations

import json
import math
import os

import torch

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STUDIO_JSON = os.path.join(ROOT, "assets", "domino", "studio_linkage.json")

LEG_ORDER = ("FL", "FR", "BL", "BR")  # firmware kMoveLegFns order
SIDE = {"FL": 1, "FR": -1, "BL": 1, "BR": -1}

# Firmware command frame neutral (src/leg_controller.cpp)
NEUTRAL_COMMAND_X = -15.75
NEUTRAL_COMMAND_Y = 38.0
NEUTRAL_COMMAND_Z = 280.0
CAD_NEUTRAL_FOOT_Y = 38.1
MAX_DELTA = 45.0
MAX_HIP_DELTA = 30.0
FD_DEG = 0.05


def load_studio(path: str = STUDIO_JSON) -> dict:
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    return {leg["label"]: leg for leg in data["legs"]}


def leg_points_mm(label: str, studio: dict | None = None) -> dict:
    """Pins of one leg relative to its hip origin, mm, {name: (x, y, z)}."""
    studio = studio or load_studio()
    pts = studio[label]["points"]
    hip = pts["hip_origin"]
    return {k: tuple((v[i] - hip[i]) * 1000.0 for i in range(3)) for k, v in pts.items()}


class CadGeometry:
    """Firmware CadGeometry for one side, as tensors on a device."""

    def __init__(self, label: str, device="cpu", dtype=torch.float32, studio=None):
        p = leg_points_mm(label, studio)
        xz = lambda name: torch.tensor([p[name][0], p[name][2]], device=device, dtype=dtype)  # noqa: E731
        self.lowerDrive = xz("lower_drive")
        self.upperDrive = xz("upper_drive")
        self.lowerPassive = xz("lower_passive")
        self.lowerClosureDriver = xz("lower_closure_diagonal")  # knee P6
        self.upperClosureDriver = xz("upper_closure_driver")  # P2
        self.upperClosureCoupler = xz("upper_closure_coupler")  # P3
        self.lowerCoupler = xz("lower_coupler")  # P5
        self.lowerClosureDiagonal = xz("lower_closure_driver")  # P4
        self.foot = xz("foot_tip")
        studio = studio or load_studio()
        self.neutralLowerOffsetDeg = float(studio[label]["neutralLinkageOffsetDeg"]["lower"])
        self.label = label
        # Firmware's neutralFoot is the foot at zero servo deltas (it equals
        # `foot` on the right; on the left it includes the 4.18 deg offset).
        nf = self.foot_for_deltas(torch.zeros(1, device=device, dtype=dtype), torch.zeros(1, device=device, dtype=dtype))[0]
        self.neutralFoot = nf[0]

    # -- solver ----------------------------------------------------------------
    @staticmethod
    def _rotate(point, pivot, angle):
        x = point[..., 0] - pivot[..., 0]
        z = point[..., 1] - pivot[..., 1]
        c, s = torch.cos(angle), torch.sin(angle)
        return torch.stack([pivot[..., 0] + c * x - s * z, pivot[..., 1] + s * x + c * z], dim=-1)

    @staticmethod
    def _orientation(a, b, p):
        return (b[..., 0] - a[..., 0]) * (p[..., 1] - a[..., 1]) - (b[..., 1] - a[..., 1]) * (p[..., 0] - a[..., 0])

    @classmethod
    def _circle(cls, ca, ra, cb, rb, assembly_orientation):
        """Intersection of two circles, picking the firmware's assembly branch.
        Returns (point, ok)."""
        d = cb - ca
        dist = torch.linalg.norm(d, dim=-1)
        lo = abs(ra - rb) if not torch.is_tensor(ra) else torch.abs(ra - rb)
        hi = ra + rb
        ok = (dist > 1e-5) & (dist >= lo - 1e-4) & (dist <= hi + 1e-4)
        dist_c = torch.clamp(torch.maximum(dist, torch.as_tensor(lo, device=dist.device)), max=hi)
        along = (ra * ra - rb * rb + dist_c * dist_c) / (2.0 * dist_c)
        height = torch.sqrt(torch.clamp(ra * ra - along * along, min=0.0))
        n = d / torch.clamp(dist, min=1e-9).unsqueeze(-1)
        base = ca + along.unsqueeze(-1) * n
        c0 = torch.stack([base[..., 0] - height * n[..., 1], base[..., 1] + height * n[..., 0]], -1)
        c1 = torch.stack([base[..., 0] + height * n[..., 1], base[..., 1] - height * n[..., 0]], -1)
        o0 = cls._orientation(ca, cb, c0) > 0.0
        pick0 = o0 == (assembly_orientation > 0.0)
        return torch.where(pick0.unsqueeze(-1), c0, c1), ok

    def pose(self, upper_deg, lower_deg):
        """Firmware solveCadPose: all moving pins for servo deltas (deg).
        Returns dict of (..., 2) tensors and an ok mask."""
        g = self
        shape = upper_deg.shape
        exp = lambda v: v.expand(*shape, 2)  # noqa: E731
        upper = -upper_deg * (math.pi / 180.0)
        lower = (-lower_deg + g.neutralLowerOffsetDeg) * (math.pi / 180.0)
        lowerPassive = g._rotate(exp(g.lowerPassive), exp(g.lowerDrive), lower)
        lowerClosureDriver = g._rotate(exp(g.lowerClosureDriver), exp(g.lowerDrive), lower)
        upperClosureDriver = g._rotate(exp(g.upperClosureDriver), exp(g.upperDrive), upper)
        r_coupler = torch.linalg.norm(g.lowerPassive - g.upperClosureCoupler)
        r_upper = torch.linalg.norm(g.upperClosureCoupler - g.upperClosureDriver)
        orient1 = g._orientation(g.lowerPassive, g.upperClosureDriver, g.upperClosureCoupler)
        upperClosureCoupler, ok1 = g._circle(lowerPassive, r_coupler, upperClosureDriver, r_upper, orient1)
        a0 = torch.atan2(g.upperClosureCoupler[1] - g.lowerPassive[1], g.upperClosureCoupler[0] - g.lowerPassive[0])
        a1 = torch.atan2(upperClosureCoupler[..., 1] - lowerPassive[..., 1], upperClosureCoupler[..., 0] - lowerPassive[..., 0])
        crank = a1 - a0
        lowerCoupler = g._rotate(exp(g.lowerCoupler), exp(g.lowerPassive), crank)
        lowerCoupler = lowerCoupler + (lowerPassive - g.lowerPassive)
        r_diag = torch.linalg.norm(g.lowerCoupler - g.lowerClosureDiagonal)
        r_lower = torch.linalg.norm(g.lowerClosureDriver - g.lowerClosureDiagonal)
        orient2 = g._orientation(g.lowerCoupler, g.lowerClosureDriver, g.lowerClosureDiagonal)
        lowerClosureDiagonal, ok2 = g._circle(lowerCoupler, r_diag, lowerClosureDriver, r_lower, orient2)
        return {
            "thigh_angle": lower,
            "arm_angle": upper,
            "crank_angle": crank,
            "lowerPassive": lowerPassive,
            "knee": lowerClosureDriver,
            "P2": upperClosureDriver,
            "P3": upperClosureCoupler,
            "P5": lowerCoupler,
            "P4": lowerClosureDiagonal,
        }, ok1 & ok2

    def foot_for_deltas(self, upper_deg, lower_deg):
        """Firmware cadFootForServoDeltas. Returns (foot (...,2), ok, pose)."""
        pose, ok = self.pose(upper_deg, lower_deg)
        g = self
        a0 = torch.atan2(g.lowerClosureDiagonal[1] - g.lowerClosureDriver[1], g.lowerClosureDiagonal[0] - g.lowerClosureDriver[0])
        knee, p4 = pose["knee"], pose["P4"]
        a1 = torch.atan2(p4[..., 1] - knee[..., 1], p4[..., 0] - knee[..., 0])
        ang = a1 - a0
        lx = g.foot[0] - g.lowerClosureDriver[0]
        lz = g.foot[1] - g.lowerClosureDriver[1]
        foot = torch.stack([knee[..., 0] + torch.cos(ang) * lx - torch.sin(ang) * lz,
                            knee[..., 1] + torch.sin(ang) * lx + torch.cos(ang) * lz], -1)
        pose["shank_angle"] = ang
        pose["foot"] = foot
        return foot, ok, pose

    def solve_planar(self, target, seed_upper, seed_lower, iterations: int = 10):
        """Firmware solveCadPlanar (Newton with finite differences)."""
        up, lo = seed_upper.clone(), seed_lower.clone()
        converged = torch.zeros_like(up, dtype=torch.bool)
        for _ in range(iterations):
            cur, ok0, _ = self.foot_for_deltas(up, lo)
            stepU, okU, _ = self.foot_for_deltas(up + FD_DEG, lo)
            stepL, okL, _ = self.foot_for_deltas(up, lo + FD_DEG)
            err = target - cur
            done = torch.linalg.norm(err, dim=-1) <= 0.05
            converged = converged | done
            j00 = (stepU[..., 0] - cur[..., 0]) / FD_DEG
            j10 = (stepU[..., 1] - cur[..., 1]) / FD_DEG
            j01 = (stepL[..., 0] - cur[..., 0]) / FD_DEG
            j11 = (stepL[..., 1] - cur[..., 1]) / FD_DEG
            det = j00 * j11 - j01 * j10
            safe = det.abs() > 1e-6
            det = torch.where(safe, det, torch.ones_like(det))
            dU = (err[..., 0] * j11 - j01 * err[..., 1]) / det
            dL = (j00 * err[..., 1] - err[..., 0] * j10) / det
            move = (~converged) & safe & ok0 & okU & okL
            up = torch.where(move, torch.clamp(up + torch.clamp(dU, -8, 8), -MAX_DELTA, MAX_DELTA), up)
            lo = torch.where(move, torch.clamp(lo + torch.clamp(dL, -8, 8), -MAX_DELTA, MAX_DELTA), lo)
        foot, ok, _ = self.foot_for_deltas(up, lo)
        ok = ok & (torch.linalg.norm(target - foot, dim=-1) <= 1.0)
        return up, lo, foot, ok

    def solve_endpoint(self, x, y, z, side: int, seed_upper, seed_lower):
        """Firmware solveCadEndpoint: leg command (mm; y body-signed) ->
        (shoulderDelta, upperDelta, lowerDelta, ok). shoulderDelta is the
        canonical outward-positive hip angle in degrees."""
        neutralY = side * CAD_NEUTRAL_FOOT_Y
        tx = self.neutralFoot[0] + (x - NEUTRAL_COMMAND_X)
        ty = neutralY + (y - side * NEUTRAL_COMMAND_Y)
        tz = self.neutralFoot[1] - (z - NEUTRAL_COMMAND_Z)
        radius = torch.sqrt(ty * ty + tz * tz)
        planar_z = -torch.sqrt(torch.clamp(radius * radius - neutralY * neutralY, min=0.0))
        up, lo, foot, ok = self.solve_planar(torch.stack([tx, planar_z], -1), seed_upper, seed_lower)
        base = torch.atan2(foot[..., 1], torch.full_like(foot[..., 1], neutralY))
        root = torch.atan2(tz, ty) - base
        root = torch.remainder(root + math.pi, 2 * math.pi) - math.pi
        shoulder = torch.clamp(-root * (180.0 / math.pi) / side, -MAX_HIP_DELTA, MAX_HIP_DELTA)
        return shoulder, up, lo, ok, root


class LinkageIK:
    """Closed-form inverse of the same linkage, for all four legs at once.

    Gives the solution the firmware's Newton solver converges to (it is the
    parallelogram's exact inverse), in one vectorised pass, so the simulator
    is not dominated by solver kernels:
      knee   = circle(lower_drive, thigh) n circle(foot target, knee->foot)
      shank  = angle of (foot - knee) vs the authored shank; crank = shank
      arm    = circle(upper_drive, arm) n circle(P3(crank), push rod)
    Deltas outside the firmware's +-45 deg (+-30 deg hip) clamp, and
    unreachable targets, return ok = False (the caller holds the last pose,
    as the firmware does). Verified against src/leg_controller.cpp by
    tools/test_firmware_linkage.py.
    """

    def __init__(self, device="cpu", dtype=torch.float32, studio=None):
        geos = [CadGeometry(leg, device=device, dtype=dtype, studio=studio) for leg in LEG_ORDER]
        st = lambda attr: torch.stack([getattr(g, attr) for g in geos])  # noqa: E731  (4, 2)
        self.L, self.U = st("lowerDrive"), st("upperDrive")
        self.P6, self.P4 = st("lowerClosureDriver"), st("lowerClosureDiagonal")
        self.P2, self.P3 = st("upperClosureDriver"), st("upperClosureCoupler")
        self.foot = st("foot")
        self.neutralFoot = st("neutralFoot")
        self.offset = torch.tensor([g.neutralLowerOffsetDeg for g in geos], device=device, dtype=dtype)
        self.side = torch.tensor([SIDE[l] for l in LEG_ORDER], device=device, dtype=dtype)
        n = lambda v: torch.linalg.norm(v, dim=-1)  # noqa: E731
        self.r_thigh = n(self.P6 - self.L)
        self.r_foot = n(self.foot - self.P6)
        self.r_arm = n(self.P2 - self.U)
        self.r_push = n(self.P3 - self.P2)
        ang = lambda v: torch.atan2(v[..., 1], v[..., 0])  # noqa: E731
        self.a_thigh0 = ang(self.P6 - self.L)
        self.a_shank0 = ang(self.foot - self.P6)
        self.a_arm0 = ang(self.P2 - self.U)
        # Branch signs of the authored assembly.
        self.o_knee = torch.sign(CadGeometry._orientation(self.L, self.foot, self.P6))
        self.o_arm = torch.sign(CadGeometry._orientation(self.U, self.P3, self.P2))

    @staticmethod
    def _wrap(a):
        return torch.remainder(a + math.pi, 2 * math.pi) - math.pi

    def _circles(self, ca, ra, cb, rb, orient):
        d = cb - ca
        dist = torch.linalg.norm(d, dim=-1)
        ok = (dist > 1e-6) & (dist <= ra + rb) & (dist >= torch.abs(ra - rb))
        dist_c = torch.clamp(dist, min=1e-6)
        along = (ra * ra - rb * rb + dist_c * dist_c) / (2 * dist_c)
        h = torch.sqrt(torch.clamp(ra * ra - along * along, min=0.0))
        nrm = d / dist_c.unsqueeze(-1)
        base = ca + along.unsqueeze(-1) * nrm
        c0 = torch.stack([base[..., 0] - h * nrm[..., 1], base[..., 1] + h * nrm[..., 0]], -1)
        c1 = torch.stack([base[..., 0] + h * nrm[..., 1], base[..., 1] - h * nrm[..., 0]], -1)
        pick0 = torch.sign(CadGeometry._orientation(ca, cb, c0)) == orient
        return torch.where(pick0.unsqueeze(-1), c0, c1), ok

    def solve(self, x, y_body, z):
        """x, y_body (body-signed), z: (n, 4) firmware leg commands in mm.
        Returns root (rad), upperDelta, lowerDelta (deg), ok, each (n, 4)."""
        side = self.side
        neutralY = side * CAD_NEUTRAL_FOOT_Y
        tx = self.neutralFoot[:, 0] + (x - NEUTRAL_COMMAND_X)
        ty = neutralY + (y_body - side * NEUTRAL_COMMAND_Y)
        tz = self.neutralFoot[:, 1] - (z - NEUTRAL_COMMAND_Z)
        planar_z = -torch.sqrt(torch.clamp(ty * ty + tz * tz - neutralY * neutralY, min=0.0))
        F = torch.stack([tx, planar_z], -1)  # (n, 4, 2)
        knee, ok1 = self._circles(self.L.expand_as(F), self.r_thigh, F, self.r_foot, self.o_knee)
        thigh = self._wrap(torch.atan2(knee[..., 1] - self.L[:, 1], knee[..., 0] - self.L[:, 0]) - self.a_thigh0)
        shank = self._wrap(torch.atan2(F[..., 1] - knee[..., 1], F[..., 0] - knee[..., 0]) - self.a_shank0)
        # Parallelogram: the bell crank turns with the shank about the lower drive.
        c, s = torch.cos(shank), torch.sin(shank)
        p3x = self.P3[:, 0] - self.L[:, 0]
        p3z = self.P3[:, 1] - self.L[:, 1]
        P3 = torch.stack([self.L[:, 0] + c * p3x - s * p3z, self.L[:, 1] + s * p3x + c * p3z], -1)
        P2, ok2 = self._circles(self.U.expand_as(P3), self.r_arm, P3, self.r_push, self.o_arm)
        arm = self._wrap(torch.atan2(P2[..., 1] - self.U[:, 1], P2[..., 0] - self.U[:, 0]) - self.a_arm0)
        upper = -arm * (180.0 / math.pi)
        lower = self.offset - thigh * (180.0 / math.pi)
        base = torch.atan2(planar_z, neutralY)
        root = self._wrap(torch.atan2(tz, ty) - base)
        shoulder = -root * (180.0 / math.pi) / side
        ok = ok1 & ok2 & (upper.abs() <= MAX_DELTA) & (lower.abs() <= MAX_DELTA) & (shoulder.abs() <= MAX_HIP_DELTA)
        return root, upper, lower, ok


def deltas_to_sim_joints(root_rad, upper_deg, lower_deg, offset_deg):
    """Firmware solver output -> sim joint angles (rad, URDF conventions,
    zero = Studio's authored stand pose):
      hip   about +x : the leg-plane rotation `root` from solveCadEndpoint
      thigh about +y : -(mathematical thigh rotation) = (lowerDelta - offset)
      arm   about +y : -(mathematical arm rotation)   = upperDelta
    """
    k = math.pi / 180.0
    return root_rad, (lower_deg - offset_deg) * k, upper_deg * k
