"""Checks for the action contract and the linkage port (no Isaac needed; torch).

Run:  python -m pytest simulation/isaac_lab/tests -q
The firmware parity test (compiles src/leg_controller.cpp) is
tools/test_firmware_linkage.py.
"""

import os
import sys

import numpy as np
import torch

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from domino_lab import geometry as g  # noqa: E402
from domino_lab import linkage as L  # noqa: E402


def test_zero_action_is_neutral_stance():
    x, y, z = g.actions_to_foot_mm(np, np.zeros((1, 12)), np.array([250.0]))
    assert np.allclose(x, g.NEUTRAL_X_MM) and np.allclose(y, g.NEUTRAL_Y_MM) and np.allclose(z, 250.0)


def test_neutral_command_solves_to_zero_servo_deltas():
    # The firmware's neutral command lands on the CAD neutral foot, i.e. zero
    # servo deltas (right) / the 4.18 deg left-CAD offset folded into the solver.
    for leg in L.LEG_ORDER:
        gm = L.CadGeometry(leg, dtype=torch.float64)
        side = L.SIDE[leg]
        t = lambda v: torch.tensor([v], dtype=torch.float64)  # noqa: E731
        sh, up, lo, ok, _ = gm.solve_endpoint(t(-15.75), t(side * 38.0), t(280.0), side, t(0.0), t(0.0))
        assert bool(ok)
        assert abs(up.item()) < 0.05 and abs(lo.item()) < 0.05 and abs(sh.item()) < 0.1


def test_action_box_is_reachable_and_avoids_the_toggle_corner():
    rng = np.random.default_rng(0)
    n = 5000
    a = rng.uniform(-1.5, 1.5, (n, 12))
    h = rng.uniform(g.RIDE_HEIGHT_MIN_MM, g.RIDE_HEIGHT_MAX_MM, n)
    x, y, z = g.actions_to_foot_mm(np, a, h)
    for i, leg in enumerate(L.LEG_ORDER):
        gm = L.CadGeometry(leg, dtype=torch.float64)
        s = L.SIDE[leg]
        T = lambda v: torch.tensor(v, dtype=torch.float64)  # noqa: E731
        sh, up, lo, ok, _ = gm.solve_endpoint(T(x[:, i]), T(s * y[:, i]), T(z[:, i]), s, torch.zeros(n, dtype=torch.float64), torch.zeros(n, dtype=torch.float64))
        assert ok.double().mean() > 0.995
        # Upper > +28 with lower < -33 is past the push-rod toggle (crossed linkage).
        assert not bool(((up > 28) & (lo < -33)).any())


def test_crank_follows_shank_parallelogram():
    gm = L.CadGeometry("BR", dtype=torch.float64)
    up = torch.linspace(-30, 25, 12, dtype=torch.float64)
    lo = torch.linspace(-15, 40, 12, dtype=torch.float64)
    _, ok, pose = gm.foot_for_deltas(up, lo)
    assert bool(ok.all())
    # Thigh and parallel rod stay parallel, so the shank turns with the crank.
    assert torch.allclose(pose["shank_angle"], pose["crank_angle"], atol=1e-6)
