"""Parity test: domino_lab/linkage.py vs the firmware's own C++ leg solver.

Compiles a test program that #includes src/leg_controller.cpp (so the
anonymous-namespace solver is reachable) against the SIL Arduino stubs,
solves the same random foot targets in the same order for all four legs,
and compares the servo deltas.

    python tools/test_firmware_linkage.py      (with a Python that has torch)
"""

from __future__ import annotations

import os
import subprocess
import sys
import tempfile

import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(ROOT))
sys.path.insert(0, ROOT)

from domino_lab import linkage as L  # noqa: E402

N = 400
HARNESS = r"""
#include <stdio.h>
#include "leg_controller.cpp"
int main() {
  const LegConfig* legs[4] = {&FL_leg, &FR_leg, &BL_leg, &BR_leg};
  float x, y, z; int leg;
  while (scanf("%d %f %f %f", &leg, &x, &y, &z) == 4) {
    float sh = 0, up = 0, lo = 0;
    bool ok = solveCadEndpoint(*legs[leg], x, y, z, &sh, &up, &lo);
    printf("%d %.6f %.6f %.6f\n", ok ? 1 : 0, sh, up, lo);
  }
  return 0;
}
"""


def build():
    gxx = os.path.join(os.path.expanduser("~"), ".platformio", "packages", "toolchain-gccmingw32", "bin", "g++.exe")
    work = tempfile.mkdtemp(prefix="domino-linkage-")
    with open(os.path.join(work, "main.cpp"), "w") as f:
        f.write(HARNESS)
    exe = os.path.join(work, "linkage.exe")
    cmd = [gxx, "-std=gnu++17", "-O2", "-DDOMINO_SIL", "-I", os.path.join(REPO, "simulation", "sil", "include"),
           "-I", os.path.join(REPO, "src"), "-o", exe, os.path.join(work, "main.cpp"),
           os.path.join(REPO, "src", "servo_calibration.cpp"), os.path.join(REPO, "simulation", "sil", "src", "arduino_sim.cpp"),
           os.path.join(REPO, "simulation", "sil", "src", "pwm_sim.cpp")]
    env = dict(os.environ, PATH=os.path.dirname(gxx) + os.pathsep + os.environ.get("PATH", ""))
    r = subprocess.run(cmd, capture_output=True, text=True, env=env)
    if r.returncode != 0:
        print(r.stdout[-3000:], r.stderr[-3000:])
        sys.exit(1)
    return exe, env


def main():
    exe, env = build()
    rng = np.random.default_rng(3)
    targets = []
    for leg in range(4):
        side = L.SIDE[L.LEG_ORDER[leg]]
        for _ in range(N):
            x = L.NEUTRAL_COMMAND_X + rng.uniform(-60, 60)
            y = side * (L.NEUTRAL_COMMAND_Y + rng.uniform(-25, 30))
            z = rng.uniform(200, 290)
            targets.append((leg, x, y, z))
    inp = "".join(f"{l} {x:.4f} {y:.4f} {z:.4f}\n" for l, x, y, z in targets)
    out = subprocess.run([exe], input=inp, capture_output=True, text=True, env=env).stdout.split("\n")
    fw = np.array([[float(v) for v in line.split()] for line in out if line.strip()])
    worst = {}
    for leg in range(4):
        label = L.LEG_ORDER[leg]
        g = L.CadGeometry(label, dtype=torch.float32)
        side = L.SIDE[label]
        su = torch.zeros(1)
        sl = torch.zeros(1)
        rows = [i for i, t in enumerate(targets) if t[0] == leg]
        diffs = []
        oks = 0
        for i in rows:
            _, x, y, z = targets[i]
            sh, up, lo, ok, _ = g.solve_endpoint(torch.tensor([x]), torch.tensor([y]), torch.tensor([z]), side, su, sl)
            if bool(ok):
                su, sl = up, lo
            okf = fw[i, 0] == 1
            oks += int(okf == bool(ok))
            if okf and bool(ok):
                diffs.append(np.abs(np.array([sh.item(), up.item(), lo.item()]) - fw[i, 1:]).max())
        worst[label] = (max(diffs) if diffs else float("nan"), oks, len(rows), len(diffs))
    # Closed-form solver used by the simulator, same targets, all legs at once.
    ik = L.LinkageIK(dtype=torch.float64)
    per = len(targets) // 4
    arr = np.array([t[1:] for t in targets]).reshape(4, per, 3).transpose(1, 0, 2)
    root, up, lo, okc = ik.solve(torch.tensor(arr[..., 0]), torch.tensor(arr[..., 1]), torch.tensor(arr[..., 2]))
    fwr = fw.reshape(4, per, 4).transpose(1, 0, 2)
    both = (fwr[..., 0] == 1) & okc.numpy()
    cf = np.abs(np.stack([up.numpy(), lo.numpy()], -1) - fwr[..., 2:4])[both].max()
    same = ((fwr[..., 0] == 1) == okc.numpy()).mean()
    print(f"closed-form (simulator): same success flag {same * 100:.2f}%, max |delta difference| vs firmware {cf:.4f} deg")
    for label, (d, oks, n, solved) in worst.items():
        print(f"{label}: solved {solved}/{n}, same success flag {oks}/{n}, max |delta difference| {d:.4f} deg")
    ok = all(v[0] < 0.05 and v[1] == v[2] for v in worst.values()) and cf < 0.05 and same > 0.995
    print("LINKAGE_PARITY", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
