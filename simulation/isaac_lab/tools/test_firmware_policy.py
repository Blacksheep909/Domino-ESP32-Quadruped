"""Compile src/policy_runner.cpp natively with an exported policy header and
check that the firmware's forward pass reproduces the exporter's reference
output, and that a few closed-loop steps stay finite and inside the foot box.

    python tools/test_firmware_policy.py runs/<run>/policy/policy_weights.h

Uses PlatformIO's MinGW g++ (the same toolchain as the SIL build).
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))

MAIN = r"""
#include <math.h>
#include <stdio.h>
#include "policy_runner.h"

int main() {
  if (!policySelfTest()) {
    printf("SELFTEST FAIL err=%g\n", policyStatus().selfTestMaxError);
    return 1;
  }
  printf("selftest ok err=%.3g id=%s obs=%d\n", policyStatus().selfTestMaxError, policyId(), policyObsDim());
  PolicyImu imu{};
  imu.accelG[2] = 1.0f;  // level, still
  imu.fresh = true;
  policyReset(imu);
  PolicyCommand cmd{0.2f, 0.0f, 0.0f, 260.0f};
  PolicyFeet feet{};
  for (int i = 0; i < 250; ++i) {
    imu.gyroRadps[0] = 0.05f * sinf(i * 0.3f);
    if (!policyStep(cmd, imu, 0.02f, &feet)) {
      printf("STEP FAIL %d fault=%s\n", i, policyFaultName(policyStatus().fault));
      return 1;
    }
    for (int leg = 0; leg < 4; ++leg) {
      if (!(feet.zMm[leg] >= 165.0f && feet.zMm[leg] <= 300.0f) || !(feet.yOutMm[leg] >= 8.0f && feet.yOutMm[leg] <= 78.0f)) {
        printf("FOOT OUT OF BOX step %d leg %d\n", i, leg);
        return 1;
      }
    }
  }
  printf("closed loop ok: FL foot %.1f %.1f %.1f\n", feet.xMm[0], feet.yOutMm[0], feet.zMm[0]);
  // Tilt fault must trip.
  imu.accelG[0] = 1.0f; imu.accelG[2] = 0.0f;
  for (int i = 0; i < 200 && policyStatus().fault == PolicyFault::None; ++i) policyStep(cmd, imu, 0.02f, &feet);
  printf("tilt fault=%s\n", policyFaultName(policyStatus().fault));
  return policyStatus().fault == PolicyFault::Tilt ? 0 : 1;
}
"""


def main(header: str) -> int:
    gxx = os.path.join(os.path.expanduser("~"), ".platformio", "packages", "toolchain-gccmingw32", "bin", "g++.exe")
    if not os.path.exists(gxx):
        gxx = shutil.which("g++") or gxx
    work = tempfile.mkdtemp(prefix="domino-policy-test-")
    for name in ("policy_runner.h", "policy_runner.cpp"):
        shutil.copy(os.path.join(REPO, "src", name), work)
    shutil.copy(header, os.path.join(work, "policy_weights.h"))
    with open(os.path.join(work, "main.cpp"), "w") as f:
        f.write(MAIN)
    exe = os.path.join(work, "policy_test.exe")
    env = dict(os.environ, PATH=os.path.dirname(gxx) + os.pathsep + os.environ.get("PATH", ""))
    build = subprocess.run([gxx, "-std=gnu++17", "-O2", "-Wall", "-Wextra", "-o", exe, "main.cpp", "policy_runner.cpp"],
                           cwd=work, capture_output=True, text=True, env=env)
    if build.returncode != 0:
        print(build.stdout + build.stderr)
        return 1
    run = subprocess.run([exe], cwd=work, capture_output=True, text=True, env=env)
    print(run.stdout.strip())
    return run.returncode


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
