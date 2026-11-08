#!/usr/bin/env bash
# Build the firmware software-in-the-loop simulator on macOS or Linux.
# Windows uses build.ps1 (MinGW). Usage: simulation/sil/build.sh [output-name]
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/../.." && pwd)"
output_name="${1:-domino_sil}"
output_dir="$script_dir/bin"
compiler="${CXX:-c++}"
mkdir -p "$output_dir"

"$compiler" \
  -std=c++11 -O2 -Wall -Wextra \
  -DDOMINO_SIL=1 \
  -I"$script_dir/include" \
  -I"$repo_root/src" \
  -I"$repo_root/lib/Ramp/src" \
  "$repo_root/src/main.cpp" \
  "$repo_root/src/crsf.cpp" \
  "$repo_root/src/gait_profile.cpp" \
  "$repo_root/src/manual_control_guard.cpp" \
  "$repo_root/src/motion_smoothing.cpp" \
  "$repo_root/src/power_fault_guard.cpp" \
  "$repo_root/src/power_monitor_math.cpp" \
  "$repo_root/src/leg_controller.cpp" \
  "$repo_root/src/servo_calibration.cpp" \
  "$repo_root/src/ik.cpp" \
  "$repo_root/src/policy_runner.cpp" \
  "$script_dir/src/arduino_sim.cpp" \
  "$script_dir/src/pwm_sim.cpp" \
  "$script_dir/src/imu_sim.cpp" \
  "$script_dir/src/sil_main.cpp" \
  -lpthread \
  -o "$output_dir/$output_name"

echo "Built $output_dir/$output_name"
