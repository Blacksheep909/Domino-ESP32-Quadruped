#!/usr/bin/env bash
# Build and run the firmware SIL scenario on macOS or Linux (see test.ps1).
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bash "$script_dir/build.sh" domino_sil_test

runtime_dir="$script_dir/viewer/runtime"
mkdir -p "$runtime_dir"
rm -f "$runtime_dir/telemetry.jsonl"

"$script_dir/bin/domino_sil_test" \
  --state-file "$runtime_dir/state.json" \
  --telemetry-file "$runtime_dir/telemetry.jsonl"
