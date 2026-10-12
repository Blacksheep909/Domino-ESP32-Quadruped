$ErrorActionPreference = "Stop"

$testExecutableName = "domino_sil_test.exe"
& (Join-Path $PSScriptRoot "build.ps1") -OutputName $testExecutableName

$runtimeDirectory = Join-Path $PSScriptRoot "viewer\runtime"
New-Item -ItemType Directory -Force -Path $runtimeDirectory | Out-Null
$statePath = Join-Path $runtimeDirectory "state.json"
$telemetryPath = Join-Path $runtimeDirectory "telemetry.jsonl"
Remove-Item -LiteralPath $telemetryPath -ErrorAction SilentlyContinue

& (Join-Path $PSScriptRoot "bin\$testExecutableName") `
    --state-file $statePath `
    --telemetry-file $telemetryPath

if ($LASTEXITCODE -ne 0) {
    throw "SIL scenario failed with exit code $LASTEXITCODE"
}
