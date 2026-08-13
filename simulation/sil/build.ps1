param(
    [string]$OutputName = "domino_sil.exe"
)

$ErrorActionPreference = "Stop"

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$compiler = Join-Path $env:USERPROFILE ".platformio\packages\toolchain-gccmingw32\bin\g++.exe"
$compilerDirectory = Split-Path -Parent $compiler
$outputDirectory = Join-Path $PSScriptRoot "bin"
$outputPath = Join-Path $outputDirectory $OutputName

if (-not (Test-Path -LiteralPath $compiler)) {
    throw "Native compiler not found. Install it with: pio pkg install --global --tool platformio/toolchain-gccmingw32"
}

New-Item -ItemType Directory -Force -Path $outputDirectory | Out-Null
$env:PATH = "$compilerDirectory;$env:PATH"

$arguments = @(
    "-std=c++11",
    "-O2",
    "-Wall",
    "-Wextra",
    "-DDOMINO_SIL=1",
    "-static-libgcc",
    "-static-libstdc++",
    "-I$($PSScriptRoot)\include",
    "-I$($repoRoot)\src",
    "-I$($repoRoot)\lib\Ramp\src",
    (Join-Path $repoRoot "src\main.cpp"),
    (Join-Path $repoRoot "src\crsf.cpp"),
    (Join-Path $repoRoot "src\leg_controller.cpp"),
    (Join-Path $repoRoot "src\servo_calibration.cpp"),
    (Join-Path $repoRoot "src\ik.cpp"),
    (Join-Path $PSScriptRoot "src\arduino_sim.cpp"),
    (Join-Path $PSScriptRoot "src\pwm_sim.cpp"),
    (Join-Path $PSScriptRoot "src\imu_sim.cpp"),
    (Join-Path $PSScriptRoot "src\sil_main.cpp"),
    "-o",
    $outputPath
)

& $compiler @arguments
if ($LASTEXITCODE -ne 0) {
    throw "SIL compilation failed with exit code $LASTEXITCODE"
}

Write-Host "Built $outputPath"
