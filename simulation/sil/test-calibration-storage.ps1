$ErrorActionPreference = "Stop"

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$compiler = Join-Path $env:USERPROFILE ".platformio\packages\toolchain-gccmingw32\bin\g++.exe"
if (-not (Test-Path -LiteralPath $compiler)) {
    throw "Native compiler not found. Install it with: pio pkg install --global --tool platformio/toolchain-gccmingw32"
}
$env:PATH = "$(Split-Path -Parent $compiler);$env:PATH"

$outputDirectory = Join-Path $PSScriptRoot "bin"
New-Item -ItemType Directory -Force -Path $outputDirectory | Out-Null
$outputPath = Join-Path $outputDirectory "domino_calibration_storage_test.exe"

& $compiler -std=c++11 -O1 -Wall -Wextra -static "-I$repoRoot\src" `
    (Join-Path $repoRoot "src\calibration_storage.cpp") `
    (Join-Path $repoRoot "src\servo_calibration.cpp") `
    (Join-Path $PSScriptRoot "src\calibration_storage_test.cpp") `
    -o $outputPath
if ($LASTEXITCODE -ne 0) { throw "Calibration storage test failed to compile" }

& $outputPath
if ($LASTEXITCODE -ne 0) { throw "Calibration storage test failed" }
