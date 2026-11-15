$ErrorActionPreference = "Stop"

# Native checks for the emote & sound settings record and the emote timelines.
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$compiler = Join-Path $env:USERPROFILE ".platformio\packages\toolchain-gccmingw32\bin\g++.exe"
if (-not (Test-Path -LiteralPath $compiler)) {
    throw "Native compiler not found. Install it with: pio pkg install --global --tool platformio/toolchain-gccmingw32"
}
$env:PATH = "$(Split-Path -Parent $compiler);$env:PATH"

$outputDirectory = Join-Path $PSScriptRoot "bin"
New-Item -ItemType Directory -Force -Path $outputDirectory | Out-Null
$outputPath = Join-Path $outputDirectory "domino_personality_test.exe"

& $compiler -std=c++11 -O1 -Wall -Wextra -static -DDOMINO_PERSONALITY_PURE "-I$repoRoot\src" `
    (Join-Path $repoRoot "src\personality.cpp") `
    (Join-Path $PSScriptRoot "src\personality_test.cpp") `
    -o $outputPath
if ($LASTEXITCODE -ne 0) { throw "Personality test failed to compile" }

& $outputPath
if ($LASTEXITCODE -ne 0) { throw "Personality test failed" }
