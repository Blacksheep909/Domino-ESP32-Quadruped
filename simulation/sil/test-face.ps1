$ErrorActionPreference = "Stop"

# Native checks for the OLED face engine, plus the renderer dump that Studio's
# test compares pixel for pixel (simulation/sil/bin/face_render_dump.json).
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$compiler = Join-Path $env:USERPROFILE ".platformio\packages\toolchain-gccmingw32\bin\g++.exe"
if (-not (Test-Path -LiteralPath $compiler)) {
    throw "Native compiler not found. Install it with: pio pkg install --global --tool platformio/toolchain-gccmingw32"
}
$env:PATH = "$(Split-Path -Parent $compiler);$env:PATH"
$out = Join-Path $PSScriptRoot "bin"
New-Item -ItemType Directory -Force -Path $out | Out-Null

& $compiler -std=c++11 -O1 -Wall -Wextra -static "-I$repoRoot\src" `
    (Join-Path $repoRoot "src\face_engine.cpp") (Join-Path $repoRoot "src\face_renderer.cpp") `
    (Join-Path $PSScriptRoot "src\face_test.cpp") -o (Join-Path $out "domino_face_test.exe")
if ($LASTEXITCODE -ne 0) { throw "Face test failed to compile" }
& (Join-Path $out "domino_face_test.exe")
if ($LASTEXITCODE -ne 0) { throw "Face test failed" }

& $compiler -std=c++11 -O1 -Wall -Wextra -static "-I$repoRoot\src" `
    (Join-Path $repoRoot "src\face_renderer.cpp") (Join-Path $PSScriptRoot "src\face_render_dump.cpp") `
    -o (Join-Path $out "domino_face_render_dump.exe")
if ($LASTEXITCODE -ne 0) { throw "Face renderer dump failed to compile" }
& (Join-Path $out "domino_face_render_dump.exe") | Out-File -Encoding ascii (Join-Path $out "face_render_dump.json")
Write-Host "face renderer dump written"
