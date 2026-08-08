$ErrorActionPreference = "Stop"

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$silRoot = Join-Path $repoRoot "simulation\sil"
$runtimeDirectory = Join-Path $PSScriptRoot "runtime"
$codexNode = Join-Path $env:USERPROFILE ".cache\codex-runtimes\codex-primary-runtime\dependencies\node\bin\node.exe"
$node = if (Test-Path -LiteralPath $codexNode) {
    $codexNode
} else {
    (Get-Command node -ErrorAction Stop).Source
}

& (Join-Path $PSScriptRoot "stop.ps1")
& (Join-Path $silRoot "stop.ps1")
& (Join-Path $silRoot "build.ps1")

Push-Location $PSScriptRoot
try {
    & $node `
        (Join-Path $PSScriptRoot "node_modules\vite\bin\vite.js") `
        build `
        --config (Join-Path $PSScriptRoot "vite.config.js")
    if ($LASTEXITCODE -ne 0) {
        throw "Standalone renderer build failed with exit code $LASTEXITCODE"
    }
} finally {
    Pop-Location
}

New-Item -ItemType Directory -Force -Path $runtimeDirectory | Out-Null
$controlPath = Join-Path $runtimeDirectory "controls.txt"
$statePath = Join-Path $runtimeDirectory "state.json"
$firmwarePath = Join-Path $silRoot "bin\domino_sil.exe"
$serverOutputPath = Join-Path $runtimeDirectory "server-output.log"
$serverErrorPath = Join-Path $runtimeDirectory "server-error.log"
$firmwareOutputPath = Join-Path $runtimeDirectory "firmware-output.log"
$firmwareErrorPath = Join-Path $runtimeDirectory "firmware-error.log"

$server = Start-Process `
    -FilePath $node `
    -ArgumentList @((Join-Path $PSScriptRoot "server.mjs")) `
    -WorkingDirectory $PSScriptRoot `
    -WindowStyle Hidden `
    -RedirectStandardOutput $serverOutputPath `
    -RedirectStandardError $serverErrorPath `
    -PassThru

Start-Sleep -Milliseconds 500

$firmware = Start-Process `
    -FilePath $firmwarePath `
    -ArgumentList @(
        "--realtime",
        "--loop",
        "--control-file", $controlPath,
        "--state-file", $statePath
    ) `
    -WindowStyle Hidden `
    -RedirectStandardOutput $firmwareOutputPath `
    -RedirectStandardError $firmwareErrorPath `
    -PassThru

Set-Content -LiteralPath (Join-Path $runtimeDirectory "server.pid") -Value $server.Id
Set-Content -LiteralPath (Join-Path $runtimeDirectory "firmware.pid") -Value $firmware.Id

Write-Host "Domino standalone server PID:   $($server.Id)"
Write-Host "Domino firmware SIL PID:        $($firmware.Id)"
Write-Host "Open http://127.0.0.1:8770"
