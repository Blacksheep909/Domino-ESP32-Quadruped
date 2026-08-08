$ErrorActionPreference = "Stop"

& (Join-Path $PSScriptRoot "stop.ps1")
& (Join-Path $PSScriptRoot "build.ps1")

$runtimeDirectory = Join-Path $PSScriptRoot "viewer\runtime"
New-Item -ItemType Directory -Force -Path $runtimeDirectory | Out-Null
$statePath = Join-Path $runtimeDirectory "state.json"
$simulatorPath = Join-Path $PSScriptRoot "bin\domino_sil.exe"
$viewerPath = Join-Path $PSScriptRoot "viewer"
$platformioPython = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\python.exe"
$python = if (Test-Path -LiteralPath $platformioPython) {
    $platformioPython
} else {
    (Get-Command python -ErrorAction Stop).Source
}

$simulator = Start-Process `
    -FilePath $simulatorPath `
    -ArgumentList @("--realtime", "--loop", "--state-file", $statePath) `
    -WindowStyle Hidden `
    -PassThru

$server = Start-Process `
    -FilePath $python `
    -ArgumentList @("-m", "http.server", "8765", "--directory", $viewerPath) `
    -WindowStyle Hidden `
    -PassThru

Set-Content -LiteralPath (Join-Path $runtimeDirectory "simulator.pid") -Value $simulator.Id
Set-Content -LiteralPath (Join-Path $runtimeDirectory "viewer.pid") -Value $server.Id

Write-Host "Domino SIL simulator PID: $($simulator.Id)"
Write-Host "Domino SIL viewer PID:    $($server.Id)"
Write-Host "Open http://127.0.0.1:8765"
