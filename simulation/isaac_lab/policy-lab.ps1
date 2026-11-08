<#
.SYNOPSIS
  One entry point for watching and launching Domino policy training.

.EXAMPLE
  .\policy-lab.ps1                 # open Studio's Policy Lab page in your browser (default)
  .\policy-lab.ps1 watch           # short training with the Isaac Sim window open (16 robots)
  .\policy-lab.ps1 train           # full headless run (4096 robots, 3000 iterations)
  .\policy-lab.ps1 play            # watch the newest checkpoint walk in Isaac Sim
  .\policy-lab.ps1 drive           # drive the newest checkpoint with the Boxer (USB) or keyboard
  .\policy-lab.ps1 tensorboard     # every training curve in TensorBoard
  .\policy-lab.ps1 stand           # sanity check: hold the neutral stand and compare with firmware
#>
param(
    [ValidateSet("studio", "watch", "train", "play", "drive", "tensorboard", "stand")]
    [string]$Command = "studio",
    [int]$NumEnvs = 0,
    [int]$Iterations = 0,
    [string]$Checkpoint = ""
)
$ErrorActionPreference = "Stop"
$lab = $PSScriptRoot
$localSettings = Join-Path $lab "local-settings.ps1"
if (Test-Path -LiteralPath $localSettings) { . $localSettings }
$python = if ($env:DOMINO_PYTHON) { $env:DOMINO_PYTHON } else { "C:\isaac-projects\sim6\Scripts\python.exe" }
$train = Join-Path $lab "run-train.ps1"

function Latest-Checkpoint {
    $latest = Get-ChildItem (Join-Path $lab "runs") -Recurse -Filter "model_*.pt" -ErrorAction SilentlyContinue |
        Sort-Object LastWriteTime -Descending | Select-Object -First 1
    if (-not $latest) { throw "No checkpoints yet. Train first: .\policy-lab.ps1 watch" }
    return $latest.FullName
}

switch ($Command) {
    "studio" {
        $standalone = Join-Path (Split-Path (Split-Path $lab)) "simulation\standalone"
        $port = 8791
        if (-not (Test-Path (Join-Path $standalone "dist\index.html"))) {
            Push-Location $standalone; npm run build; Pop-Location
        }
        $env:DOMINO_STANDALONE_PORT = "$port"
        Start-Process -FilePath "node" -ArgumentList "server.mjs" -WorkingDirectory $standalone -WindowStyle Minimized
        Start-Sleep -Seconds 2
        Start-Process "http://127.0.0.1:$port/#policy-lab"
        Write-Host "Studio Policy Lab: http://127.0.0.1:$port/#policy-lab  (close the minimized node window to stop it)"
    }
    "watch" {
        $n = if ($NumEnvs) { $NumEnvs } else { 16 }
        $it = if ($Iterations) { $Iterations } else { 300 }
        & $train -Visible -NumEnvs $n -Iterations $it -RunName "watch"
    }
    "train" {
        $n = if ($NumEnvs) { $NumEnvs } else { 4096 }
        $it = if ($Iterations) { $Iterations } else { 3000 }
        & $train -NumEnvs $n -Iterations $it
    }
    "play" {
        $ckpt = if ($Checkpoint) { $Checkpoint } else { Latest-Checkpoint }
        Write-Host "Playing $ckpt"
        & $train -Play -Checkpoint $ckpt -NumEnvs $(if ($NumEnvs) { $NumEnvs } else { 16 })
    }
    "drive" {
        $ckpt = if ($Checkpoint) { $Checkpoint } else { Latest-Checkpoint }
        Write-Host "Driving $ckpt  (Boxer in USB Joystick mode, or keyboard in the Isaac window)"
        & $train -Play -Drive -Checkpoint $ckpt -NumEnvs $(if ($NumEnvs) { $NumEnvs } else { 1 })
    }
    "tensorboard" {
        Start-Process "http://127.0.0.1:6006"
        & $python -m tensorboard.main --logdir (Join-Path $lab "runs") --port 6006
    }
    "stand" {
        $env:PYTHONPATH = $lab
        & $python (Join-Path $lab "scripts\stand_check.py") --no_dr
    }
}
