<#
.SYNOPSIS
  Train (or play) the Domino walking policy in Isaac Lab.

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File simulation/isaac_lab/run-train.ps1 -NumEnvs 4096 -Iterations 3000
  powershell -ExecutionPolicy Bypass -File simulation/isaac_lab/run-train.ps1 -NumEnvs 16 -Iterations 20 -Visible
  powershell -ExecutionPolicy Bypass -File simulation/isaac_lab/run-train.ps1 -Play -Checkpoint simulation/isaac_lab/runs/<run>/model_3000.pt
  powershell -ExecutionPolicy Bypass -File simulation/isaac_lab/run-train.ps1 -Play -Drive -Checkpoint simulation/isaac_lab/runs/<run>/model_3000.pt
#>
param(
    # Python env with `isaacsim` (pip, 6.2) + Isaac Lab 3.0 (see setup-isaac-env.ps1).
    # See simulation/isaac_lab/README.md "Install" for the one-time setup.
    [string]$Python = $(if ($env:DOMINO_PYTHON) { $env:DOMINO_PYTHON } else { "C:\isaac-projects\sim6\Scripts\python.exe" }),
    [int]$NumEnvs = 0,
    [int]$Iterations = 0,
    [int]$Seed = -1,
    [string]$RunName = "",
    [string]$Resume = "",
    [string]$Config = "",
    [switch]$Visible,
    [switch]$Play,
    [string]$Checkpoint = "",
    [switch]$Video,
    # With -Play: drive one robot with the Boxer (USB joystick) or the keyboard.
    [switch]$Drive
)

$ErrorActionPreference = "Stop"
$labRoot = $PSScriptRoot
# Machine-specific settings (git-ignored), e.g. your own acceptance of the
# NVIDIA Omniverse EULA: copy local-settings.example.ps1 to local-settings.ps1.
$localSettings = Join-Path $labRoot "local-settings.ps1"
if (Test-Path -LiteralPath $localSettings) { . $localSettings }
if (-not $PSBoundParameters.ContainsKey("Python") -and $env:DOMINO_PYTHON) { $Python = $env:DOMINO_PYTHON }
$python = $Python
if (-not (Test-Path -LiteralPath $python)) {
    throw "Isaac Lab Python not found at $python. Run simulation/isaac_lab/setup-isaac-env.ps1 or set DOMINO_PYTHON."
}
$env:PYTHONPATH = $labRoot
$env:PYTHONUNBUFFERED = "1"
if ($env:OMNI_KIT_ACCEPT_EULA -ne "YES") {
    Write-Warning ("Isaac Sim needs the NVIDIA Omniverse EULA accepted before it starts headless. Read " +
        "https://docs.omniverse.nvidia.com/platform/latest/common/NVIDIA_Omniverse_License_Agreement.html, then set " +
        "OMNI_KIT_ACCEPT_EULA=YES yourself (see local-settings.example.ps1).")
}

$script = if ($Play) { Join-Path $labRoot "scripts\play.py" } else { Join-Path $labRoot "scripts\train.py" }
$argsList = @($script)
# Playback and driving always open the Isaac Sim window.
if (-not $Visible -and -not $Play) { $argsList += "--headless" }
if ($NumEnvs -gt 0) { $argsList += @("--num_envs", "$NumEnvs") }
if ($Play) {
    if ($Checkpoint) { $argsList += @("--checkpoint", $Checkpoint) }
    if ($Video) { $argsList += @("--video", "--enable_cameras") }
    if ($Drive) { $argsList += "--drive" }
} else {
    if ($Iterations -gt 0) { $argsList += @("--max_iterations", "$Iterations") }
    if ($Seed -ge 0) { $argsList += @("--seed", "$Seed") }
    if ($RunName) { $argsList += @("--run_name", $RunName) }
    if ($Resume) { $argsList += @("--resume", $Resume) }
    if ($Config) { $argsList += @("--config", $Config) }
}

Push-Location $labRoot
try {
    & $python @argsList
    exit $LASTEXITCODE
} finally {
    Pop-Location
}
