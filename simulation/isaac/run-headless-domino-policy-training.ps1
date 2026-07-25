param(
    [string]$IsaacSimRoot = $(if ($env:ISAAC_SIM_ROOT) { $env:ISAAC_SIM_ROOT } else { "C:\isaac-sim" }),
    [string]$IsaacLabRoot = $(if ($env:ISAACLAB_ROOT) { $env:ISAACLAB_ROOT } else { "C:\isaac-projects\IsaacLab" }),
    [int]$NumEnvs = 10,
    [int]$Iterations = 500,
    [int]$Seed = 42,
    [string]$ResumeCheckpoint = "",
    [switch]$Fresh
)

$ErrorActionPreference = "Stop"
$launcher = Join-Path $PSScriptRoot "run-visible-domino-actual-cad-learning.ps1"
$checkpointRoot = Join-Path $PSScriptRoot "out\cad_identity\next_policy\actual_cad_warmstart_walk"

if ($Fresh -and -not [string]::IsNullOrWhiteSpace($ResumeCheckpoint)) {
    throw "Use either -Fresh or -ResumeCheckpoint, not both."
}

if (-not $Fresh -and [string]::IsNullOrWhiteSpace($ResumeCheckpoint) -and (Test-Path -LiteralPath $checkpointRoot)) {
    $latestCheckpoint = Get-ChildItem -LiteralPath $checkpointRoot -Filter "model_*.pt" -File -Recurse |
        Sort-Object LastWriteTimeUtc |
        Select-Object -Last 1
    if ($latestCheckpoint) {
        $ResumeCheckpoint = $latestCheckpoint.FullName
    }
}

Write-Host "Domino actual-CAD PPO training"
Write-Host ("Environments: {0}; iterations: {1}; seed: {2}" -f $NumEnvs, $Iterations, $Seed)
if ([string]::IsNullOrWhiteSpace($ResumeCheckpoint)) {
    Write-Host "Checkpoint: none; initializing the actor from the tracked diagonal-trot reference."
} else {
    Write-Host ("Checkpoint: {0}" -f $ResumeCheckpoint)
}

& $launcher `
    -IsaacSimRoot $IsaacSimRoot `
    -IsaacLabRoot $IsaacLabRoot `
    -NumEnvs $NumEnvs `
    -Iterations $Iterations `
    -Seed $Seed `
    -ResumeCheckpoint $ResumeCheckpoint `
    -Headless

exit $LASTEXITCODE
