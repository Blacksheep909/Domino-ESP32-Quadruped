param(
    [Parameter(Mandatory = $true)]
    [string]$InitialCheckpoint,
    [int]$TargetIteration = 500,
    [int]$ChunkIterations = 25,
    [int]$NumEnvs = 32,
    [string]$IsaacSimRoot = "C:\isaac-sim",
    [string]$IsaacLabRoot = "C:\isaac-projects\IsaacLab"
)

$ErrorActionPreference = "Stop"
$runner = Join-Path $PSScriptRoot "run-visible-domino-training.ps1"
$checkpoint = (Resolve-Path -LiteralPath $InitialCheckpoint).Path
$repoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot "..\..")).Path

function Get-CheckpointIteration([string]$Path) {
    $match = [regex]::Match([IO.Path]::GetFileNameWithoutExtension($Path), '^model_(\d+)$')
    if (-not $match.Success) {
        throw "Checkpoint filename does not contain an iteration: $Path"
    }
    return [int]$match.Groups[1].Value
}

while ((Get-CheckpointIteration $checkpoint) -lt $TargetIteration) {
    $startIteration = Get-CheckpointIteration $checkpoint
    $remaining = $TargetIteration - $startIteration
    $iterations = [Math]::Min($ChunkIterations, $remaining)
    $expectedIteration = $startIteration + $iterations
    $runName = "actual_cad_open_walk_v3_anti_hop_chunk_{0:D3}" -f $expectedIteration
    $logPath = Join-Path $repoRoot "simulation\isaac\out\$runName.log"

    Write-Host "Training Domino from iteration $startIteration to $expectedIteration..."
    & $runner `
        -IsaacSimRoot $IsaacSimRoot `
        -IsaacLabRoot $IsaacLabRoot `
        -NumEnvs $NumEnvs `
        -Iterations $iterations `
        -NumStepsPerEnv 32 `
        -PolicyDevice "cuda:0" `
        -PhysicsDevice "cpu" `
        -SaveInterval $iterations `
        -PolicyValidationSteps 120 `
        -PolicyValidationSettleSteps 30 `
        -PolicyValidationRampSteps 30 `
        -CommandXMps 0.10 `
        -GaitFrequencyHz 1.25 `
        -EpisodeLengthS 20 `
        -MinHeightM 0.18 `
        -MaxTiltDeg 50 `
        -ActionScaleDeg 16 `
        -ServoTargetRateLimitDegS 180 `
        -InitialNoiseStd 0.35 `
        -PpoLearningRate 0.0003 `
        -PpoEntropyCoefficient 0.01 `
        -RunName $runName `
        -ResumeCheckpoint $checkpoint `
        -OpenPolicy `
        -ReferenceActionIdentityInit:$false `
        -ReferenceActionBcSteps 0 `
        -SkipPPOAfterBC:$false `
        -Headless `
        -NoHoldOpen *> $logPath

    $runRoot = Join-Path $repoRoot "simulation\isaac\out\cad_identity\next_policy\$runName"
    $nextCheckpoint = Get-ChildItem -LiteralPath $runRoot -Recurse -Filter "model_*.pt" -ErrorAction SilentlyContinue |
        Sort-Object { Get-CheckpointIteration $_.FullName } -Descending |
        Select-Object -First 1
    if (-not $nextCheckpoint) {
        throw "Chunk produced no checkpoint. Inspect $logPath"
    }
    if ((Get-CheckpointIteration $nextCheckpoint.FullName) -le $startIteration) {
        throw "Chunk made no iteration progress. Inspect $logPath"
    }
    $checkpoint = $nextCheckpoint.FullName
    Write-Host "Saved $checkpoint"
}

Write-Host "Resilient Domino training reached $(Get-CheckpointIteration $checkpoint): $checkpoint"
