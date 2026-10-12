param(
    [Parameter(Mandatory = $true)]
    [string]$InitialCheckpoint,
    [int]$TargetIteration = 600,
    [int]$ChunkIterations = 100,
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
    $remainingIterations = $TargetIteration - $startIteration
    $iterations = [Math]::Min($ChunkIterations, $remainingIterations + 1)
    $expectedIteration = $startIteration + $iterations - 1
    $runName = "actual_cad_open_walk_v6_front_guard_chunk_{0:D3}" -f $expectedIteration
    $logPath = Join-Path $repoRoot "simulation\isaac\out\$runName.log"

    Write-Host "Continuing Domino front-guard training from iteration $startIteration to $expectedIteration..."
    & $runner `
        -IsaacSimRoot $IsaacSimRoot `
        -IsaacLabRoot $IsaacLabRoot `
        -NumEnvs $NumEnvs `
        -Iterations $iterations `
        -NumStepsPerEnv 32 `
        -PolicyDevice "cuda:0" `
        -PhysicsDevice "cpu" `
        -SaveInterval $iterations `
        -PolicyValidationSteps 0 `
        -Seed 9306 `
        -InitialNoiseStd 0.12 `
        -PpoLearningRate 0.00005 `
        -PpoEntropyCoefficient 0.002 `
        -ActionScaleDeg 16 `
        -ServoTargetRateLimitDegS 180 `
        -ResetSettleSteps 15 `
        -CommandXMps 0.10 `
        -GaitFrequencyHz 1.25 `
        -EpisodeLengthS 20 `
        -MinHeightM 0.18 `
        -MaxTiltDeg 35 `
        -GroundSizeM 20 `
        -AliveRewardScale 0.2 `
        -VerticalVelocityRewardScale -0.5 `
        -AngularVelocityRewardScale -0.25 `
        -FlatOrientationRewardScale -8.0 `
        -PitchOrientationRewardScale -12.0 `
        -CommandProgressRewardScale 10.0 `
        -CommandVelocityRewardScale -4.0 `
        -CommandVelocityTrackingRewardScale 12.0 `
        -CommandVelocityTrackingSigma 0.04 `
        -CommandStagnationPenaltyScale -8.0 `
        -LateralDriftRewardScale -10.0 `
        -YawDriftRewardScale -2.0 `
        -CommandYawRewardScale -1.0 `
        -GaitContactRewardScale 5.0 `
        -StanceContactRewardScale 1.0 `
        -SwingContactPenaltyScale -6.0 `
        -FootClearanceRewardScale 4.0 `
        -FootSlipRewardScale -0.5 `
        -AirTimeVarianceRewardScale -1.0 `
        -ValidFootCycleRewardScale 6.0 `
        -FrontRearSupportRewardScale 3.0 `
        -AxleSupportImbalancePenaltyScale -4.0 `
        -SameAxleAirbornePenaltyScale -6.0 `
        -ExcessAirbornePenaltyScale -4.0 `
        -FrontFootBackwardReachPenaltyScale -8.0 `
        -FrontPairBackwardReachPenaltyScale -12.0 `
        -FrontFootMinBodyXM 0.20 `
        -FrontFootReachNormalizationM 0.10 `
        -FrontFootBackwardTerminationBodyXM 0.10 `
        -ActionRewardScale -0.01 `
        -ActionRateRewardScale -0.03 `
        -ReferenceTrackingRewardScale 0.25 `
        -ReferenceTrackingSigma 0.45 `
        -ReferenceMseRewardScale -0.10 `
        -ReferenceActionBcSteps 0 `
        -RunName $runName `
        -ResumeCheckpoint $checkpoint `
        -OpenPolicy `
        -OpenPolicyReferencePrior `
        -ReferenceActionIdentityInit:$false `
        -SkipPPOAfterBC:$false `
        -Headless `
        -NoHoldOpen *> $logPath

    $runRoot = Join-Path $repoRoot "simulation\isaac\out\cad_identity\next_policy\$runName"
    $nextCheckpoint = Get-ChildItem -LiteralPath $runRoot -Recurse -Filter "model_*.pt" -ErrorAction SilentlyContinue |
        Where-Object { $_.BaseName -match '^model_\d+$' } |
        Sort-Object { Get-CheckpointIteration $_.FullName } -Descending |
        Select-Object -First 1
    if (-not $nextCheckpoint) {
        throw "Front-guard recovery produced no checkpoint. Inspect $logPath"
    }
    if ((Get-CheckpointIteration $nextCheckpoint.FullName) -le $startIteration) {
        throw "Front-guard recovery made no iteration progress. Inspect $logPath"
    }
    $checkpoint = $nextCheckpoint.FullName
    Write-Host "Saved $checkpoint"
}

Write-Host "Domino front-guard recovery reached $(Get-CheckpointIteration $checkpoint): $checkpoint"
