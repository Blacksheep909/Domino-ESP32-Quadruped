param(
    [string]$IsaacSimRoot = $(if ($env:ISAAC_SIM_ROOT) { $env:ISAAC_SIM_ROOT } else { "C:\isaac-sim" }),
    [string]$IsaacLabRoot = $(if ($env:ISAACLAB_ROOT) { $env:ISAACLAB_ROOT } else { "C:\isaac-projects\IsaacLab" }),
    [int]$NumEnvs = 1,
    [int]$Iterations = 10000,
    [int]$Seed = 42,
    [string]$ResumeCheckpoint = "",
    [switch]$Headless
)

$ErrorActionPreference = "Stop"
$trainer = Join-Path $PSScriptRoot "run-visible-domino-training.ps1"
$referenceGait = Join-Path $PSScriptRoot "config\domino_neutral_diagonal_trot_seed.json"
$identityInitializeActor = [string]::IsNullOrWhiteSpace($ResumeCheckpoint)
$stepDelayS = if ($Headless) { 0.0 } else { 0.005 }

& $trainer `
    -IsaacSimRoot $IsaacSimRoot `
    -IsaacLabRoot $IsaacLabRoot `
    -NumEnvs $NumEnvs `
    -Iterations $Iterations `
    -NumStepsPerEnv 24 `
    -SaveInterval 10 `
    -PolicyValidationSteps 0 `
    -VisibleStepDelayS $stepDelayS `
    -Seed $Seed `
    -InitialNoiseStd 0.10 `
    -PpoLearningRate 0.0001 `
    -PpoEntropyCoefficient 0.0025 `
    -ActionScaleDeg 16.0 `
    -ServoTargetRateLimitDegS 90.0 `
    -ResetSettleSteps 10 `
    -CommandXMps 0.10 `
    -EpisodeLengthS 10.0 `
    -MinHeightM 0.02 `
    -MaxTiltDeg 75.0 `
    -ActualCadGroundClearanceM 0.001 `
    -GroundSizeM 30.0 `
    -AliveRewardScale 0.2 `
    -CommandProgressRewardScale 8.0 `
    -CommandVelocityRewardScale 0.0 `
    -CommandVelocityTrackingRewardScale 4.0 `
    -CommandVelocityTrackingSigma 0.05 `
    -LateralDriftRewardScale -20.0 `
    -YawDriftRewardScale -1.5 `
    -CommandYawRewardScale -1.0 `
    -GaitContactRewardScale 2.0 `
    -StanceContactRewardScale 0.2 `
    -SwingContactPenaltyScale -1.5 `
    -FootClearanceRewardScale 2.0 `
    -FootContactRewardScale 0.0 `
    -ActionRewardScale -0.001 `
    -ActionRateRewardScale -0.005 `
    -ReferenceTrackingRewardScale 2.0 `
    -ReferenceTrackingSigma 0.55 `
    -ReferenceMseRewardScale -2.5 `
    -ReferenceActionIdentityInit $identityInitializeActor `
    -ReferenceActionBcSteps 0 `
    -ReferenceGait $referenceGait `
    -RunName "actual_cad_warmstart_walk" `
    -FootCollisionMode "actual-cad-visual-bottom" `
    -ClosureModel "passive" `
    -ResumeCheckpoint $ResumeCheckpoint `
    -Headless:$Headless `
    -SkipPPOAfterBC:$false `
    -NoHoldOpen

exit $LASTEXITCODE
