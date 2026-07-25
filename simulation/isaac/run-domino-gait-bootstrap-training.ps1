param(
    [string]$IsaacSimRoot = "C:\isaac-sim",
    [string]$IsaacLabRoot = "C:\isaac-projects\IsaacLab",
    [int]$NumEnvs = 10,
    [int]$Iterations = 1000000,
    [string]$ResumeCheckpoint = "",
    [switch]$Visible
)

$ErrorActionPreference = "Stop"

$trainingLauncher = Join-Path $PSScriptRoot "run-visible-domino-training.ps1"
$referenceGait = Join-Path $PSScriptRoot "out\cad_identity\teacher_grid\teacher_calibrated_neutral_all_foot_best.json"
$startingFresh = -not $ResumeCheckpoint -or $ResumeCheckpoint.Trim().Length -eq 0

& $trainingLauncher `
    -IsaacSimRoot $IsaacSimRoot `
    -IsaacLabRoot $IsaacLabRoot `
    -NumEnvs $NumEnvs `
    -Iterations $Iterations `
    -NumStepsPerEnv 64 `
    -SaveInterval 25 `
    -PolicyValidationSteps 0 `
    -VisibleStepDelayS 0 `
    -InitialNoiseStd 0.10 `
    -PpoLearningRate 0.00002 `
    -PpoEntropyCoefficient 0.0005 `
    -ActionScaleDeg 4 `
    -ServoTargetRateLimitDegS 90 `
    -ResetSettleSteps 2 `
    -CommandXMps 0.04 `
    -GaitFrequencyHz 1 `
    -EpisodeLengthS 6 `
    -MinHeightM 0.08 `
    -MaxTiltDeg 45 `
    -ActualCadGroundClearanceM 0.001 `
    -GroundSizeM 50 `
    -AliveRewardScale 0.20 `
    -CommandProgressRewardScale 80 `
    -CommandVelocityRewardScale -10 `
    -CommandVelocityTrackingRewardScale 3 `
    -CommandVelocityTrackingSigma 0.015 `
    -LateralDriftRewardScale -10 `
    -YawDriftRewardScale -1 `
    -CommandYawRewardScale -0.5 `
    -GaitContactRewardScale 0.50 `
    -StanceContactRewardScale 0.20 `
    -SwingContactPenaltyScale -0.50 `
    -FootClearanceRewardScale 5 `
    -FootContactRewardScale 0 `
    -ActionRewardScale 0 `
    -ActionRateRewardScale -0.002 `
    -ReferenceTrackingRewardScale 2 `
    -ReferenceTrackingSigma 0.55 `
    -ReferenceMseRewardScale -1 `
    -ReferenceActionIdentityInit:$startingFresh `
    -ReferenceActionBcSteps 0 `
    -ResumeCheckpoint $ResumeCheckpoint `
    -RunName "domino_gait_anchored_forward_walk" `
    -ReferenceGait $referenceGait `
    -TerrainType flat `
    -FootCollisionMode actual-cad-visual-bottom `
    -ClosureModel passive `
    -NoHoldOpen `
    -Headless:(-not $Visible) `
    -SkipPPOAfterBC:$false

exit $LASTEXITCODE
