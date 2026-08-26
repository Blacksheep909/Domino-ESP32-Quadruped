param(
    [int]$Iterations = 500,
    [int]$NumEnvs = 16,
    [int]$Seed = 9306,
    [string]$IsaacSimRoot = "C:\isaac-sim",
    [string]$IsaacLabRoot = "C:\isaac-projects\IsaacLab",
    [switch]$Visible
)

$ErrorActionPreference = "Stop"
$runner = Join-Path $PSScriptRoot "run-visible-domino-training.ps1"
$runName = "actual_cad_open_walk_v7_fresh_{0}env_seed{1}" -f $NumEnvs, $Seed

Write-Host "Starting a fresh Domino walk policy with no resume checkpoint."
Write-Host "Run: $runName"
Write-Host "Policy device: CUDA; physics device: CPU (the CAD/USD synchronization path is faster on CPU)."

$optionalFlags = @()
if (-not $Visible) {
    $optionalFlags = @("-Headless", "-NoHoldOpen")
}

& $runner `
    -IsaacSimRoot $IsaacSimRoot `
    -IsaacLabRoot $IsaacLabRoot `
    -NumEnvs $NumEnvs `
    -Iterations $Iterations `
    -NumStepsPerEnv 32 `
    -PolicyDevice "cuda:0" `
    -PhysicsDevice "cpu" `
    -SaveInterval 25 `
    -PolicyValidationSteps 0 `
    -PolicyValidationSettleSteps 0 `
    -PolicyValidationRampSteps 0 `
    -Seed $Seed `
    -InitialNoiseStd 0.18 `
    -PpoLearningRate 0.00008 `
    -PpoEntropyCoefficient 0.003 `
    -ActionScaleDeg 16 `
    -ServoTargetRateLimitDegS 180 `
    -ResetSettleSteps 15 `
    -CommandXMps 0.10 `
    -GaitFrequencyHz 1.25 `
    -EpisodeLengthS 20 `
    -MinHeightM 0.18 `
    -MaxTiltDeg 35 `
    -GroundSizeM 16 `
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
    -ReferenceActionBcSteps 0 `
    -ReferenceTrackingRewardScale 0.0 `
    -ReferenceMseRewardScale 0.0 `
    -RunName $runName `
    -OpenPolicy `
    -OpenPolicyReferencePrior `
    -ReferenceActionIdentityInit:$false `
    -SkipPPOAfterBC:$false `
    @optionalFlags

exit $LASTEXITCODE
