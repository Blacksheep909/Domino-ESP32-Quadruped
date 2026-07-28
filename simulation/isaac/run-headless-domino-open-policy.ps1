param(
    [string]$IsaacSimRoot = $(if ($env:ISAAC_SIM_ROOT) { $env:ISAAC_SIM_ROOT } else { "C:\isaac-sim" }),
    [string]$IsaacLabRoot = $(if ($env:ISAACLAB_ROOT) { $env:ISAACLAB_ROOT } else { "C:\isaac-projects\IsaacLab" }),
    [int]$NumEnvs = 32,
    [int]$Iterations = 500,
    [ValidateSet("cpu", "cuda:0")]
    [string]$PolicyDevice = "cuda:0",
    [ValidateSet("same", "cpu", "cuda:0")]
    [string]$PhysicsDevice = "cpu",
    [int]$Seed = 280728,
    [int]$PolicyValidationSteps = 300,
    [string]$RunName = "actual_cad_open_walk_v1"
)

$ErrorActionPreference = "Stop"
$trainer = Join-Path $PSScriptRoot "run-visible-domino-training.ps1"

Write-Host "Domino open-policy PPO training"
Write-Host ("Environments: {0}; iterations: {1}; seed: {2}" -f $NumEnvs, $Iterations, $Seed)
Write-Host "Checkpoint: none; reference observation: none; imitation reward: none."

& $trainer `
    -IsaacSimRoot $IsaacSimRoot `
    -IsaacLabRoot $IsaacLabRoot `
    -NumEnvs $NumEnvs `
    -Iterations $Iterations `
    -PolicyDevice $PolicyDevice `
    -PhysicsDevice $PhysicsDevice `
    -NumStepsPerEnv 24 `
    -SaveInterval 10 `
    -PolicyValidationSteps $PolicyValidationSteps `
    -PolicyValidationSettleSteps 120 `
    -PolicyValidationRampSteps 60 `
    -PolicyGateMinForwardM 0.03 `
    -PolicyGateMaxLateralM 0.12 `
    -PolicyGateMaxYawRad 0.70 `
    -PolicyGateMaxTiltDeg 30.0 `
    -PolicyGateMaxSwingContact 0.80 `
    -PolicyGateMinGaitContactMatch 0.48 `
    -PolicyGateMinSwingClearanceM 0.002 `
    -PolicyGateMinEachCadFootClearanceM 0.004 `
    -PolicyGateMinFootMotionM 0.045 `
    -PolicyGateMinEachLinkageDriveMotionDeg 3.0 `
    -Seed $Seed `
    -InitialNoiseStd 0.35 `
    -PpoLearningRate 0.0003 `
    -PpoEntropyCoefficient 0.01 `
    -ActionScaleDeg 16.0 `
    -ServoTargetRateLimitDegS 180.0 `
    -ResetSettleSteps 15 `
    -CommandXMps 0.10 `
    -GaitFrequencyHz 1.25 `
    -EpisodeLengthS 10.0 `
    -MinHeightM 0.18 `
    -MaxTiltDeg 50.0 `
    -ActualCadGroundClearanceM 0.001 `
    -GroundSizeM 30.0 `
    -AliveRewardScale 0.0 `
    -CommandProgressRewardScale 20.0 `
    -CommandVelocityRewardScale -2.0 `
    -CommandVelocityTrackingRewardScale 10.0 `
    -CommandVelocityTrackingSigma 0.06 `
    -CommandStagnationPenaltyScale -5.0 `
    -CommandStagnationSpeedMps 0.03 `
    -LateralDriftRewardScale -3.0 `
    -YawDriftRewardScale -0.5 `
    -CommandYawRewardScale -0.5 `
    -GaitContactRewardScale 4.0 `
    -StanceContactRewardScale 0.5 `
    -SwingContactPenaltyScale -5.0 `
    -FootClearanceRewardScale 4.0 `
    -FootContactRewardScale 0.0 `
    -FootSlipRewardScale -0.5 `
    -AirTimeVarianceRewardScale -1.0 `
    -ValidFootCycleRewardScale 1.0 `
    -FootCycleMinAirTimeS 0.06 `
    -FootCycleTargetAirTimeS 0.20 `
    -FootCycleMinClearanceM 0.004 `
    -FootCycleMinBodyRelativeTravelM 0.015 `
    -ActionRewardScale -0.0001 `
    -ActionRateRewardScale -0.002 `
    -ReferenceTrackingRewardScale 0.0 `
    -ReferenceMseRewardScale 0.0 `
    -ReferenceActionIdentityInit:$false `
    -ReferenceActionBcSteps 0 `
    -RunName $RunName `
    -FootCollisionMode "actual-cad-visual-bottom" `
    -ClosureModel "passive" `
    -OpenPolicy `
    -Headless `
    -SkipPPOAfterBC:$false `
    -NoHoldOpen

exit $LASTEXITCODE
