param(
    [string]$IsaacSimRoot = $(if ($env:ISAAC_SIM_ROOT) { $env:ISAAC_SIM_ROOT } else { "C:\isaac-sim" }),
    [string]$IsaacLabRoot = $(if ($env:ISAACLAB_ROOT) { $env:ISAACLAB_ROOT } else { "C:\isaac-projects\IsaacLab" }),
    [int]$NumEnvs = 10,
    [int]$GateIterations = 40,
    [int]$TotalIterations = 500,
    [int]$Seed = 73
)

$ErrorActionPreference = "Stop"
if ($GateIterations -lt 1 -or $TotalIterations -le $GateIterations) {
    throw "TotalIterations must be greater than GateIterations, and GateIterations must be positive."
}

$trainer = Join-Path $PSScriptRoot "run-visible-domino-training.ps1"
$referenceGait = Join-Path $PSScriptRoot "config\domino_neutral_diagonal_trot_seed.json"
$gateRunName = "actual_cad_velocity_walk_v2_gate"
$finalRunName = "actual_cad_velocity_walk_v2"
$nextPolicyRoot = Join-Path $PSScriptRoot "out\cad_identity\next_policy"
$gateReportPath = Join-Path $nextPolicyRoot ($gateRunName + ".json")
$finalReportPath = Join-Path $nextPolicyRoot ($finalRunName + ".json")

$common = @{
    IsaacSimRoot = $IsaacSimRoot
    IsaacLabRoot = $IsaacLabRoot
    NumEnvs = $NumEnvs
    NumStepsPerEnv = 24
    SaveInterval = 10
    PolicyValidationSettleSteps = 10
    PolicyValidationRampSteps = 0
    VisibleStepDelayS = 0.0
    Seed = $Seed
    InitialNoiseStd = 0.18
    PpoLearningRate = 0.0002
    PpoEntropyCoefficient = 0.005
    ActionScaleDeg = 16.0
    ServoTargetRateLimitDegS = 100.0
    ResetSettleSteps = 10
    CommandXMps = 0.12
    GaitFrequencyHz = 1.25
    EpisodeLengthS = 10.0
    MinHeightM = 0.02
    MaxTiltDeg = 60.0
    ActualCadGroundClearanceM = 0.001
    GroundSizeM = 30.0
    AliveRewardScale = 0.0
    CommandProgressRewardScale = 20.0
    CommandVelocityRewardScale = -1.0
    CommandVelocityTrackingRewardScale = 10.0
    CommandVelocityTrackingSigma = 0.06
    CommandStagnationPenaltyScale = -3.0
    CommandStagnationSpeedMps = 0.03
    LateralDriftRewardScale = -3.0
    YawDriftRewardScale = -0.5
    CommandYawRewardScale = -0.5
    GaitContactRewardScale = 0.5
    StanceContactRewardScale = 0.05
    SwingContactPenaltyScale = -0.25
    FootClearanceRewardScale = 0.5
    FootContactRewardScale = 0.0
    ActionRewardScale = -0.0001
    ActionRateRewardScale = -0.002
    ReferenceTrackingRewardScale = 0.25
    ReferenceTrackingSigma = 0.55
    ReferenceMseRewardScale = -0.1
    ReferenceActionBcSteps = 0
    ReferenceGait = $referenceGait
    TerrainType = "flat"
    FootCollisionMode = "actual-cad-visual-bottom"
    ClosureModel = "passive"
    Headless = $true
    NoHoldOpen = $true
    SkipPPOAfterBC = $false
}

Write-Host "Stage 1: velocity-learning gate"
Write-Host "A stationary policy now receives a -3.0 stagnation term and no survival reward."
$stageOne = $common.Clone()
$stageOne.Iterations = $GateIterations
$stageOne.PolicyValidationSteps = 200
$stageOne.PolicyGateMinForwardM = 0.05
$stageOne.PolicyGateMaxLateralM = 0.25
$stageOne.PolicyGateMaxYawRad = 0.80
$stageOne.PolicyGateMaxTiltDeg = 60.0
$stageOne.PolicyGateMaxSwingContact = 0.90
$stageOne.PolicyGateMinSwingClearanceM = 0.001
$stageOne.PolicyGateMinEachCadFootClearanceM = 0.002
$stageOne.PolicyGateMinFootMotionM = 0.015
$stageOne.PolicyGateMinEachLinkageDriveMotionDeg = 1.0
$stageOne.ReferenceActionIdentityInit = $true
$stageOne.RunName = $gateRunName

& $trainer @stageOne
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

$gateReport = Get-Content -Raw -LiteralPath $gateReportPath | ConvertFrom-Json
if ($gateReport.status -ne "passed" -or -not $gateReport.trained_policy_validation.passed) {
    Write-Error (
        "Stage-one locomotion gate failed; the 460-iteration continuation was not started. Failures: {0}" -f
        ($gateReport.trained_policy_validation.failures -join "; ")
    )
    exit 2
}

$gateCheckpoint = Join-Path $nextPolicyRoot (
    "{0}\domino_cad_linkage_direct\{1}\{2}" -f
        $gateRunName,
        $gateReport.log_dir_name,
        $gateReport.latest_checkpoint
)
if (-not (Test-Path -LiteralPath $gateCheckpoint)) {
    throw "Stage-one gate passed but its checkpoint was not found: $gateCheckpoint"
}

$remainingIterations = $TotalIterations - $GateIterations
Write-Host "Stage 1 passed. Continuing the same optimizer for $remainingIterations iterations."
$stageTwo = $common.Clone()
$stageTwo.Iterations = $remainingIterations
$stageTwo.PolicyValidationSteps = 240
$stageTwo.PolicyGateMinForwardM = 0.18
$stageTwo.PolicyGateMaxLateralM = 0.20
$stageTwo.PolicyGateMaxYawRad = 0.60
$stageTwo.PolicyGateMaxTiltDeg = 50.0
$stageTwo.PolicyGateMaxSwingContact = 0.80
$stageTwo.PolicyGateMinSwingClearanceM = 0.002
$stageTwo.PolicyGateMinEachCadFootClearanceM = 0.003
$stageTwo.PolicyGateMinFootMotionM = 0.030
$stageTwo.PolicyGateMinEachLinkageDriveMotionDeg = 2.0
$stageTwo.ReferenceActionIdentityInit = $false
$stageTwo.ResumeCheckpoint = $gateCheckpoint
$stageTwo.ResumeLoadOptimizer = $true
$stageTwo.RunName = $finalRunName

& $trainer @stageTwo
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

$finalReport = Get-Content -Raw -LiteralPath $finalReportPath | ConvertFrom-Json
if ($finalReport.status -ne "passed" -or -not $finalReport.trained_policy_validation.passed) {
    Write-Error (
        "The final policy did not pass the walking gate. Failures: {0}" -f
        ($finalReport.trained_policy_validation.failures -join "; ")
    )
    exit 3
}

Write-Host "Domino velocity policy passed the final walking gate."
exit 0
