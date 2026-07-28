param(
    [string]$IsaacSimRoot = $(if ($env:ISAAC_SIM_ROOT) { $env:ISAAC_SIM_ROOT } else { "C:\isaac-sim" }),
    [string]$IsaacLabRoot = $(if ($env:ISAACLAB_ROOT) { $env:ISAACLAB_ROOT } else { "C:\isaac-projects\IsaacLab" }),
    [int]$NumEnvs = 32,
    [int]$GateIterations = 40,
    [int]$CorrectionIterations = 80,
    [int]$TotalIterations = 500,
    [int]$Seed = 73,
    [string]$BootstrapCheckpoint = ""
)

$ErrorActionPreference = "Stop"
if ($GateIterations -lt 1 -or $CorrectionIterations -lt 1 -or $TotalIterations -le ($GateIterations + $CorrectionIterations)) {
    throw "TotalIterations must exceed GateIterations plus CorrectionIterations, and both stages must be positive."
}

$trainer = Join-Path $PSScriptRoot "run-visible-domino-training.ps1"
$referenceGait = Join-Path $PSScriptRoot "config\domino_neutral_diagonal_trot_seed.json"
$gateRunName = "actual_cad_velocity_walk_v2_gate"
$correctionRunName = "actual_cad_velocity_walk_v3_correction"
$finalRunName = "actual_cad_velocity_walk_v2"
$nextPolicyRoot = Join-Path $PSScriptRoot "out\cad_identity\next_policy"
$gateReportPath = Join-Path $nextPolicyRoot ($gateRunName + ".json")
$correctionReportPath = Join-Path $nextPolicyRoot ($correctionRunName + ".json")
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
    FootSlipRewardScale = -0.20
    AirTimeVarianceRewardScale = -0.25
    ValidFootCycleRewardScale = 0.25
    FootCycleMinAirTimeS = 0.06
    FootCycleTargetAirTimeS = 0.20
    FootCycleMinClearanceM = 0.004
    FootCycleMinBodyRelativeTravelM = 0.015
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

$gateCheckpoint = ""
if ($BootstrapCheckpoint -and $BootstrapCheckpoint.Trim().Length -gt 0) {
    $gateCheckpoint = (Resolve-Path -LiteralPath $BootstrapCheckpoint).Path
    Write-Host "Using supplied motor-exploration checkpoint: $gateCheckpoint"
} else {
    Write-Host "Stage 1: coordinated motor-exploration bootstrap"
    Write-Host "This stage teaches active linkage motion; direction is corrected in stage 2."
    $stageOne = $common.Clone()
    $stageOne.Iterations = $GateIterations
    $stageOne.PolicyValidationSteps = 200
    $stageOne.PolicyGateMinForwardM = -0.20
    $stageOne.PolicyGateMaxLateralM = 0.25
    $stageOne.PolicyGateMaxYawRad = 0.80
    $stageOne.PolicyGateMaxTiltDeg = 60.0
    $stageOne.PolicyGateMaxSwingContact = 0.95
    $stageOne.PolicyGateMinSwingClearanceM = 0.001
    $stageOne.PolicyGateMinEachCadFootClearanceM = 0.0
    $stageOne.PolicyGateMinFootMotionM = 0.015
    $stageOne.PolicyGateMinEachLinkageDriveMotionDeg = 1.0
    $stageOne.PolicyGateMinValidCyclesPerFoot = 0
    $stageOne.PolicyGateMinValidCycleRatio = 0.0
    $stageOne.PolicyGateMaxFootCycleDominationRatio = 1.0
    $stageOne.ReferenceActionIdentityInit = $true
    $stageOne.RunName = $gateRunName

    & $trainer @stageOne
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }

    $gateReport = Get-Content -Raw -LiteralPath $gateReportPath | ConvertFrom-Json
    if ($gateReport.status -ne "passed" -or -not $gateReport.trained_policy_validation.passed) {
        Write-Error (
            "Stage-one motor gate failed; direction training was not started. Failures: {0}" -f
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
}
if (-not (Test-Path -LiteralPath $gateCheckpoint)) {
    throw "Motor-exploration checkpoint was not found: $gateCheckpoint"
}

Write-Host "Stage 2: remove teacher reward and correct travel direction"
$stageTwo = $common.Clone()
$stageTwo.Iterations = $CorrectionIterations
$stageTwo.PolicyValidationSteps = 200
$stageTwo.PolicyGateMinForwardM = 0.05
$stageTwo.PolicyGateMaxLateralM = 0.25
$stageTwo.PolicyGateMaxYawRad = 0.80
$stageTwo.PolicyGateMaxTiltDeg = 60.0
$stageTwo.PolicyGateMaxSwingContact = 0.55
$stageTwo.PolicyGateMinGaitContactMatch = 0.65
$stageTwo.PolicyGateMinSwingClearanceM = 0.003
$stageTwo.PolicyGateMinEachCadFootClearanceM = 0.004
$stageTwo.PolicyGateMinFootMotionM = 0.020
$stageTwo.PolicyGateMinEachLinkageDriveMotionDeg = 1.5
$stageTwo.PolicyGateMinValidCyclesPerFoot = 1
$stageTwo.PolicyGateMinValidCycleRatio = 0.30
$stageTwo.PolicyGateMaxFootCycleDominationRatio = 0.70
$stageTwo.GaitCycleMaxTiltDeg = 45.0
$stageTwo.CommandProgressRewardScale = 40.0
$stageTwo.CommandVelocityRewardScale = -2.0
$stageTwo.CommandVelocityTrackingRewardScale = 15.0
$stageTwo.CommandStagnationPenaltyScale = -5.0
$stageTwo.GaitContactRewardScale = 4.0
$stageTwo.StanceContactRewardScale = 0.5
$stageTwo.SwingContactPenaltyScale = -5.0
$stageTwo.FootClearanceRewardScale = 4.0
$stageTwo.FootSlipRewardScale = -0.50
$stageTwo.AirTimeVarianceRewardScale = -1.0
$stageTwo.ValidFootCycleRewardScale = 1.0
$stageTwo.ReferenceTrackingRewardScale = 1.0
$stageTwo.ReferenceMseRewardScale = -0.75
$stageTwo.ReferenceActionIdentityInit = $false
$stageTwo.ResumeCheckpoint = $gateCheckpoint
$stageTwo.ResumeLoadOptimizer = $true
$stageTwo.RunName = $correctionRunName

& $trainer @stageTwo
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

$correctionReport = Get-Content -Raw -LiteralPath $correctionReportPath | ConvertFrom-Json
if ($correctionReport.status -ne "passed" -or -not $correctionReport.trained_policy_validation.passed) {
    Write-Error (
        "Direction correction did not pass; the long run was not started. Failures: {0}" -f
        ($correctionReport.trained_policy_validation.failures -join "; ")
    )
    exit 3
}

$correctionCheckpoint = Join-Path $nextPolicyRoot (
    "{0}\domino_cad_linkage_direct\{1}\{2}" -f
        $correctionRunName,
        $correctionReport.log_dir_name,
        $correctionReport.latest_checkpoint
)
if (-not (Test-Path -LiteralPath $correctionCheckpoint)) {
    throw "Direction-correction checkpoint was not found: $correctionCheckpoint"
}

$remainingIterations = $TotalIterations - $GateIterations - $CorrectionIterations
Write-Host "Direction gate passed. Continuing for $remainingIterations iterations."
$stageThree = $stageTwo.Clone()
$stageThree.Iterations = $remainingIterations
$stageThree.PolicyValidationSteps = 240
$stageThree.PolicyGateMinForwardM = 0.18
$stageThree.PolicyGateMaxLateralM = 0.20
$stageThree.PolicyGateMaxYawRad = 0.60
$stageThree.PolicyGateMaxTiltDeg = 50.0
$stageThree.PolicyGateMaxSwingContact = 0.45
$stageThree.PolicyGateMinGaitContactMatch = 0.72
$stageThree.PolicyGateMinSwingClearanceM = 0.004
$stageThree.PolicyGateMinEachCadFootClearanceM = 0.005
$stageThree.PolicyGateMinFootMotionM = 0.030
$stageThree.PolicyGateMinEachLinkageDriveMotionDeg = 2.0
$stageThree.PolicyGateMinValidCyclesPerFoot = 1
$stageThree.PolicyGateMinValidCycleRatio = 0.60
$stageThree.PolicyGateMaxFootCycleDominationRatio = 0.40
$stageThree.GaitCycleMaxTiltDeg = 30.0
$stageThree.GaitContactRewardScale = 5.0
$stageThree.SwingContactPenaltyScale = -6.0
$stageThree.FootClearanceRewardScale = 5.0
$stageThree.FootSlipRewardScale = -0.75
$stageThree.AirTimeVarianceRewardScale = -1.0
$stageThree.ValidFootCycleRewardScale = 1.5
$stageThree.ReferenceTrackingRewardScale = 0.5
$stageThree.ReferenceMseRewardScale = -0.25
$stageThree.ResumeCheckpoint = $correctionCheckpoint
$stageThree.ResumeLoadOptimizer = $true
$stageThree.RunName = $finalRunName

& $trainer @stageThree
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

$finalReport = Get-Content -Raw -LiteralPath $finalReportPath | ConvertFrom-Json
if ($finalReport.status -ne "passed" -or -not $finalReport.trained_policy_validation.passed) {
    Write-Error (
        "The final policy did not pass the walking gate. Failures: {0}" -f
        ($finalReport.trained_policy_validation.failures -join "; ")
    )
    exit 4
}

Write-Host "Domino velocity policy passed the final walking gate."
exit 0
