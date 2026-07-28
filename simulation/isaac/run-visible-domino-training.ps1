param(
    [string]$IsaacSimRoot = $env:ISAAC_SIM_ROOT,
    [string]$IsaacLabRoot = $env:ISAACLAB_ROOT,
    [int]$NumEnvs = 1,
    [int]$Iterations = 1,
    [int]$NumStepsPerEnv = 32,
    [ValidateSet("cpu", "cuda:0")]
    [string]$PolicyDevice = "cuda:0",
    [ValidateSet("same", "cpu", "cuda:0")]
    [string]$PhysicsDevice = "cpu",
    [int]$SaveInterval = 1,
    [int]$PolicyValidationSteps = 2360,
    [int]$PolicyValidationSettleSteps = 120,
    [int]$PolicyValidationRampSteps = 0,
    [double]$PolicyGateMinForwardM = 0.001,
    [double]$PolicyGateMaxLateralM = 0.12,
    [double]$PolicyGateMaxYawRad = 0.70,
    [double]$PolicyGateMaxTiltDeg = 30.0,
    [double]$PolicyGateMaxSwingContact = 0.60,
    [double]$PolicyGateMinGaitContactMatch = 0.0,
    [double]$PolicyGateMinSwingClearanceM = 0.003,
    [double]$PolicyGateMinEachCadFootClearanceM = 0.005,
    [double]$PolicyGateMinFootMotionM = 0.050,
    [double]$PolicyGateMinEachLinkageDriveMotionDeg = 4.0,
    [int]$PolicyGateMinValidCyclesPerFoot = 1,
    [double]$PolicyGateMinValidCycleRatio = 0.50,
    [double]$PolicyGateMaxFootCycleDominationRatio = 0.50,
    [int]$GaitCycleMinAirSteps = 3,
    [int]$GaitCycleTouchdownConfirmSteps = 2,
    [double]$GaitCycleMinClearanceM = 0.004,
    [double]$GaitCycleMinBodyRelativeTravelM = 0.020,
    [double]$GaitCycleMaxTiltDeg = 25.0,
    [int]$GaitCycleMinTouchdownSupportFeet = 2,
    [double]$VisibleStepDelayS = 0.0,
    [int]$Seed = 9306,
    [double]$InitialNoiseStd = 0.05,
    [double]$PpoLearningRate = 0.00004,
    [double]$PpoEntropyCoefficient = 0.0008,
    [double]$ActionScaleDeg = 16.0,
    [double]$ServoTargetRateLimitDegS = 90.0,
    [int]$ResetSettleSteps = 15,
    [double]$CommandXMps = 0.05,
    [double]$GaitFrequencyHz = 1.0,
    [double]$EpisodeLengthS = 70.0,
    [double]$MinHeightM = 0.02,
    [double]$MaxTiltDeg = 75.0,
    [double]$ActualCadGroundClearanceM = 0.001,
    [double]$GroundSizeM = 10.0,
    [double]$AliveRewardScale = 1.0,
    [double]$CommandProgressRewardScale = 3.6,
    [double]$CommandVelocityRewardScale = -4.0,
    [double]$CommandVelocityTrackingRewardScale = 1.5,
    [double]$CommandVelocityTrackingSigma = 0.06,
    [double]$CommandStagnationPenaltyScale = 0.0,
    [double]$CommandStagnationSpeedMps = 0.03,
    [double]$LateralDriftRewardScale = -650.0,
    [double]$YawDriftRewardScale = -3.0,
    [double]$CommandYawRewardScale = -1.5,
    [double]$GaitContactRewardScale = 2.2,
    [double]$StanceContactRewardScale = 0.45,
    [double]$SwingContactPenaltyScale = -4.0,
    [double]$FootClearanceRewardScale = 4.0,
    [double]$FootContactRewardScale = 0.0,
    [double]$FootSlipRewardScale = -0.25,
    [double]$AirTimeVarianceRewardScale = -0.5,
    [double]$ValidFootCycleRewardScale = 0.5,
    [double]$FootCycleMinAirTimeS = 0.06,
    [double]$FootCycleTargetAirTimeS = 0.20,
    [double]$FootCycleMinClearanceM = 0.004,
    [double]$FootCycleMinBodyRelativeTravelM = 0.015,
    [double]$ActionRewardScale = -0.02,
    [double]$ActionRateRewardScale = -0.02,
    [double]$ReferenceTrackingRewardScale = 2.0,
    [double]$ReferenceTrackingSigma = 0.55,
    [double]$ReferenceMseRewardScale = -2.5,
    [bool]$ReferenceActionIdentityInit = $true,
    [int]$ReferenceActionBcSteps = 1,
    [string]$RunName = "linkage_swing_hipframe_bc_training",
    [string]$ReferenceGait = "",
    [switch]$OpenPolicy,
    [ValidateSet("flat", "stairs")]
    [string]$TerrainType = "flat",
    [ValidateSet("linkage-lower-closure", "actual-cad-visual-bottom", "actual-cad-grounded-support")]
    [string]$FootCollisionMode = "actual-cad-visual-bottom",
    [ValidateSet("direct", "passive")]
    [string]$ClosureModel = "passive",
    [string]$ResumeCheckpoint = "",
    [ValidateSet("reset", "final", "policy")]
    [string]$HoldOpenMode = "final",
    [int]$HoldOpenRenderFrames = 4,
    [int]$HoldOpenExitAfterFrames = 0,
    [switch]$NoHoldOpen,
    [switch]$Headless,
    [switch]$ResumeLoadOptimizer,
    [switch]$SkipPPOAfterBC = $true,
    [switch]$AllowIndefinitePolicyHoldOpen,
    [switch]$NoVulkanWorkaround
)

$ErrorActionPreference = "Stop"

$isaacDir = $PSScriptRoot
$repoRoot = (Resolve-Path -LiteralPath (Join-Path $isaacDir "..\..")).Path
$runningKit = Get-Process -Name "kit" -ErrorAction SilentlyContinue
if ($runningKit) {
    $runningDescriptions = $runningKit | ForEach-Object {
        "PID {0}, started {1}" -f $_.Id, $_.StartTime
    }
    throw (
        "Isaac Kit is already running ({0}). Close the existing Isaac window before launching so the current " +
        "Domino code and CAD contacts are loaded."
    ) -f ($runningDescriptions -join "; ")
}

if (-not $IsaacSimRoot -or $IsaacSimRoot.Trim().Length -eq 0) {
    throw "Set -IsaacSimRoot or ISAAC_SIM_ROOT to your Isaac Sim install."
}

if (-not $IsaacLabRoot -or $IsaacLabRoot.Trim().Length -eq 0) {
    throw "Set -IsaacLabRoot or ISAACLAB_ROOT to your Isaac Lab checkout."
}

$resolvedIsaacSimRoot = (Resolve-Path -LiteralPath $IsaacSimRoot).Path
$resolvedIsaacLabRoot = (Resolve-Path -LiteralPath $IsaacLabRoot).Path
$isaacPython = Join-Path $resolvedIsaacSimRoot "python.bat"
if (-not (Test-Path -LiteralPath $isaacPython)) {
    throw "Could not find Isaac Sim Python at $isaacPython"
}

$trainScript = Join-Path $repoRoot "simulation\isaac\prototypes\pin_linkage\run_domino_cad_linkage_rsl_rl_train.py"
$resolvedResumeCheckpoint = ""
if ($ResumeCheckpoint -and $ResumeCheckpoint.Trim().Length -gt 0) {
    $resolvedResumeCheckpoint = (Resolve-Path -LiteralPath $ResumeCheckpoint).Path
}
if ($OpenPolicy -and ($ReferenceActionIdentityInit -or $ReferenceActionBcSteps -gt 0)) {
    throw "Open-policy training requires -ReferenceActionIdentityInit:`$false and -ReferenceActionBcSteps 0."
}
$referenceGaitCandidates = @(
    (Join-Path $repoRoot "simulation\isaac\config\domino_linkage_swing_cycle_teacher.json"),
    (Join-Path $repoRoot "simulation\isaac\config\domino_calibrated_neutral_teacher.json"),
    (Join-Path $repoRoot "simulation\isaac\out\cad_identity\teacher_grid\teacher_grounded_support_scale20_seed240704.json"),
    (Join-Path $repoRoot "simulation\isaac\out\cad_identity\teacher_grid\teacher_grounded_support_valid_lower_scale20_seed240704.json"),
    (Join-Path $repoRoot "simulation\isaac\out\cad_identity\teacher_grid\teacher_random001_scale70_freq225.json")
)
$referenceGait = ""
if (-not $OpenPolicy) {
    if ($ReferenceGait -and $ReferenceGait.Trim().Length -gt 0) {
        $referenceGait = (Resolve-Path -LiteralPath $ReferenceGait).Path
    } else {
        $referenceGait = $referenceGaitCandidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
    }
}
$logRoot = Join-Path $repoRoot ("simulation\isaac\out\cad_identity\next_policy\" + $RunName)
$reportPath = Join-Path $repoRoot ("simulation\isaac\out\cad_identity\next_policy\" + $RunName + ".json")

$requiredArtifacts = @($trainScript)
if (-not $OpenPolicy) {
    $requiredArtifacts += $referenceGait
}
if ($resolvedResumeCheckpoint) {
    $requiredArtifacts += $resolvedResumeCheckpoint
}
foreach ($required in $requiredArtifacts) {
    if (-not (Test-Path -LiteralPath $required)) {
        throw "Missing required Domino training artifact: $required"
    }
}

Set-Location -LiteralPath $repoRoot

$pythonPathParts = @(
    (Join-Path $resolvedIsaacLabRoot "source\isaaclab"),
    (Join-Path $resolvedIsaacLabRoot "source\isaaclab_rl"),
    (Join-Path $resolvedIsaacLabRoot "source\isaaclab_tasks"),
    (Join-Path $resolvedIsaacLabRoot "source\isaaclab_mimic"),
    (Join-Path $repoRoot "simulation\isaac\out\isaac_py_deps_clean"),
    (Join-Path $resolvedIsaacSimRoot "site")
)
if ($env:PYTHONPATH) {
    $pythonPathParts += $env:PYTHONPATH
}
$env:ISAAC_SIM_ROOT = $resolvedIsaacSimRoot
$env:PYTHONPATH = ($pythonPathParts -join ";")
$env:PYTHONUNBUFFERED = "1"

$args = @(
    $trainScript,
    "--num-envs", ([string]$NumEnvs),
    "--iterations", ([string]$Iterations),
    "--num-steps-per-env", ([string]$NumStepsPerEnv),
    "--device", $PolicyDevice,
    "--physics-device", $PhysicsDevice,
    "--save-interval", ([string]$SaveInterval),
    "--visible-step-delay-s", ([string]$VisibleStepDelayS),
    "--seed", ([string]$Seed),
    "--init-noise-std", ([string]$InitialNoiseStd),
    "--ppo-learning-rate", ([string]$PpoLearningRate),
    "--ppo-entropy-coef", ([string]$PpoEntropyCoefficient),
    "--action-scale-deg", ([string]$ActionScaleDeg),
    "--servo-target-rate-limit-deg-s", ([string]$ServoTargetRateLimitDegS),
    "--reset-settle-steps", ([string]$ResetSettleSteps),
    "--command-x-m-s", ([string]$CommandXMps),
    "--command-y-m-s", "0.0",
    "--command-yaw-rad-s", "0.0",
    "--gait-frequency-hz", ([string]$GaitFrequencyHz),
    "--episode-length-s", ([string]$EpisodeLengthS),
    "--min-height-m", ([string]$MinHeightM),
    "--max-tilt-deg", ([string]$MaxTiltDeg),
    "--actual-cad-ground-clearance-m", ([string]$ActualCadGroundClearanceM),
    "--ground-size-m", ([string]$GroundSizeM),
    "--policy-validation-steps", ([string]$PolicyValidationSteps),
    "--policy-validation-settle-steps", ([string]$PolicyValidationSettleSteps),
    "--policy-validation-ramp-steps", ([string]$PolicyValidationRampSteps),
    "--policy-reference-action-snap-tolerance", "0.0",
    "--policy-gate-max-joint-separation-m", "0.001",
    "--policy-gate-min-forward-m", ([string]$PolicyGateMinForwardM),
    "--policy-gate-max-lateral-m", ([string]$PolicyGateMaxLateralM),
    "--policy-gate-max-yaw-rad", ([string]$PolicyGateMaxYawRad),
    "--policy-gate-max-tilt-deg", ([string]$PolicyGateMaxTiltDeg),
    "--policy-gate-max-swing-contact", ([string]$PolicyGateMaxSwingContact),
    "--policy-gate-min-gait-contact-match", ([string]$PolicyGateMinGaitContactMatch),
    "--policy-gate-min-swing-clearance-m", ([string]$PolicyGateMinSwingClearanceM),
    "--policy-gate-min-each-cad-foot-clearance-m", ([string]$PolicyGateMinEachCadFootClearanceM),
    "--policy-gate-min-foot-motion-m", ([string]$PolicyGateMinFootMotionM),
    "--policy-gate-min-each-linkage-drive-motion-deg", ([string]$PolicyGateMinEachLinkageDriveMotionDeg),
    "--policy-gate-max-visual-foot-motion-m", "0.25",
    "--policy-gate-min-valid-cycles-per-foot", ([string]$PolicyGateMinValidCyclesPerFoot),
    "--policy-gate-min-valid-cycle-ratio", ([string]$PolicyGateMinValidCycleRatio),
    "--policy-gate-max-foot-cycle-domination-ratio", ([string]$PolicyGateMaxFootCycleDominationRatio),
    "--gait-cycle-min-air-steps", ([string]$GaitCycleMinAirSteps),
    "--gait-cycle-touchdown-confirm-steps", ([string]$GaitCycleTouchdownConfirmSteps),
    "--gait-cycle-min-clearance-m", ([string]$GaitCycleMinClearanceM),
    "--gait-cycle-min-body-relative-travel-m", ([string]$GaitCycleMinBodyRelativeTravelM),
    "--gait-cycle-max-tilt-deg", ([string]$GaitCycleMaxTiltDeg),
    "--gait-cycle-min-touchdown-support-feet", ([string]$GaitCycleMinTouchdownSupportFeet),
    "--foot-collision-mode", $FootCollisionMode,
    "--closure-model", $ClosureModel,
    "--terrain-type", $TerrainType,
    "--alive-reward-scale", ([string]$AliveRewardScale),
    "--command-progress-reward-scale", ([string]$CommandProgressRewardScale),
    "--command-velocity-reward-scale", ([string]$CommandVelocityRewardScale),
    "--command-velocity-tracking-reward-scale", ([string]$CommandVelocityTrackingRewardScale),
    "--command-velocity-tracking-sigma", ([string]$CommandVelocityTrackingSigma),
    "--command-stagnation-penalty-scale", ([string]$CommandStagnationPenaltyScale),
    "--command-stagnation-speed-m-s", ([string]$CommandStagnationSpeedMps),
    "--lateral-drift-reward-scale", ([string]$LateralDriftRewardScale),
    "--yaw-drift-reward-scale", ([string]$YawDriftRewardScale),
    "--command-yaw-reward-scale", ([string]$CommandYawRewardScale),
    "--gait-contact-reward-scale", ([string]$GaitContactRewardScale),
    "--stance-contact-reward-scale", ([string]$StanceContactRewardScale),
    "--swing-contact-penalty-scale", ([string]$SwingContactPenaltyScale),
    "--foot-clearance-reward-scale", ([string]$FootClearanceRewardScale),
    "--foot-contact-reward-scale", ([string]$FootContactRewardScale),
    "--foot-slip-reward-scale", ([string]$FootSlipRewardScale),
    "--air-time-variance-reward-scale", ([string]$AirTimeVarianceRewardScale),
    "--valid-foot-cycle-reward-scale", ([string]$ValidFootCycleRewardScale),
    "--foot-cycle-min-air-time-s", ([string]$FootCycleMinAirTimeS),
    "--foot-cycle-target-air-time-s", ([string]$FootCycleTargetAirTimeS),
    "--foot-cycle-min-clearance-m", ([string]$FootCycleMinClearanceM),
    "--foot-cycle-min-body-relative-travel-m", ([string]$FootCycleMinBodyRelativeTravelM),
    "--action-reward-scale", ([string]$ActionRewardScale),
    "--action-rate-reward-scale", ([string]$ActionRateRewardScale),
    "--log-root", $logRoot,
    "--report-path", $reportPath,
    "--rendering_mode", "performance"
)

if (-not $OpenPolicy) {
    $args += @(
        "--reference-gait-candidate", $referenceGait,
        "--include-reference-actions-in-observation",
        "--reference-action-bc-steps", ([string]$ReferenceActionBcSteps),
        "--reference-action-bc-settle-steps", "0",
        "--reference-action-bc-replay-steps", "0",
        "--reference-action-bc-batch-size", "256",
        "--reference-action-bc-lr", "0.0002",
        "--reference-action-bc-output-penalty", "0.0",
        "--reference-action-bc-lower-linkage-weight", "1.5",
        "--reference-action-bc-upper-pitch-weight", "1.5",
        "--reference-action-tracking-reward-scale", ([string]$ReferenceTrackingRewardScale),
        "--reference-action-tracking-sigma", ([string]$ReferenceTrackingSigma),
        "--reference-action-mse-reward-scale", ([string]$ReferenceMseRewardScale)
    )
}

if ($NumEnvs -gt 1) {
    $args += "--allow-multi-env-viewport"
}

if (-not $OpenPolicy -and $ReferenceActionIdentityInit) {
    $args += "--reference-action-identity-init"
}

if (-not $NoVulkanWorkaround) {
    $args += "--kit_args=--/app/vulkan=false"
}

if ($resolvedResumeCheckpoint) {
    $args += @("--resume-checkpoint", $resolvedResumeCheckpoint)
}

if ($ResumeLoadOptimizer) {
    $args += "--resume-load-optimizer"
}

if ($Headless) {
    $args += "--headless"
}

if ($SkipPPOAfterBC) {
    $args += "--skip-ppo-after-bc"
}

if ($AllowIndefinitePolicyHoldOpen) {
    $args += "--allow-indefinite-policy-hold-open"
}

if (-not $NoHoldOpen) {
    $args += @(
        "--hold-open",
        "--hold-open-mode", $HoldOpenMode,
        "--hold-open-render-frames", ([string]$HoldOpenRenderFrames),
        "--hold-open-exit-after-frames", ([string]$HoldOpenExitAfterFrames)
    )
}

Write-Host "Launching Domino actual-CAD linkage-swing policy training..."
Write-Host "Contact model: CAD-fitted 11.991 mm foot spheres with startup terrain-penetration gate."
Write-Host ("Command range: +/-{0:N1} deg; servo target slew: {1:N1} deg/s." -f $ActionScaleDeg, $ServoTargetRateLimitDegS)
Write-Host ("Devices: policy={0}; physics={1}." -f $PolicyDevice, $PhysicsDevice)
Write-Host ("Report: {0}" -f $reportPath)
Write-Host ("Log root: {0}" -f $logRoot)
Write-Host ("Policy initialization: {0}" -f $(if ($OpenPolicy) { "random PPO; no reference observation or imitation reward" } else { "reference-guided" }))
if ($resolvedResumeCheckpoint) {
    Write-Host ("Resume checkpoint: {0}" -f $resolvedResumeCheckpoint)
} else {
    Write-Host "Resume checkpoint: none; starting fresh for this CAD/contact setup."
}

& $isaacPython @args
exit $LASTEXITCODE
