param(
    [string]$IsaacSimRoot = $env:ISAAC_SIM_ROOT,
    [string]$IsaacLabRoot = $env:ISAACLAB_ROOT,
    [ValidateSet("checkpoint", "reference", "zero", "fixed")]
    [string]$PolicyMode = "checkpoint",
    [ValidateSet("cpu", "cuda:0")]
    [string]$Device = "cpu",
    [string]$Checkpoint = "",
    [int]$Steps = 2360,
    [int]$StartupZeroSteps = 120,
    [int]$PolicyRampSteps = 0,
    [string]$StartupActions = "0,0,0,0,0,0,0,0,0,0,0,0",
    [string]$FixedActions = "",
    [int]$Seed = 240727,
    [double]$ActionScaleDeg = 16.0,
    [double]$ServoTargetRateLimitDegS = 90.0,
    [double]$EpisodeLengthS = 70.0,
    [double]$CommandXMps = 0.0,
    [double]$FloatingHeightM = [double]::NaN,
    [double]$MinHeightM = 0.22,
    [double]$MaxTiltDeg = 30.0,
    [string]$RunName = "linkage_swing_hipframe_bc_policy_visible",
    [ValidateSet("flat", "stairs")]
    [string]$TerrainType = "flat",
    [ValidateSet("linkage-lower-closure", "actual-cad-visual-bottom", "actual-cad-grounded-support")]
    [string]$FootCollisionMode = "actual-cad-visual-bottom",
    [ValidateSet("direct", "passive")]
    [string]$ClosureModel = "passive",
    [int]$HoldOpenRenderFrames = 4,
    [int]$HoldOpenExitAfterFrames = 0,
    [double]$VisibleStartDelayS = 0.0,
    [double]$VisibleStepDelayS = 0.0,
    [string]$CaptureViewportPath = "",
    [string]$ReferenceGait = "",
    [switch]$NoHoldOpen,
    [switch]$Headless,
    [switch]$FixedBase,
    [switch]$DisableGravity,
    [switch]$AlignRenderedVisualMinFootAfterStartup,
    [switch]$NoVulkanWorkaround
)

$ErrorActionPreference = "Stop"

$isaacDir = $PSScriptRoot
$repoRoot = (Resolve-Path -LiteralPath (Join-Path $isaacDir "..\..")).Path

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

$playScript = Join-Path $repoRoot "simulation\isaac\prototypes\pin_linkage\run_domino_cad_linkage_rsl_rl_play.py"
$nextPolicyRoot = Join-Path $repoRoot "simulation\isaac\out\cad_identity\next_policy"
$verifiedPolicyCandidates = @()
$trainingRoot = Join-Path $nextPolicyRoot "linkage_swing_hipframe_bc_training"
$verifiedTrainingReportPath = Join-Path $nextPolicyRoot "linkage_swing_hipframe_bc_training.json"
$freshValidationReportPath = Join-Path $nextPolicyRoot "linkage_swing_hipframe_bc_training_fresh_validation.json"
if (Test-Path -LiteralPath $verifiedTrainingReportPath) {
    try {
        $verifiedTrainingReport = Get-Content -Raw -LiteralPath $verifiedTrainingReportPath | ConvertFrom-Json
        $freshValidationPassed = $false
        if (Test-Path -LiteralPath $freshValidationReportPath) {
            $freshValidationReport = Get-Content -Raw -LiteralPath $freshValidationReportPath | ConvertFrom-Json
            $freshValidationPassed = (
                $freshValidationReport.status -eq "passed" -and
                $freshValidationReport.policy_mode -eq "checkpoint" -and
                $freshValidationReport.checkpoint_run -eq $verifiedTrainingReport.log_dir_name -and
                $freshValidationReport.checkpoint -eq $verifiedTrainingReport.latest_checkpoint
            )
        }
        $embeddedValidationPassed = (
            $verifiedTrainingReport.status -eq "passed" -and
            $verifiedTrainingReport.trained_policy_validation.passed -eq $true
        )
        if (
            ($embeddedValidationPassed -or $freshValidationPassed) -and
            $verifiedTrainingReport.log_dir_name -and
            $verifiedTrainingReport.latest_checkpoint
        ) {
            $reportedCheckpoint = Join-Path $trainingRoot (
                "domino_cad_linkage_direct\{0}\{1}" -f
                    $verifiedTrainingReport.log_dir_name,
                    $verifiedTrainingReport.latest_checkpoint
            )
            if (Test-Path -LiteralPath $reportedCheckpoint) {
                $verifiedPolicyCandidates += (Resolve-Path -LiteralPath $reportedCheckpoint).Path
            }
        }
    } catch {
        Write-Warning "Could not read the verified linkage-swing training report: $($_.Exception.Message)"
    }
}
$defaultCheckpointCandidates = @($verifiedPolicyCandidates)
$checkpointPath = ""
if ($PolicyMode -eq "checkpoint") {
    if ($Checkpoint -and $Checkpoint.Trim().Length -gt 0) {
        $checkpointPath = (Resolve-Path -LiteralPath $Checkpoint).Path
    } else {
        $checkpointPath = $defaultCheckpointCandidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
    }
    if (-not $checkpointPath) {
        throw "Could not find the verified hip-frame linkage-swing checkpoint. Run run-visible-domino-training.ps1 first, or pass -Checkpoint explicitly."
    }
}

$referenceGaitCandidates = @(
    (Join-Path $repoRoot "simulation\isaac\config\domino_linkage_swing_cycle_teacher.json"),
    (Join-Path $repoRoot "simulation\isaac\config\domino_weight_transfer_cycle_teacher.json"),
    (Join-Path $repoRoot "simulation\isaac\config\domino_calibrated_neutral_teacher.json"),
    (Join-Path $repoRoot "simulation\isaac\out\cad_identity\teacher_grid\teacher_grounded_support_scale20_seed240704.json"),
    (Join-Path $repoRoot "simulation\isaac\out\cad_identity\teacher_grid\teacher_grounded_support_valid_lower_scale20_seed240704.json"),
    (Join-Path $repoRoot "simulation\isaac\out\cad_identity\teacher_grid\teacher_random001_scale70_freq225.json")
)
if ($ReferenceGait -and $ReferenceGait.Trim().Length -gt 0) {
    $referenceGait = (Resolve-Path -LiteralPath $ReferenceGait).Path
} else {
    $referenceGait = $referenceGaitCandidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
}
$reportPath = Join-Path $repoRoot ("simulation\isaac\out\cad_identity\next_policy\" + $RunName + ".json")

$requiredArtifacts = @($playScript, $referenceGait)
if ($PolicyMode -eq "checkpoint") {
    $requiredArtifacts += $checkpointPath
}
foreach ($required in $requiredArtifacts) {
    if (-not (Test-Path -LiteralPath $required)) {
        throw "Missing required Domino playback artifact: $required"
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

$args = @(
    $playScript,
    "--policy-mode", $PolicyMode,
    "--device", $Device,
    "--num-envs", "1",
    "--steps", ([string]$Steps),
    "--startup-zero-steps", ([string]$StartupZeroSteps),
    "--policy-ramp-steps", ([string]$PolicyRampSteps),
    "--seed", ([string]$Seed),
    "--reference-gait-candidate", $referenceGait,
    "--include-reference-actions-in-observation",
    "--reference-action-snap-tolerance", "0.0001",
    "--action-scale-deg", ([string]$ActionScaleDeg),
    "--servo-target-rate-limit-deg-s", ([string]$ServoTargetRateLimitDegS),
    "--min-each-linkage-drive-motion-deg", "4.0",
    "--min-each-foot-motion-m", "0.050",
    "--max-joint-separation-m", "0.001",
    "--command-x-m-s", ([string]$CommandXMps),
    "--command-y-m-s", "0.0",
    "--command-yaw-rad-s", "0.0",
    "--gait-frequency-hz", "1.0",
    "--episode-length-s", ([string]$EpisodeLengthS),
    "--foot-collision-mode", $FootCollisionMode,
    "--closure-model", $ClosureModel,
    "--terrain-type", $TerrainType,
    "--command-progress-reward-scale", "3.6",
    "--command-velocity-reward-scale", "-4.0",
    "--command-velocity-tracking-reward-scale", "1.5",
    "--lateral-drift-reward-scale", "-650.0",
    "--yaw-drift-reward-scale", "-3.0",
    "--command-yaw-reward-scale", "-1.5",
    "--gait-contact-reward-scale", "2.2",
    "--stance-contact-reward-scale", "0.45",
    "--swing-contact-penalty-scale", "-4.0",
    "--foot-clearance-reward-scale", "4.0",
    "--foot-contact-reward-scale", "0.0",
    "--reference-action-tracking-reward-scale", "2.0",
    "--reference-action-tracking-sigma", "0.55",
    "--reference-action-mse-reward-scale", "-2.5",
    "--report-path", $reportPath,
    "--rendering_mode", "performance"
)

if ($VisibleStartDelayS -gt 0.0) {
    $args += @("--visible-start-delay-s", ([string]$VisibleStartDelayS))
}

if ($VisibleStepDelayS -gt 0.0) {
    $args += @("--visible-step-delay-s", ([string]$VisibleStepDelayS))
}

if ($CaptureViewportPath -and $CaptureViewportPath.Trim().Length -gt 0) {
    $args += @("--capture-viewport-path", $CaptureViewportPath)
}

if ($StartupActions -and $StartupActions.Trim().Length -gt 0) {
    $args += ("--startup-actions={0}" -f $StartupActions)
}

if ($FixedActions -and $FixedActions.Trim().Length -gt 0) {
    $args += ("--fixed-actions={0}" -f $FixedActions)
}

if ($AlignRenderedVisualMinFootAfterStartup) {
    $args += "--align-rendered-visual-min-foot-after-startup"
}

if (-not [double]::IsNaN($FloatingHeightM)) {
    $args += @("--floating-height-m", ([string]$FloatingHeightM))
}

if (-not [double]::IsNaN($MinHeightM)) {
    $args += @("--min-height-m", ([string]$MinHeightM))
}

if (-not [double]::IsNaN($MaxTiltDeg)) {
    $args += @("--max-tilt-deg", ([string]$MaxTiltDeg))
}

if ($PolicyMode -eq "checkpoint") {
    $args += @("--checkpoint", $checkpointPath)
}

if ($Headless) {
    $args += "--headless"
}

if ($FixedBase) {
    $args += "--fixed-base"
}

if ($DisableGravity) {
    $args += "--disable-gravity"
}

if (-not $NoVulkanWorkaround) {
    $args += "--kit_args=--/app/vulkan=false"
}

if (-not $NoHoldOpen) {
    $args += @(
        "--hold-open",
        "--hold-open-render-frames", ([string]$HoldOpenRenderFrames),
        "--hold-open-exit-after-frames", ([string]$HoldOpenExitAfterFrames)
    )
}

Write-Host "Launching visible Domino actual-CAD policy playback..."
Write-Host ("Policy mode: {0}" -f $PolicyMode)
if ($PolicyMode -eq "checkpoint") {
    Write-Host ("Checkpoint: {0}" -f $checkpointPath)
}
Write-Host ("Report: {0}" -f $reportPath)

& $isaacPython @args
exit $LASTEXITCODE
