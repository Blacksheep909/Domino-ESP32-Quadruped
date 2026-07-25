param(
    [string]$IsaacSimRoot = $env:ISAAC_SIM_ROOT,
    [string]$IsaacLabRoot = $env:ISAACLAB_ROOT,
    [int]$SettleSteps = 120,
    [int]$Iterations = 2,
    [string]$CandidateValues = "-1.0,-0.5,0.0,0.5,1.0",
    [string]$ActiveChannels = "all",
    [ValidateSet("coordinate", "leg-pairs")]
    [string]$SearchMode = "coordinate",
    [string]$InitialActions = "",
    [double]$ActionScaleDeg = 20.0,
    [double]$ServoTargetRateLimitDegS = 180.0,
    [double]$TargetFootZM = 0.002,
    [double]$TargetPlanarReachM = 0.065,
    [string]$RunName = "grounded_rendered_foot_fixedbase_stance_calibration",
    [double]$FloatingHeightM = [double]::NaN,
    [switch]$FixedBase,
    [switch]$DisableGravity,
    [ValidateSet("flat", "stairs")]
    [string]$TerrainType = "flat",
    [ValidateSet("linkage-lower-closure", "actual-cad-visual-bottom", "actual-cad-grounded-support")]
    [string]$FootCollisionMode = "actual-cad-visual-bottom",
    [ValidateSet("direct", "passive")]
    [string]$ClosureModel = "passive",
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

$calibrationScript = Join-Path $repoRoot "simulation\isaac\prototypes\pin_linkage\run_domino_cad_linkage_stance_calibration.py"
if (-not (Test-Path -LiteralPath $calibrationScript)) {
    throw "Missing Domino stance calibration script: $calibrationScript"
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

$reportPath = Join-Path $repoRoot ("simulation\isaac\out\cad_identity\next_policy\" + $RunName + ".json")
$args = @(
    $calibrationScript,
    "--headless",
    "--settle-steps", ([string]$SettleSteps),
    "--iterations", ([string]$Iterations),
    ("--candidate-values={0}" -f $CandidateValues),
    "--active-channels", $ActiveChannels,
    "--search-mode", $SearchMode,
    "--action-scale-deg", ([string]$ActionScaleDeg),
    "--servo-target-rate-limit-deg-s", ([string]$ServoTargetRateLimitDegS),
    "--target-foot-z-m", ([string]$TargetFootZM),
    "--target-planar-reach-m", ([string]$TargetPlanarReachM),
    "--foot-collision-mode", $FootCollisionMode,
    "--closure-model", $ClosureModel,
    "--terrain-type", $TerrainType,
    "--report-path", $reportPath,
    "--no-print-report"
)

if ($InitialActions -and $InitialActions.Trim().Length -gt 0) {
    $args += ("--initial-actions={0}" -f $InitialActions)
}

if (-not [double]::IsNaN($FloatingHeightM)) {
    $args += @("--floating-height-m", ([string]$FloatingHeightM))
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

Write-Host "Launching Domino visible-CAD stance calibration..."
Write-Host ("Report: {0}" -f $reportPath)

& $isaacPython @args
exit $LASTEXITCODE
