param(
    [int]$Steps = 300,
    [double]$ActionAmplitude = 0.15,
    [double]$HoldOpenSeconds = 45.0,
    [string]$GateName = "single_robot_slow_sweep"
)

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$isaacSimRoot = "C:\isaac-sim"
$isaacLabRoot = "C:\isaac-projects\IsaacLab"
$isaacPython = Join-Path $isaacSimRoot "python.bat"
$runner = Join-Path $repoRoot "simulation\isaac\prototypes\pin_linkage\run_domino_cad_linkage_env_smoke.py"
$outputRoot = Join-Path $repoRoot "simulation\isaac\out\cad_identity\mechanism_visual_gate"
$reportPath = Join-Path $outputRoot ($GateName + ".json")
$capturePath = Join-Path $outputRoot ($GateName + ".png")

New-Item -ItemType Directory -Force -Path $outputRoot | Out-Null

$pythonPathParts = @(
    (Join-Path $isaacLabRoot "source\isaaclab"),
    (Join-Path $isaacLabRoot "source\isaaclab_assets"),
    (Join-Path $isaacLabRoot "source\isaaclab_rl"),
    (Join-Path $isaacLabRoot "source\isaaclab_tasks"),
    (Join-Path $isaacLabRoot "source\isaaclab_mimic"),
    (Join-Path $repoRoot "simulation\isaac\out\isaac_py_deps_clean"),
    (Join-Path $isaacSimRoot "site")
)
$env:ISAAC_SIM_ROOT = $isaacSimRoot
$env:PYTHONPATH = ($pythonPathParts -join ";")

Set-Location -LiteralPath $repoRoot
& $isaacPython `
    $runner `
    --num-envs 1 `
    --steps $Steps `
    --action-amplitude $ActionAmplitude `
    --action-scale-deg 16 `
    --closure-model passive `
    --visible-step-delay-s 0.02 `
    --capture-viewport-path $capturePath `
    --hold-open-seconds $HoldOpenSeconds `
    --report-path $reportPath `
    --device cpu `
    --kit_args=--/app/vulkan=false

exit $LASTEXITCODE
