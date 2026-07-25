param(
    [string]$IsaacSimRoot = "C:\isaac-sim",
    [string]$IsaacLabRoot = "C:\isaac-projects\IsaacLab",
    [int]$NumEnvs = 1,
    [int]$Iterations = 1000000
)

$ErrorActionPreference = "Stop"

$trainingLauncher = Join-Path $PSScriptRoot "run-visible-domino-training.ps1"

& $trainingLauncher `
    -IsaacSimRoot $IsaacSimRoot `
    -IsaacLabRoot $IsaacLabRoot `
    -NumEnvs $NumEnvs `
    -Iterations $Iterations `
    -NumStepsPerEnv 500 `
    -SaveInterval 10 `
    -PolicyValidationSteps 0 `
    -VisibleStepDelayS 0.02 `
    -ActionScaleDeg 16 `
    -ServoTargetRateLimitDegS 90 `
    -EpisodeLengthS 48 `
    -MinHeightM 0.16 `
    -MaxTiltDeg 60 `
    -ActualCadGroundClearanceM 0.001 `
    -RunName "domino_verified_swing_teacher_ppo_training" `
    -ReferenceGait (Join-Path $PSScriptRoot "config\domino_linkage_swing_cycle_teacher.json") `
    -TerrainType flat `
    -FootCollisionMode actual-cad-visual-bottom `
    -ClosureModel passive `
    -NoHoldOpen `
    -SkipPPOAfterBC:$false

exit $LASTEXITCODE
