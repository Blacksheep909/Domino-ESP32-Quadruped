param(
    [string]$IsaacLabRoot = $env:ISAACLAB_ROOT,
    [string]$IsaacPython = $env:ISAAC_PYTHON,
    [string]$UrdfPath,
    [string]$OutputUsd,
    [switch]$AcceptEula,
    [switch]$NoMergeJoints
)

$ErrorActionPreference = "Stop"

$isaacDir = $PSScriptRoot
if (-not $UrdfPath -or $UrdfPath.Trim().Length -eq 0) {
    $UrdfPath = Join-Path $isaacDir "..\urdf\generated\Domino_URDF_Parts_Combined_Final_description\urdf\Domino_URDF_Parts_Combined_Final.urdf"
}

if (-not $OutputUsd -or $OutputUsd.Trim().Length -eq 0) {
    $OutputUsd = Join-Path $isaacDir "out\domino_raw_import.usd"
}

if (-not $IsaacLabRoot -or $IsaacLabRoot.Trim().Length -eq 0) {
    throw "Set -IsaacLabRoot or ISAACLAB_ROOT to your IsaacLab checkout."
}

if (-not $IsaacPython -or $IsaacPython.Trim().Length -eq 0) {
    throw "Set -IsaacPython or ISAAC_PYTHON to the Python executable for your Isaac Sim / Isaac Lab environment."
}

$resolvedIsaacLabRoot = Resolve-Path -LiteralPath $IsaacLabRoot
$resolvedIsaacPython = Resolve-Path -LiteralPath $IsaacPython
$resolvedUrdf = Resolve-Path -LiteralPath $UrdfPath

$convertScript = Join-Path $resolvedIsaacLabRoot "scripts\tools\convert_urdf.py"
if (-not (Test-Path -LiteralPath $convertScript)) {
    throw "Could not find Isaac Lab URDF converter at $convertScript"
}

$outputDir = Split-Path -Parent $OutputUsd
if ($outputDir -and -not (Test-Path -LiteralPath $outputDir)) {
    New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
}

if ($AcceptEula) {
    $env:OMNI_KIT_ACCEPT_EULA = "YES"
}

$pythonDir = Split-Path -Parent $resolvedIsaacPython
$env:PATH = @(
    $pythonDir,
    (Join-Path $pythonDir "Scripts"),
    (Join-Path $pythonDir "Library\bin"),
    $env:PATH
) -join ";"

$args = @(
    $convertScript,
    $resolvedUrdf.Path,
    $OutputUsd,
    "--joint-stiffness", "0.0",
    "--joint-damping", "0.0",
    "--joint-target-type", "none",
    "--headless"
)

if (-not $NoMergeJoints) {
    $args += "--merge-joints"
}

Write-Host "Running Isaac Lab URDF import smoke test..."
Write-Host ("URDF: {0}" -f $resolvedUrdf.Path)
Write-Host ("Output: {0}" -f $OutputUsd)

& $resolvedIsaacPython.Path @args

if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

Write-Host "Import command completed."
