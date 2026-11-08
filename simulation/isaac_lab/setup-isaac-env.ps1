<#
.SYNOPSIS
  One-time setup of the Python environment used to train Domino in Isaac Lab.

  Creates C:\isaac-projects\sim6 (Python 3.12) with Isaac Sim 6.2 from NVIDIA's
  pip index and Isaac Lab 3.0 (PhysX backend + RSL-RL) installed from an Isaac
  Lab v3.0.0 checkout. Needs ~30 GB, an NVIDIA RTX GPU and a 595-series or newer
  NVIDIA driver (Isaac Sim 5.1's renderer crashes on those drivers; 6.x needs them).

  Isaac Sim requires you to accept the NVIDIA Omniverse License Agreement.
  Read it, and if you accept, put $env:OMNI_KIT_ACCEPT_EULA = "YES" in
  simulation/isaac_lab/local-settings.ps1 (see local-settings.example.ps1).

  The project code also still runs on the older Isaac Sim 5.1 / Isaac Lab 2.3
  stack (domino_lab/compat.py); point DOMINO_PYTHON at such an env to use it.

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File simulation/isaac_lab/setup-isaac-env.ps1
#>
param(
    [string]$EnvRoot = "C:\isaac-projects\sim6",
    [string]$IsaacLabRoot = "C:\isaac-projects\IsaacLab3"
)
$ErrorActionPreference = "Stop"
if (-not (Test-Path (Join-Path $IsaacLabRoot "source\isaaclab"))) {
    git clone --depth 1 --branch v3.0.0 https://github.com/isaac-sim/IsaacLab.git $IsaacLabRoot
}
# Python 3.12 from python.org (winget Python.Python.3.12). uv's standalone Python
# builds crash Isaac Sim's windowed mode on Windows.
$py312 = Join-Path $env:LOCALAPPDATA "Programs\Python\Python312\python.exe"
if (-not (Test-Path $py312)) {
    winget install --id Python.Python.3.12 --exact --scope user --silent --accept-package-agreements --accept-source-agreements
}
& $py312 -m venv $EnvRoot
$python = Join-Path $EnvRoot "Scripts\python.exe"
& $python -m pip install --upgrade pip
& $python -m pip install "isaacsim[all,extscache]==6.2.0.0" --extra-index-url https://pypi.nvidia.com
# Isaac Lab core + RSL-RL; its installer also swaps in the CUDA build of PyTorch.
$env:VIRTUAL_ENV = $EnvRoot
$env:PATH = "$(Join-Path $EnvRoot 'Scripts');$env:PATH"
$p = Start-Process -FilePath (Join-Path $IsaacLabRoot "isaaclab.bat") -ArgumentList '-i', '"rl[rsl-rl]"' -WorkingDirectory $IsaacLabRoot -NoNewWindow -PassThru -Wait
if ($p.ExitCode -ne 0) { throw "Isaac Lab install failed ($($p.ExitCode))" }
# Boxer / gamepad driving in play.py --drive.
& $python -m pip install "hidapi==0.15.0"
& $python -c "import torch, rsl_rl; assert torch.cuda.is_available(); print('Domino Isaac env ready:', torch.__version__, torch.cuda.get_device_name(0))"
Write-Host "Next: accept the NVIDIA Omniverse EULA in local-settings.ps1 (see local-settings.example.ps1), then run run-train.ps1."
