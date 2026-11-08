# Copy to local-settings.ps1 (git-ignored) to keep machine-specific settings
# out of the repository. run-train.ps1 and policy-lab.ps1 load it.

# Only set this after you have read and accepted the NVIDIA Omniverse License
# Agreement yourself:
# https://docs.omniverse.nvidia.com/platform/latest/common/NVIDIA_Omniverse_License_Agreement.html
# $env:OMNI_KIT_ACCEPT_EULA = "YES"

# Python env with Isaac Sim + Isaac Lab (default C:\isaac-projects\sim6).
# $env:DOMINO_PYTHON = "D:\isaac\sim6\Scripts\python.exe"
