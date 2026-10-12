param(
    [ValidateSet("wifi", "usb", "bluetooth")]
    [string]$Transport = "wifi",
    [string]$RobotHost = "",
    [ValidateRange(1, 65535)]
    [int]$RobotPort = 8766,
    [string]$Device = "",
    [ValidateRange(9600, 921600)]
    [int]$Baud = 460800,
    [string]$Relay = "ws://127.0.0.1:8770/control",
    [string]$AdapterId = "domino-physical-1",
    [string]$RobotId = "domino-1"
)

$ErrorActionPreference = "Stop"

if ($Transport -eq "wifi" -and [string]::IsNullOrWhiteSpace($RobotHost)) {
    throw "Wi-Fi transport requires -RobotHost, for example 192.168.1.123."
}
if ($Transport -ne "wifi" -and [string]::IsNullOrWhiteSpace($Device)) {
    throw "$Transport transport requires -Device, for example COM5."
}
if ($Transport -ne "usb" -and [string]::IsNullOrWhiteSpace($env:DOMINO_ROBOT_LINK_KEY)) {
    throw "Wireless transport requires DOMINO_ROBOT_LINK_KEY to match src/live_robot_secrets.h."
}
if ($Transport -ne "usb" -and $env:DOMINO_ROBOT_LINK_KEY.Length -lt 16) {
    throw "DOMINO_ROBOT_LINK_KEY must contain at least 16 characters."
}

$bundledNode = Join-Path $env:USERPROFILE ".cache\codex-runtimes\codex-primary-runtime\dependencies\node\bin\node.exe"
$node = if (Test-Path -LiteralPath $bundledNode) {
    $bundledNode
} else {
    (Get-Command node -ErrorAction Stop).Source
}

$arguments = @(
    (Join-Path $PSScriptRoot "live-companion-adapter.mjs"),
    "--transport", $Transport,
    "--relay", $Relay,
    "--adapter-id", $AdapterId,
    "--robot-id", $RobotId
)
if ($Transport -eq "wifi") {
    $arguments += @("--robot-host", $RobotHost, "--robot-port", [string]$RobotPort)
} else {
    $arguments += @("--device", $Device, "--baud", [string]$Baud)
}

Write-Host "Starting Domino LIVE companion ($($Transport.ToUpperInvariant()))"
Write-Host "Relay: $Relay"
Write-Host "Robot: $(if ($Transport -eq 'wifi') { "$RobotHost`:$RobotPort" } else { "$Device @ $Baud" })"
Write-Host "Keep this terminal open. Press Ctrl+C to neutralize and stop the adapter."

& $node @arguments
exit $LASTEXITCODE
