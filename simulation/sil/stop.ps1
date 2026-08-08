$ErrorActionPreference = "Stop"

$runtimeDirectory = Join-Path $PSScriptRoot "viewer\runtime"

function Stop-DominoProcess {
    param([string]$PidFile)

    if (-not (Test-Path -LiteralPath $PidFile)) {
        return
    }

    $processId = Get-Content -LiteralPath $PidFile -ErrorAction SilentlyContinue
    if ($processId -match "^\d+$") {
        $process = Get-Process -Id ([int]$processId) -ErrorAction SilentlyContinue
        if ($null -ne $process) {
            Stop-Process -Id $process.Id
            $process.WaitForExit()
        }
    }
    Remove-Item -LiteralPath $PidFile -ErrorAction SilentlyContinue
}

Stop-DominoProcess (Join-Path $runtimeDirectory "simulator.pid")
Stop-DominoProcess (Join-Path $runtimeDirectory "viewer.pid")
