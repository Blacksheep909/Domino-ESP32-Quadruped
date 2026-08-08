$ErrorActionPreference = "Stop"

$runtimeDirectory = Join-Path $PSScriptRoot "runtime"

function Stop-DominoStandaloneProcess {
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

Stop-DominoStandaloneProcess (Join-Path $runtimeDirectory "firmware.pid")
Stop-DominoStandaloneProcess (Join-Path $runtimeDirectory "server.pid")
