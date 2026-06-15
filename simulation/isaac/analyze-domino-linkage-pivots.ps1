param(
    [string]$UrdfPath,
    [string]$MarkdownReportPath,
    [string]$JsonReportPath
)

$ErrorActionPreference = "Stop"

$isaacDir = $PSScriptRoot
if (-not $UrdfPath -or $UrdfPath.Trim().Length -eq 0) {
    $UrdfPath = Join-Path $isaacDir "..\urdf\generated\Domino_URDF_Parts_Combined_Final_description\urdf\Domino_URDF_Parts_Combined_Final.urdf"
}
if (-not $MarkdownReportPath -or $MarkdownReportPath.Trim().Length -eq 0) {
    $MarkdownReportPath = Join-Path $isaacDir "reports\domino-linkage-pivots.md"
}
if (-not $JsonReportPath -or $JsonReportPath.Trim().Length -eq 0) {
    $JsonReportPath = Join-Path $isaacDir "reports\domino-linkage-pivots.json"
}

function ConvertTo-DoubleVector {
    param([string]$Text)
    if (-not $Text -or $Text.Trim().Length -eq 0) {
        return @()
    }
    return @($Text.Trim() -split "\s+" | ForEach-Object { [double]$_ })
}

function Add-Vector {
    param([double[]]$A, [double[]]$B)
    return @(
        [double]($A[0] + $B[0]),
        [double]($A[1] + $B[1]),
        [double]($A[2] + $B[2])
    )
}

function Subtract-Vector {
    param([double[]]$A, [double[]]$B)
    return @(
        [double]($A[0] - $B[0]),
        [double]($A[1] - $B[1]),
        [double]($A[2] - $B[2])
    )
}

function Average-Vector {
    param([double[][]]$Vectors)
    $count = [double]$Vectors.Count
    return @(
        [double](($Vectors | ForEach-Object { $_[0] } | Measure-Object -Sum).Sum / $count),
        [double](($Vectors | ForEach-Object { $_[1] } | Measure-Object -Sum).Sum / $count),
        [double](($Vectors | ForEach-Object { $_[2] } | Measure-Object -Sum).Sum / $count)
    )
}

function Get-Distance {
    param([double[]]$A, [double[]]$B)
    $dx = $A[0] - $B[0]
    $dy = $A[1] - $B[1]
    $dz = $A[2] - $B[2]
    return [Math]::Sqrt(($dx * $dx) + ($dy * $dy) + ($dz * $dz))
}

function Format-Vector {
    param([double[]]$Vector)
    if (-not $Vector -or $Vector.Count -lt 3) {
        return "-"
    }
    return ("{0:N6}, {1:N6}, {2:N6}" -f $Vector[0], $Vector[1], $Vector[2])
}

function Format-PlaneVector {
    param([double[]]$Vector)
    if (-not $Vector -or $Vector.Count -lt 3) {
        return "-"
    }
    return ("{0:N6}, {1:N6}" -f $Vector[0], $Vector[2])
}

$resolvedUrdf = Resolve-Path $UrdfPath
[xml]$doc = Get-Content -Raw -LiteralPath $resolvedUrdf

$jointRows = @($doc.robot.joint | ForEach-Object {
    $origin = ConvertTo-DoubleVector $_.origin.xyz
    $rpy = ConvertTo-DoubleVector $_.origin.rpy
    $axis = ConvertTo-DoubleVector $_.axis.xyz
    $lower = $null
    $upper = $null
    if ($_.limit) {
        $lower = [double]$_.limit.lower
        $upper = [double]$_.limit.upper
    }
    [pscustomobject]@{
        Name = [string]$_.name
        Type = [string]$_.type
        Parent = [string]$_.parent.link
        Child = [string]$_.child.link
        Origin = $origin
        Rpy = $rpy
        Axis = $axis
        LimitLower = $lower
        LimitUpper = $upper
        ParentFrameWorld = $null
        PivotWorld = $null
    }
})

$jointByName = @{}
foreach ($joint in $jointRows) {
    $jointByName[$joint.Name] = $joint
}

$worldFrames = @{
    "base_link" = @([double]0.0, [double]0.0, [double]0.0)
}
$frameSources = @{
    "base_link" = "root"
}

$changed = $true
while ($changed) {
    $changed = $false
    foreach ($joint in $jointRows) {
        if (-not $worldFrames.ContainsKey($joint.Parent)) {
            continue
        }

        $parentFrame = [double[]]$worldFrames[$joint.Parent]
        $joint.ParentFrameWorld = $parentFrame
        $joint.PivotWorld = Add-Vector $parentFrame ([double[]]$joint.Origin)

        if (-not $worldFrames.ContainsKey($joint.Child)) {
            $worldFrames[$joint.Child] = $joint.PivotWorld
            $frameSources[$joint.Child] = $joint.Name
            $changed = $true
        }
    }
}

$nonZeroRpy = @($jointRows | Where-Object {
    $_.Rpy.Count -eq 3 -and (([Math]::Abs($_.Rpy[0]) -gt 1e-9) -or ([Math]::Abs($_.Rpy[1]) -gt 1e-9) -or ([Math]::Abs($_.Rpy[2]) -gt 1e-9))
})

$clusters = @(
    [pscustomobject]@{
        Id = "dom_p_4_1_lower_triangle"
        HipLink = "DOM_P__4__1"
        Description = "One lower-linkage loop on the DOM_P__4__1 leg. This is the first CAD-derived one-joint Isaac target."
        Joints = @("Revolute 59", "Revolute 43", "Revolute 33", "Revolute 26", "Revolute 25")
        ClosurePairs = @([pscustomobject]@{ JointA = "Revolute 25"; JointB = "Revolute 26" })
    },
    [pscustomobject]@{
        Id = "dom_p_4_1_upper_loop"
        HipLink = "DOM_P__4__1"
        Description = "Upper/lower leg loop that shares the DOM_P__4__1 leg pitch pivots."
        Joints = @("Revolute 58", "Revolute 59", "Revolute 43", "Revolute 32", "Revolute 51")
        ClosurePairs = @([pscustomobject]@{ JointA = "Revolute 32"; JointB = "Revolute 51" })
    },
    [pscustomobject]@{
        Id = "dom_p_12_1_lower_triangle"
        HipLink = "DOM_P__12__1"
        Description = "Mirrored lower-linkage loop on the DOM_P__12__1 leg."
        Joints = @("Revolute 46", "Revolute 44", "Revolute 36", "Revolute 24", "Revolute 23")
        ClosurePairs = @([pscustomobject]@{ JointA = "Revolute 23"; JointB = "Revolute 24" })
    },
    [pscustomobject]@{
        Id = "dom_p_12_1_upper_loop"
        HipLink = "DOM_P__12__1"
        Description = "Mirrored upper/lower loop on the DOM_P__12__1 leg."
        Joints = @("Revolute 55", "Revolute 46", "Revolute 44", "Revolute 29", "Revolute 50")
        ClosurePairs = @([pscustomobject]@{ JointA = "Revolute 29"; JointB = "Revolute 50" })
    },
    [pscustomobject]@{
        Id = "dom_p_25_1_lower_triangle"
        HipLink = "DOM_P__25__1"
        Description = "Mirrored lower-linkage loop on the DOM_P__25__1 leg. The CAD export currently marks the lower input joint as continuous."
        Joints = @("Revolute 47", "Revolute 45", "Revolute 35", "Revolute 22", "Revolute 21")
        ClosurePairs = @([pscustomobject]@{ JointA = "Revolute 21"; JointB = "Revolute 22" })
    },
    [pscustomobject]@{
        Id = "dom_p_25_1_upper_loop"
        HipLink = "DOM_P__25__1"
        Description = "Mirrored upper/lower loop on the DOM_P__25__1 leg."
        Joints = @("Revolute 56", "Revolute 47", "Revolute 45", "Revolute 34", "Revolute 54")
        ClosurePairs = @([pscustomobject]@{ JointA = "Revolute 34"; JointB = "Revolute 54" })
    },
    [pscustomobject]@{
        Id = "dom_p_21_1_lower_triangle"
        HipLink = "DOM_P__21__1"
        Description = "Mirrored lower-linkage loop on the DOM_P__21__1 leg."
        Joints = @("Revolute 48", "Revolute 42", "Revolute 37", "Revolute 28", "Revolute 27")
        ClosurePairs = @([pscustomobject]@{ JointA = "Revolute 27"; JointB = "Revolute 28" })
    },
    [pscustomobject]@{
        Id = "dom_p_21_1_upper_loop"
        HipLink = "DOM_P__21__1"
        Description = "Mirrored upper/lower loop on the DOM_P__21__1 leg."
        Joints = @("Revolute 57", "Revolute 48", "Revolute 42", "Revolute 31", "Revolute 53")
        ClosurePairs = @([pscustomobject]@{ JointA = "Revolute 31"; JointB = "Revolute 53" })
    }
)

$clusterReports = @()
foreach ($cluster in $clusters) {
    $hipFrame = $null
    if ($worldFrames.ContainsKey($cluster.HipLink)) {
        $hipFrame = [double[]]$worldFrames[$cluster.HipLink]
    }

    $pivotRows = @()
    foreach ($jointName in $cluster.Joints) {
        $joint = $jointByName[$jointName]
        $relative = $null
        if ($joint -and $joint.PivotWorld -and $hipFrame) {
            $relative = Subtract-Vector ([double[]]$joint.PivotWorld) $hipFrame
        }
        $pivotRows += [pscustomobject]@{
            Joint = $jointName
            Type = if ($joint) { $joint.Type } else { $null }
            Parent = if ($joint) { $joint.Parent } else { $null }
            Child = if ($joint) { $joint.Child } else { $null }
            Axis = if ($joint) { $joint.Axis } else { $null }
            LimitLower = if ($joint) { $joint.LimitLower } else { $null }
            LimitUpper = if ($joint) { $joint.LimitUpper } else { $null }
            PivotWorldM = if ($joint) { $joint.PivotWorld } else { $null }
            PivotRelativeToHipM = $relative
            PivotRelativeXZToHipM = if ($relative) { @([double]$relative[0], [double]$relative[2]) } else { $null }
        }
    }

    $closureRows = @()
    foreach ($pair in @($cluster.ClosurePairs)) {
        $jointA = $jointByName[$pair.JointA]
        $jointB = $jointByName[$pair.JointB]
        $distance = $null
        $midpoint = $null
        if ($jointA -and $jointB -and $jointA.PivotWorld -and $jointB.PivotWorld) {
            $distance = Get-Distance ([double[]]$jointA.PivotWorld) ([double[]]$jointB.PivotWorld)
            $midpoint = Average-Vector @(([double[]]$jointA.PivotWorld), ([double[]]$jointB.PivotWorld))
        }
        $closureRows += [pscustomobject]@{
            JointA = $pair.JointA
            JointB = $pair.JointB
            DistanceM = $distance
            MidpointWorldM = $midpoint
        }
    }

    $clusterReports += [pscustomobject]@{
        Id = $cluster.Id
        HipLink = $cluster.HipLink
        Description = $cluster.Description
        HipFrameWorldM = $hipFrame
        Pivots = $pivotRows
        ClosureChecks = $closureRows
    }
}

$json = [pscustomobject]@{
    Source = "simulation/urdf/generated/Domino_URDF_Parts_Combined_Final_description/urdf/Domino_URDF_Parts_Combined_Final.urdf"
    Assumptions = @(
        "URDF joint origin rpy values are zero in this export, so pivot positions are accumulated by translation only.",
        "Repeated child link names are treated as loop-closure evidence; this script reports closure pivot agreement rather than creating an Isaac articulation."
    )
    NonZeroJointOriginRpyCount = $nonZeroRpy.Count
    LinkFrameSources = $frameSources
    Clusters = $clusterReports
}

$reportDir = Split-Path -Parent $MarkdownReportPath
if ($reportDir -and -not (Test-Path -LiteralPath $reportDir)) {
    New-Item -ItemType Directory -Force -Path $reportDir | Out-Null
}
$jsonDir = Split-Path -Parent $JsonReportPath
if ($jsonDir -and -not (Test-Path -LiteralPath $jsonDir)) {
    New-Item -ItemType Directory -Force -Path $jsonDir | Out-Null
}

$lines = New-Object System.Collections.Generic.List[string]
$lines.Add("# Domino Linkage Pivot Report")
$lines.Add("")
$lines.Add("Generated by `simulation/isaac/analyze-domino-linkage-pivots.ps1`.")
$lines.Add("")
$lines.Add("This report extracts named CAD pivot points from the generated URDF so Isaac linkage tests can use Domino dimensions instead of placeholder four-bar geometry.")
$lines.Add("")
$lines.Add("## Assumptions")
$lines.Add("")
$lines.Add('- The current URDF export has zero joint-origin `rpy` values, so world pivots are accumulated by translation from `base_link`.')
$lines.Add("- Duplicate child link names are intentional CAD loop evidence here. The report checks whether both sides of a loop land on the same pivot instead of treating the URDF as a clean articulation tree.")
$lines.Add(('- Non-zero joint-origin `rpy` count: {0}' -f $nonZeroRpy.Count))
$lines.Add("")

foreach ($cluster in $clusterReports) {
    $lines.Add(("## {0}" -f $cluster.Id))
    $lines.Add("")
    $lines.Add($cluster.Description)
    $lines.Add("")
    $lines.Add(("Hip link frame in world coordinates: `{0}` m" -f (Format-Vector ([double[]]$cluster.HipFrameWorldM))))
    $lines.Add("")
    $lines.Add("| Joint | Type | Parent | Child | Axis | Limit rad | Pivot world xyz m | Pivot xz relative to hip m |")
    $lines.Add("| --- | --- | --- | --- | --- | --- | --- | --- |")
    foreach ($pivot in $cluster.Pivots) {
        $limit = "-"
        if ($null -ne $pivot.LimitLower -and $null -ne $pivot.LimitUpper) {
            $limit = ("{0:N6} to {1:N6}" -f $pivot.LimitLower, $pivot.LimitUpper)
        }
        $axis = if ($pivot.Axis) { (Format-Vector ([double[]]$pivot.Axis)) } else { "-" }
        $lines.Add(("| {0} | {1} | {2} | {3} | {4} | {5} | {6} | {7} |" -f $pivot.Joint, $pivot.Type, $pivot.Parent, $pivot.Child, $axis, $limit, (Format-Vector ([double[]]$pivot.PivotWorldM)), (Format-PlaneVector ([double[]]$pivot.PivotRelativeToHipM))))
    }
    $lines.Add("")
    $lines.Add("| Closure joints | Distance m | Midpoint world xyz m |")
    $lines.Add("| --- | ---: | --- |")
    foreach ($closure in $cluster.ClosureChecks) {
        $lines.Add(("| {0} / {1} | {2:N9} | {3} |" -f $closure.JointA, $closure.JointB, $closure.DistanceM, (Format-Vector ([double[]]$closure.MidpointWorldM))))
    }
    $lines.Add("")
}

Set-Content -LiteralPath $MarkdownReportPath -Value $lines -Encoding UTF8
Set-Content -LiteralPath $JsonReportPath -Value ($json | ConvertTo-Json -Depth 12) -Encoding UTF8
Write-Host ("Wrote {0}" -f (Resolve-Path $MarkdownReportPath))
Write-Host ("Wrote {0}" -f (Resolve-Path $JsonReportPath))
