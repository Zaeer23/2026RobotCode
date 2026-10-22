<#
.SYNOPSIS
    Points the FRC Driver Station at this project's dashboard.

.DESCRIPTION
    Follows the WPILib procedure for a custom dashboard:
    https://docs.wpilib.org/en/stable/docs/software/driverstation/manually-setting-the-driver-station-to-start-custom-dashboard.html

    The Driver Station launches whatever "DashboardCmdLine" in
    C:\Users\Public\Documents\FRC\FRC DS Data Storage.ini points at. This writes that
    line, leaving every other setting (team number, window position, practice timings)
    exactly as it was.

    This is a script rather than a one-off edit for one reason: the Driver Station
    rewrites that file from memory when it closes. Edit it while the DS is open and the
    change is silently thrown away on exit, which is what happened the first time. So the
    script refuses to run while the DS is up, and you can re-run it any time the setting
    gets reset.

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File tools\setup-ds-dashboard.ps1
#>

[CmdletBinding()]
param(
    # Set the Dashboard Type as well. The WPILib page says to set it to "Default" in the
    # DS Setup tab; the numeric value for that is not documented and has differed between
    # DS versions, so by default this leaves it alone and just tells you what it is.
    [int] $DashboardType = -1
)

$ErrorActionPreference = 'Stop'

$ini = 'C:\Users\Public\Documents\FRC\FRC DS Data Storage.ini'
$projectRoot = Split-Path -Parent $PSScriptRoot
$jar = Join-Path $projectRoot 'build\dashboard\dashboard.jar'
$natives = Join-Path $projectRoot 'build\dashboard\natives'
$javaw = 'C:\Users\Public\wpilib\2026\jdk\bin\javaw.exe'

# --- refuse to run at a moment when the edit would be thrown away -----------------------

$ds = Get-Process -Name 'DriverStation' -ErrorAction SilentlyContinue
if ($ds) {
    Write-Host ''
    Write-Host '  The Driver Station is running.' -ForegroundColor Yellow
    Write-Host '  It rewrites this file when it closes, so an edit made now would be lost.'
    Write-Host '  Close the Driver Station, then run this again.'
    Write-Host ''
    exit 1
}

if (-not (Test-Path $ini)) {
    throw "No Driver Station settings file at $ini. Has the DS ever been run on this machine?"
}

# --- check there is actually something to launch ----------------------------------------

if (-not (Test-Path $jar)) {
    Write-Host ''
    Write-Host '  The dashboard has not been built yet.' -ForegroundColor Yellow
    Write-Host '  Run this first, then run this script again:'
    Write-Host ''
    Write-Host '      .\gradlew dashboardBundle'
    Write-Host ''
    exit 1
}
if (-not (Test-Path $javaw)) {
    throw "No javaw.exe at $javaw. Point `$javaw at your WPILib JDK."
}

# javaw rather than java so no console window opens behind the dashboard, and the natives
# directory is passed explicitly because NetworkTables needs ntcore.dll beside it.
$command = "$javaw -Djava.library.path=$natives -jar $jar"

# The file stores Windows paths with every backslash doubled.
$escaped = $command -replace '\\', '\\'

# --- rewrite only the lines we own ------------------------------------------------------

$backup = "$ini.backup-$(Get-Date -Format yyyyMMdd-HHmmss)"
Copy-Item $ini $backup
Write-Host "  Backed up to $(Split-Path -Leaf $backup)" -ForegroundColor DarkGray

$lines = Get-Content $ini
$sawCmdLine = $false
$currentType = '(not set)'

$updated = foreach ($line in $lines) {
    if ($line -match '^\s*DashboardCmdLine\s*=') {
        $sawCmdLine = $true
        "DashboardCmdLine = `"$escaped`""
    }
    elseif ($line -match '^\s*DashboardType\s*=\s*(\d+)') {
        $currentType = $Matches[1]
        if ($DashboardType -ge 0) { "DashboardType = $DashboardType    " } else { $line }
    }
    else { $line }
}

if (-not $sawCmdLine) {
    # No such line yet: add it under [Setup] so the DS finds it where it expects.
    $updated = $updated | ForEach-Object {
        $_
        if ($_ -match '^\s*\[Setup\]') { "DashboardCmdLine = `"$escaped`"" }
    }
}

# CRLF, matching what the DS itself writes.
[System.IO.File]::WriteAllText($ini, ($updated -join "`r`n") + "`r`n")

Write-Host ''
Write-Host '  Driver Station will now launch:' -ForegroundColor Green
Write-Host "      $command"
Write-Host ''
if ($DashboardType -ge 0) {
    Write-Host "  Dashboard Type set to $DashboardType (was $currentType)."
} else {
    Write-Host "  Dashboard Type left at $currentType."
    Write-Host '  If the DS launches its own dashboard instead of ours, open the DS Setup tab'
    Write-Host '  (the gear) and set Dashboard to "Default", close the DS, and run this again.'
}
Write-Host ''
