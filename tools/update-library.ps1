# Updates the HHCode library in a team's folder from a newer copy of the
# template, without touching the team's own files. Windows version.
#
# 1. Unzip the new template somewhere (for example Downloads\HHCode-new).
# 2. Open YOUR TEAM'S folder in VS Code and open a terminal (Terminal > New Terminal).
# 3. Run:   .\tools\update-library.ps1 -From "C:\Users\you\Downloads\HHCode-new"
#
# Replaced:  the library (include\hh, src\hh, include\JAR-Template,
#            src\JAR-Template), the VEX build files, the docs, the tests and
#            these tools.
# Kept:      src\robot-config.cpp, include\robot-config.h, src\main.cpp,
#            include\autons.h, your autons and PID constants in src\autons,
#            and everything in the subsystems folders.
#
# The old library is saved in a folder named _library-backup-<date> first.
param(
  [Parameter(Mandatory = $true)][string]$From
)

$ErrorActionPreference = 'Stop'
$team = Split-Path $PSScriptRoot -Parent
$new = (Resolve-Path $From).Path

# Folders are replaced whole, so files removed from the library go away too.
$folders = 'include\hh', 'src\hh', 'include\JAR-Template', 'src\JAR-Template', 'vex', 'docs', 'tests\sim', 'tools'
$files = 'makefile', 'include\vex.h', 'src\autons\tests.cpp', 'tests\README.md', 'tests\run.ps1', 'tests\run.sh', 'README.md', 'CLAUDE.md'
# Team files that sometimes need a matching change when the library changes.
$watch = 'src\main.cpp', 'src\autons\auton_list.cpp', 'src\autons\constants.cpp', 'include\autons.h', 'include\robot-config.h', 'src\robot-config.cpp'

if (-not (Test-Path (Join-Path $new 'makefile')) -or -not (Test-Path (Join-Path $new 'include\hh'))) {
  Write-Host "That doesn't look like an HHCode template folder: $new"
  Write-Host "Pick the folder that has 'makefile' and 'include' directly inside it."
  exit 1
}
if ($new -eq $team) {
  Write-Host "-From is this same folder. Point it at the NEW template instead."
  exit 1
}

$backup = Join-Path $team ("_library-backup-" + (Get-Date -Format 'yyyy-MM-dd-HHmm'))
New-Item -ItemType Directory -Force $backup | Out-Null

foreach ($item in $folders + $files) {
  $source = Join-Path $new $item
  $target = Join-Path $team $item
  if (-not (Test-Path $source)) { continue }
  if (Test-Path $target) {
    $saved = Join-Path $backup $item
    New-Item -ItemType Directory -Force (Split-Path $saved -Parent) | Out-Null
    Copy-Item $target $saved -Recurse -Force
    if ($item -ne 'tools') { Remove-Item $target -Recurse -Force }
  }
  New-Item -ItemType Directory -Force (Split-Path $target -Parent) | Out-Null
  if ((Test-Path $source -PathType Container) -and (Test-Path $target)) {
    Copy-Item (Join-Path $source '*') $target -Recurse -Force   # tools: this script is running from here
  } else {
    Copy-Item $source $target -Recurse -Force
  }
  Write-Host "updated  $item"
}

Write-Host ""
Write-Host "Kept your own files. These ones differ from the new template, which is"
Write-Host "normal, but look at the new template's copy if the build fails:"
foreach ($item in $watch) {
  $source = Join-Path $new $item
  $target = Join-Path $team $item
  if ((Test-Path $source) -and (Test-Path $target)) {
    if ((Get-FileHash $source).Hash -ne (Get-FileHash $target).Hash) { Write-Host "  $item" }
  }
}
Write-Host ""
Write-Host "Done. Your old library is in $(Split-Path $backup -Leaf)."
Write-Host "Now click Build. Delete the backup folder once everything works."
