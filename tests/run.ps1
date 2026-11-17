# Builds and runs the simulator tests on Windows.
#
#   .\tests\run.ps1                      uses g++ or clang++ if one is on the PATH
#   .\tests\run.ps1 -Compiler "zig c++"  or name the compiler yourself
#   .\tests\run.ps1 -Only 2              one robot only (1 to 4)
param(
  [string]$Compiler = $env:CXX,
  [int[]]$Only = @(1, 2, 3, 4)
)

Set-Location (Split-Path $PSScriptRoot -Parent)

if (-not $Compiler) {
  foreach ($name in 'g++', 'clang++') {
    if (Get-Command $name -ErrorAction SilentlyContinue) { $Compiler = $name; break }
  }
}
if (-not $Compiler) {
  Write-Host "No C++ compiler found. Install one, or pass -Compiler. See tests/README.md."
  exit 2
}
$parts = $Compiler -split ' '
$exe = $parts[0]
$exeArgs = @($parts | Select-Object -Skip 1)

New-Item -ItemType Directory -Force build\sim | Out-Null
$sources = @(Get-ChildItem src\JAR-Template\*.cpp, src\hh\*.cpp, src\autons\*.cpp, src\main.cpp, tests\sim\*.cpp | ForEach-Object { $_.FullName })

$failed = $false
foreach ($config in $Only) {
  Write-Host "=== Robot ${config}: building"
  $out = "build\sim\sim$config.exe"
  & $exe @exeArgs -std=gnu++17 -O1 -pthread -Dmain=robot_main "-DSIM_CONFIG=$config" `
    -Itests/sim/vex -Itests/sim -Iinclude @sources -o $out
  if ($LASTEXITCODE -ne 0) { $failed = $true; continue }
  Write-Host "=== Robot ${config}: running"
  & ".\$out"
  if ($LASTEXITCODE -ne 0) { $failed = $true }
}

if ($failed) { Write-Host "SOME TESTS FAILED"; exit 1 }
Write-Host "ALL TESTS PASSED"
