$ErrorActionPreference = 'Stop'
$projectRoot = $PSScriptRoot
$cmake = 'C:\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'
$executable = Join-Path $projectRoot 'out\build\vs2022-x64\Release\pendulum_console.exe'
$buildLog = Join-Path $projectRoot 'out\launcher-build.log'

Push-Location $projectRoot
try {
    New-Item -ItemType Directory -Path (Join-Path $projectRoot 'out') -Force | Out-Null
    Write-Host 'Preparing PendulumLab...'
    & $cmake --preset vs2022-x64 *> $buildLog
    if ($LASTEXITCODE -ne 0) { throw "Configure failed. See $buildLog" }
    & $cmake --build --preset release --target pendulum_console *>> $buildLog
    if ($LASTEXITCODE -ne 0) { throw "Build failed. See $buildLog" }
    & $executable --config (Join-Path $projectRoot 'config\config.json')
    exit $LASTEXITCODE
}
finally {
    Pop-Location
}
