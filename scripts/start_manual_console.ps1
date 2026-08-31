param(
    [string]$Config = "",
    [switch]$Build
)

$projectRoot = Split-Path -Parent $PSScriptRoot
$cmake = "C:\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
$executable = Join-Path $projectRoot "out\build\vs2022-x64\Release\pendulum_manual_console.exe"

if ($Build -or -not (Test-Path -LiteralPath $executable)) {
    & $cmake --preset vs2022-x64
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & $cmake --build --preset release
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

if ([string]::IsNullOrWhiteSpace($Config)) {
    $Config = Join-Path $projectRoot "config\config.json"
}

Push-Location $projectRoot
try {
    & $executable --config $Config
    exit $LASTEXITCODE
}
finally {
    Pop-Location
}
