param(
    [string]$Config = "",
    [double]$Duration = 0
)

$projectRoot = Split-Path -Parent $PSScriptRoot
$cmake = "C:\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
$executable = Join-Path $projectRoot "out\build\vs2022-x64\Release\pendulum_double_balance.exe"

if ([string]::IsNullOrWhiteSpace($Config)) {
    $Config = Join-Path $projectRoot "config\config.json"
}

& $cmake --preset vs2022-x64
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $cmake --build --preset release --target pendulum_double_balance
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Push-Location $projectRoot
try {
    & $executable --config $Config --duration $Duration
    exit $LASTEXITCODE
}
finally {
    Pop-Location
}
