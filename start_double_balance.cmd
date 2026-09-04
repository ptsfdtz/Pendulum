@echo off
setlocal
cd /d "%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -File ".\scripts\start_double_balance.ps1"
echo.
echo Press any key to close...
pause >nul
endlocal
