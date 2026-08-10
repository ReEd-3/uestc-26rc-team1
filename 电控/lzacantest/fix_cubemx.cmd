@echo off
cd /d "%~dp0"
powershell -ExecutionPolicy Bypass -File "%~dp0fix_cubemx_cmake_project.ps1"
echo.
echo Press any key to exit...
pause >nul
