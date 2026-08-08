@echo off
setlocal
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0cubemx_post_generate.ps1"
exit /b %ERRORLEVEL%
