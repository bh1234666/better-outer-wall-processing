@echo off
setlocal
set SCRIPT_DIR=%~dp0
python "%SCRIPT_DIR%better_outer_wall_processing.py" %*
exit /b %errorlevel%
