@echo off
chcp 65001 >nul
cd /d "%~dp0"
where pythonw.exe >nul 2>nul
if %errorlevel% equ 0 (
    start "DM-G6220 GUI" pythonw.exe dm_g6220_memory_gui.py
) else (
    start "DM-G6220 GUI" "D:\PYTHON\pythonw.exe" dm_g6220_memory_gui.py
)
