@echo off
rem Double-click launcher for the real-vs-sim flight log comparison app.
rem Opens file pickers (real log first, then sim log), generates the HTML
rem report next to the real log and opens it in the browser.
cd /d "%~dp0"
python compare_logs.py %*
if errorlevel 1 pause
