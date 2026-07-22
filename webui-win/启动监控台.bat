@echo off
echo Starting sensor bridge control panel...
echo Open browser at http://localhost:8080
echo Close this window to stop.
echo.
"C:\Users\Tutul\AppData\Local\Programs\Python\Python312\python.exe" "%~dp0server.py"
pause
