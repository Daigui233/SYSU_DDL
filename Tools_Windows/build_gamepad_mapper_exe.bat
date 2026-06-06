@echo off
setlocal

cd /d "%~dp0\.."

python -m PyInstaller ^
  --noconfirm ^
  --clean ^
  --onefile ^
  --windowed ^
  --name gamepad_mapper ^
  --distpath Tools_Windows\dist ^
  --workpath Tools_Windows\build ^
  --specpath Tools_Windows ^
  Tools_Windows\gamepad_mapper.py

if errorlevel 1 (
  echo.
  echo Build failed. Please make sure PyInstaller is installed:
  echo   python -m pip install pyinstaller
  exit /b 1
)

echo.
echo Build OK:
echo   Tools_Windows\dist\gamepad_mapper.exe
