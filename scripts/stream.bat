@echo off
setlocal EnableDelayedExpansion

:: --- Configure your RTSP URL here ---
set "URL=rtsp://192.168.20.3:8554/spot-stream"

echo Select resolution (default: 1080):
echo   1) 720p
echo   2) 1080p
echo   3) 2k (2560x1440)
set /p choice=Enter choice [1-3]:

if "%choice%"=="" set choice=2

set "RES=1920:1080"
if "%choice%"=="1" set "RES=1280:720"
if "%choice%"=="2" set "RES=1920:1080"
if "%choice%"=="3" set "RES=2560:1440"

:: Warn if ffplay not found
where ffplay >nul 2>&1
if errorlevel 1 (
  echo.
  echo ERROR: ffplay not found in PATH. Install FFmpeg and add its bin folder to PATH.
  echo.
  pause
  exit /b 1
)

:: Make height slightly smaller so the top bar doesn't hide content
set "MARGIN=60"

:: Parse RES into width/height
for /f "tokens=1,2 delims=:" %%A in ("!RES!") do (
  set "W=%%A"
  set "H=%%B"
)

:: Adjusted height = H - MARGIN, clamp to minimum of 2, and make even
set /a HADJ=H-MARGIN
if !HADJ! lss 2 set /a HADJ=2
set /a HADJ=HADJ - (HADJ %% 2)

:: Scaling factor
set "SCALE=!W!:!HADJ!"

:: Play. Also crop left half, then scale to adjusted height
ffplay -fflags nobuffer -flags low_delay -framedrop -strict experimental -rtsp_transport tcp -probesize 32 -analyzeduration 0 -vf "crop=iw/2:ih:0:0,scale=!SCALE!" "%URL%"
