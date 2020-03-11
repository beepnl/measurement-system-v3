@ECHO OFF
SET jlink_id=682613435

ECHO Enabeling readback protection

start /B /wait nrfjprog --snr %jlink_id% --rbp ALL
if %ERRORLEVEL% NEQ 0 (
ECHO Read back protection could not be enabled or is already enabled
pause
exit 1
)

start /B /wait nrfjprog --snr %jlink_id% --pinreset
if %ERRORLEVEL% NEQ 0 (
ECHO Device could not be reset
pause
exit 2
)
pause
