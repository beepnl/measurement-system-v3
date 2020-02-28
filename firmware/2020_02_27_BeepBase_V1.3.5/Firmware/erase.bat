@ECHO OFF
SET jlink_id=682613435

ECHO Erase nRF52840
start /B /wait nrfjprog --snr %jlink_id% --recover
if %ERRORLEVEL% NEQ 0 (
	echo Is the debugger connected to the board and is the baord powered: %ERRORLEVEL%
	pause
	exit
)
start /B /wait nrfjprog --snr %jlink_id% --eraseall
start /B /wait nrfjprog --snr %jlink_id% --reset
ECHO Erasing Done

pause
