@ECHO OFF
SET hw_major=1
SET hw_minor=0
SET hw_ID=190222
SET /A hw_reg_val=%hw_major%*65536 + %hw_minor%
SET jlink_id=682613435

ECHO Start programming HW %hw_major%.%hw_minor%; reg:%hw_reg_val%
start /B /wait nrfjprog --snr %jlink_id% --eraseall
if %ERRORLEVEL% NEQ 0 (
	echo Erasing failed with error: %ERRORLEVEL%
	pause
	exit
)

start /B /wait nrfjprog --snr %jlink_id% --memwr 0x10001080 --val %hw_reg_val%
if %ERRORLEVEL% NEQ 0 (
	echo programming HW failed with error: %ERRORLEVEL%
	pause
	exit
)

start /B /wait nrfjprog --snr %jlink_id% --memwr 0x10001084 --val %hw_ID%
if %ERRORLEVEL% NEQ 0 (
	echo Hardware ID failed with error: %ERRORLEVEL%
	pause
	exit
)

start /B /wait nrfjprog --snr %jlink_id% --program Release/Beepbase.hex
if %ERRORLEVEL% NEQ 0 (
	echo Programming hex file failed with error: %ERRORLEVEL%
	pause
	exit
)

start /B /wait nrfjprog --snr %jlink_id% --reset
if %ERRORLEVEL% NEQ 0 (
	echo Reseting failed with error: %ERRORLEVEL%
	pause
	exit
)

ECHO Programming Done

GOTO End
:End
pause
