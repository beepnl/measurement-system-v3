@ECHO OFF
SET jlink_id=682613435

ECHO Erase nRF52832
start /B /wait nrfjprog --snr %jlink_id% --recover
start /B /wait nrfjprog --snr %jlink_id% --eraseall
start /B /wait nrfjprog --snr %jlink_id% --reset
ECHO Erasing Done

pause
