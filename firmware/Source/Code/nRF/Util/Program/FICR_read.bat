@ECHO OFF
SET jlink_id=682613435

ECHO Read FICR registers
ECHO INFO.PART:
start /B /wait nrfjprog --snr %jlink_id% --memrd 0x10000100 --w 32
ECHO INFO.VARIANT:
start /B /wait nrfjprog --snr %jlink_id% --memrd 0x10000104 --w 32
ECHO INFO.PACKAGE:
start /B /wait nrfjprog --snr %jlink_id% --memrd 0x10000108 --w 32
ECHO INFO.RAM:
start /B /wait nrfjprog --snr %jlink_id% --memrd 0x1000010C --w 32
ECHO INFO.FLASH:
start /B /wait nrfjprog --snr %jlink_id% --memrd 0x10000110 --w 32

GOTO End
:End
pause