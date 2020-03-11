echo off
REM SEGGER Embedded Studio for ARM 3.34b
start /B /wait C:/"Program Files"/SEGGER/"SEGGER Embedded Studio for ARM 4.30a"/bin/emBuild.exe -config "Release_skip_CRC" ../../BTLDR/ses/beep_bootloader.emProject
REM start /B /wait C:/"Program Files"/SEGGER/"SEGGER Embedded Studio for ARM 3.34b"/bin/emBuild.exe -config "Release_skip_CRC" ../../BTLDR/ses/beep_bootloader.emProject
if %ERRORLEVEL% NEQ 0 (
	echo Bootloader compilation failed with error: %ERRORLEVEL%
	pause
	exit
)
echo Bootloader with skipped CRC compiled


pause