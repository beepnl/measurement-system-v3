echo off

REM compilerVersion="SEGGER Embedded Studio for ARM 3.40"
REM SET compilerVersion="SEGGER Embedded Studio for ARM 3.34b"
SET compilerVersion="SEGGER Embedded Studio for ARM 4.30a"
SET build=Release
SET compileDir=Release
if exist "../../PRJ/ses/Output"		RMDIR /S /Q "../../PRJ/ses/Output"
if exist "../../BTLDR/ses/Output"	RMDIR /S /Q "../../BTLDR/ses/Output"


echo Removing all previous intermediate and output files to prevent confusion when compilation fails without the user noticing.

if exist %compileDir% 				RMDIR /S /Q %compileDir%
MD %compileDir%
echo Clean-up complete


start /B /wait ../nrfutil.exe version

start /B /wait C:/"Program Files"/SEGGER/%compilerVersion%/bin/emBuild.exe -config "%build%" ../../PRJ/ses/Beepbase.emProject
if %ERRORLEVEL% NEQ 0 (
	echo Application compilation failed with error: %ERRORLEVEL%
	pause
	exit
)
echo Application compiled

start /B /wait C:/"Program Files"/SEGGER/%compilerVersion%/bin/emBuild.exe -config "%build%" ../../BTLDR/ses/beep_bootloader.emProject
if %ERRORLEVEL% NEQ 0 (
	echo Bootloader compilation failed with error: %ERRORLEVEL%
	pause
	exit
)
echo Bootloader compiled

start /B /wait ../nrfutil.exe settings generate --family NRF52840 --application ../../PRJ/ses/Output/%build%/Exe/BeepBase.hex --application-version 1 --bootloader-version 1 --bl-settings-version 1 "%compileDir%/BL_settings.hex"
if %ERRORLEVEL% NEQ 0 (
	echo Merging failed with error: %ERRORLEVEL%
	pause
	exit
)
echo application settings generated.

start /B /wait ../mergehex.exe --merge %compileDir%/BL_settings.hex ../../BTLDR/ses/Output/%build%/Exe/beep_bootloader.hex --output %compileDir%/BTLDR_settings.hex
if %ERRORLEVEL% NEQ 0 (
	echo Merging Bootloader + settings with error: %ERRORLEVEL%
	pause
	exit
)

start /B /wait ../mergehex.exe --merge ../../PRJ/ses/Output/%build%/Exe/BeepBase.hex ../../nRF5_SDK_15.3/components/softdevice/s140/hex/s140_nrf52_6.1.1_softdevice.hex --output %compileDir%/APP_SD.hex
if %ERRORLEVEL% NEQ 0 (
	echo Merging APP_SD.hex failed with error: %ERRORLEVEL%
	pause
	exit
)

start /B /wait ../mergehex.exe --merge ../../BTLDR/ses/Output/%build%/Exe/beep_bootloader.hex %compileDir%/APP_SD.hex --output %compileDir%/APP_SD_BL.hex
if %ERRORLEVEL% NEQ 0 (
	echo Merging APP_SD_BL.hex failed with error: %ERRORLEVEL%
	pause
	exit
)

start /B /wait ../mergehex.exe --merge %compileDir%/BL_settings.hex %compileDir%/App_SD_BL.hex --output %compileDir%/Beepbase.hex
if %ERRORLEVEL% NEQ 0 (
	echo Merging Beepbase.hex with BL_settings.hex failed  with error: %ERRORLEVEL%
	pause
	exit
)

start /B /wait ../nrfutil.exe pkg generate --hw-version 52 --application-version 1 --application ../../PRJ/ses/Output/%build%/Exe/BeepBase.hex --sd-req 0xB6 --key-file ../../Key/private.key %compileDir%/Beepbase_app.zip 
if %ERRORLEVEL% NEQ 0 (
	echo Creating Beepbase_app.zip failed with error: %ERRORLEVEL%
	pause
	exit
)

echo --bootloader ../../BTLDR/ses/Output/%build%/Exe/beep_bootloader.hex --bootloader-version 1
start /B /wait ../nrfutil.exe pkg generate --application ../../PRJ/ses/Output/%build%/Exe/Beepbase.hex --application-version 1  --bootloader ../../BTLDR/ses/Output/%build%/Exe/beep_bootloader.hex --bootloader-version 1 --softdevice ../../nRF5_SDK_15.3/components/softdevice/s140/hex/s140_nrf52_6.1.1_softdevice.hex --hw-version 52 --sd-req 0xB6 --sd-id 0xB6 --key-file ../../Key/private.key %compileDir%/Beepbase_sd_boot_app.zip
if %ERRORLEVEL% NEQ 0 (
	echo Merging SD+BOOT+APP failed with error: %ERRORLEVEL%
	pause
	exit
)

del /F %compileDir%\BL_settings.hex
del /F %compileDir%\BTLDR_settings.hex
del /F %compileDir%\APP_SD.hex
del /F %compileDir%\APP_SD_BL.hex

pause