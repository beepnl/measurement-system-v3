@ECHO OFF
ECHO Erase 
start /B /wait nrfjprog --recover
start /B /wait nrfjprog --eraseall
start /B /wait nrfjprog --reset
ECHO Erasing Done

pause
