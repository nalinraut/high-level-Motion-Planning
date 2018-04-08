set l_temp=%1
set l_bootFile=%2
set l_target=
REM Management of known target : bootline.dat is kept unchanged
IF "%l_bootFile%"=="men007" set l_target=men007
IF "%l_bootFile%"=="men07n" set l_target=men07n
IF "%l_bootFile%"=="men" set l_target=men
IF "%l_bootFile%"=="men_hp" set l_target=men_hp
IF "%l_bootFile%"=="teknor" set l_target=teknor
IF NOT "%l_target%"=="" GOTO END
REM Management of transition vxworks to known target : bootline.dat will be updated
IF "%l_bootFile%"=="vxworks" GOTO SERIAL
IF "%l_bootFile%"=="vxWorks" GOTO SERIAL
GOTO END
:SERIAL
REM force the update of bootline.dat
set l_keep=false
REM Try to identity the target using the serial line name
IF "%3"=="gei" set l_target=men
IF "%3"=="fei" set l_target=teknor
IF "%4"=="gei" set l_target=men
IF "%4"=="fei" set l_target=teknor
IF "%5"=="gei" set l_target=men
IF "%5"=="fei" set l_target=teknor
IF "%6"=="gei" set l_target=men
IF "%6"=="fei" set l_target=teknor
IF "%7"=="gei" set l_target=men
IF "%7"=="fei" set l_target=teknor
IF "%8"=="gei" set l_target=men
IF "%8"=="fei" set l_target=teknor
IF "%9"=="gei" set l_target=men
IF "%9"=="fei" set l_target=teknor
IF NOT "%l_target%"=="teknor" GOTO END
REM Make sure bootrom.sys can be sent by Ftp in contiguous blocks
ECHO.
ECHO ATTENTION! Update requires version s3.2 or later version.
ECHO If the CS8 is in s3.1 or earlier version, first update to s3.2 or s4.0, reboot, then try again.
ECHO Press Ctrl+C to stop update.
PAUSE
:END
