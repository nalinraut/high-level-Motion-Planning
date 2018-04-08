@ECHO OFF
set l_ipAdress=%1
set l_login=%2
set l_password=%3
set l_target=""

REM Set temp directory
set l_temp="%TEMP%"\cs8_backup\%1

IF "%l_ipAdress%"=="" GOTO ERROR										
IF "%l_login%"=="" GOTO ERROR										
IF "%l_password%"=="" GOTO ERROR										

IF "%TEMP%"=="" set l_temp=C:\TEMP\cs8_backup\%1

REM Check that all files to update are present
set l_file=""
IF NOT EXIST _replaceStr.exe set l_file=_replaceStr.exe
IF NOT EXIST _bootline.bat set l_file=_bootline.bat
IF NOT EXIST _ftpBootline.txt set l_file=_ftpBootline.txt
IF NOT EXIST _ftpDowngrade.txt set l_file=_ftpDowngrade.txt
IF NOT EXIST _bootDown.bat set l_file=_bootDown.bat
IF NOT EXIST flash\sys\vxworks set l_file=flash\sys\vxworks
IF NOT EXIST flash\sys\vxworks.sys set l_file=flash\sys\vxworks.sys
IF NOT %l_file% == "" GOTO INVALID

REM Build directories needed for backup
IF NOT EXIST %l_temp% MD %l_temp%
IF NOT EXIST %l_temp% GOTO ERROR_TEMP
IF NOT EXIST %l_temp%\sys MD %l_temp%\sys				
IF EXIST %l_temp%\options.cfx DEL %l_temp%\options.cfx

:TARGET
REM Build bootline.dat file, using current target bootline.bat
IF EXIST %l_temp%\sys\bootline.dat DEL %l_temp%\sys\bootline.dat
type _ftpDowngrade.txt | _replaceStr #TEMP# %l_temp% | _replaceStr #LOGIN# %l_login% | _replaceStr #PASSWORD# %l_password% > %l_temp%\ftpDowngrade.txt
type _ftpBootline.txt | _replaceStr #TEMP# %l_temp% | _replaceStr #LOGIN# %l_login% | _replaceStr #PASSWORD# %l_password% > %l_temp%\ftpBootline.txt
CALL ftp -i -n -v -s:%l_temp%\ftpBootline.txt %l_ipAdress% > %l_temp%\backup.txt
IF NOT EXIST %l_temp%\sys\bootline.dat GOTO ERROR_BOOTLINE
type %l_temp%\sys\bootline.dat | _replaceStr "ata=1,0(0,0):" "CALL %l_temp%\bootDown.bat %l_temp% %l_ipAdress% " > %l_temp%\bootDown2.bat
type _bootDown.bat > %l_temp%\bootDown.bat
CALL %l_temp%\bootDown2.bat
IF EXIST %l_temp%\sys\bootline.dat GOTO FTP

:FTP
REM Launch update script if bootline.dat file has been built successfully
IF NOT EXIST %l_temp%\sys\bootline.dat GOTO ERROR_BOOTLINE
ECHO.															
ECHO  New files will be send to the target %l_ipAdress%.				
ECHO  Press Ctrl+C to stop process.
PAUSE															
CALL ftp -i -v -n -s:%l_temp%\ftpDowngrade.txt %l_ipAdress%
ECHO.
ECHO --------------------------------------------------
ECHO  Please check that there was no network error during update before reboot.
ECHO  Backup of files has been made on %l_temp%				
ECHO --------------------------------------------------		
ECHO.															
GOTO END														
																
:ERROR															
ECHO.															
ECHO ERROR IN PARAMETER. Format is :	
ECHO downgrade ip_adress login ftpPassword
GOTO END

:ERROR_TEMP
ECHO.
ECHO ERROR Could not create %l_temp% for backup
PAUSE															
GOTO END
			
:INVALID
ECHO.
ECHO ERROR: Incomplete software reference
ECHO %l_file% is missing
ECHO Check the content of flash directory
GOTO END														

:ERROR_BOOTLINE
ECHO.
ECHO ERROR: Bootline.dat missing or invalid on target.
ECHO Only CS8 (teknor) can be downgraded.
GOTO END
																
:END															
																
