@ECHO OFF
set l_ipAdress=%1
set l_login=%2
set l_password=%3
set l_order=%4
set l_target=""

REM Set temp directory
IF NOT "%TEMP%"=="" set l_temp=%TEMP%\cs8_backup
IF "%TEMP%"=="" set l_temp=C:\TEMP\cs8_backup
IF NOT EXIST %l_temp%       MD %l_temp%
IF NOT "%l_order%"=="" set l_temp=%l_temp%\%l_order%
IF "%l_order%"=="" set l_temp=%l_temp%\%l_ipAdress%

IF "%l_ipAdress%"=="" GOTO ERROR										
IF "%l_login%"=="" GOTO ERROR										
IF "%l_password%"=="" GOTO ERROR										

REM Check that all files to update are present
set l_file=""
IF NOT EXIST _replaceStr.exe set l_file=_replaceStr.exe
IF NOT EXIST _ftpBackup.txt set l_file=_ftpBackup.txt
IF NOT %l_file% == "" GOTO INVALID

REM Build directories needed for backup
IF NOT EXIST %l_temp%       MD %l_temp%
IF NOT EXIST %l_temp%	GOTO ERROR_TEMP
IF NOT EXIST %l_temp%\sys MD %l_temp%\sys				
IF NOT EXIST %l_temp%\usr MD %l_temp%\usr				
IF NOT EXIST %l_temp%\log MD %l_temp%\log				
IF NOT EXIST %l_temp%\sys\hw_v1 MD %l_temp%\sys\hw_v1 				
IF NOT EXIST %l_temp%\sys\hw_v2 MD %l_temp%\sys\hw_v2
IF NOT EXIST %l_temp%\sys\app MD %l_temp%\sys\app 				
IF NOT EXIST %l_temp%\sys\mcp MD %l_temp%\sys\mcp
IF NOT EXIST %l_temp%\sys\configs MD %l_temp%\sys\configs
IF NOT EXIST %l_temp%\sys\templates MD %l_temp%\sys\templates
IF NOT EXIST %l_temp%\sys\Starc MD %l_temp%\sys\Starc
IF NOT EXIST %l_temp%\sys\Starc\drive MD %l_temp%\sys\Starc\drive
IF NOT EXIST %l_temp%\sys\Starc\dsi MD %l_temp%\sys\Starc\dsi
IF NOT EXIST %l_temp%\sys\Starc\dsp1 MD %l_temp%\sys\Starc\dsp1
IF NOT EXIST %l_temp%\sys\Starc\dsp2 MD %l_temp%\sys\Starc\dsp2
IF NOT EXIST %l_temp%\sys\Starc\fpga MD %l_temp%\sys\Starc\fpga
IF NOT EXIST %l_temp%\sys\Starc\lib MD %l_temp%\sys\Starc\lib
IF NOT EXIST %l_temp%\usr\configs MD %l_temp%\usr\configs
IF NOT EXIST %l_temp%\usr\configs\profiles MD %l_temp%\usr\configs\profiles
IF NOT EXIST %l_temp%\usr\usrapp MD %l_temp%\usr\usrapp
IF NOT EXIST %l_temp%\usr\templates MD %l_temp%\usr\templates
IF NOT EXIST %l_temp%\usr\applicom MD %l_temp%\usr\applicom
IF NOT EXIST %l_temp%\usr\applicom\io MD %l_temp%\usr\applicom\io
IF NOT EXIST %l_temp%\usr\applicom\modbus MD %l_temp%\usr\applicom\modbus
IF NOT EXIST %l_temp%\usr\temp MD %l_temp%\usr\temp

:FTP
type _ftpBackup.txt | _replaceStr #TEMP# %l_temp% | _replaceStr #LOGIN# %l_login% | _replaceStr #PASSWORD# %l_password% > %l_temp%\ftpBackup.txt
REM Launch update script if bootline.dat file has been built successfully
ECHO.															
ECHO  Press Enter to start backup.
ECHO  Press Ctrl+C to stop process.
PAUSE															
CALL ftp -i -v -n -s:%l_temp%\ftpBackup.txt %l_ipAdress%
ECHO.
ECHO --------------------------------------------------
ECHO  Backup of files has been made on %l_temp%				
ECHO --------------------------------------------------		
ECHO.															
GOTO END														
																
:ERROR															
ECHO.															
ECHO ERROR IN PARAMETER. Format is :	
ECHO backup ip_adress login ftpPassword [Serial-number]		
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
														
:END															
																
