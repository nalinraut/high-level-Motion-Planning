@ECHO OFF
set l_ipAdress=%1
set l_login=%2
set l_password=%3

REM Set temp directory
IF NOT "%TEMP%"=="" set l_temp=%TEMP%\cs8_backup
IF "%TEMP%"=="" set l_temp=C:\TEMP\cs8_backup
IF NOT EXIST %l_temp%       MD %l_temp%
set l_temp=%l_temp%\%l_ipAdress%

IF "%l_ipAdress%"=="" GOTO ERROR										
IF "%l_login%"=="" GOTO ERROR										
IF "%l_password%"=="" GOTO ERROR										

REM Check that all files to update are present
set l_file=""
IF NOT EXIST _replaceStr.exe set l_file=_replaceStr.exe
IF NOT EXIST flash\sys\mcp\D24275103B.mcp set l_file=flash\sys\mcp\D24275103B.mcp
IF NOT EXIST flash\sys\mcp\D24258902A.mcp set l_file=flash\sys\mcp\D24258902A.mcp
IF NOT %l_file% == "" GOTO INVALID

REM Build directories needed for backup
IF NOT EXIST %l_temp%       MD %l_temp%
IF NOT EXIST %l_temp%	GOTO ERROR_TEMP
IF NOT EXIST %l_temp%\sys MD %l_temp%\sys				
IF EXIST %l_temp%\options.cfx DEL %l_temp%\options.cfx

:TARGET
type _ftpMcp.txt | _replaceStr #TEMP# %l_temp% | _replaceStr #LOGIN# %l_login% | _replaceStr #PASSWORD# %l_password% > %l_temp%\ftpMcp.txt

:FTP
REM Launch update script if bootline.dat file has been built successfully
ECHO.															
ECHO New files will be send to the target %l_ipAdress%.				
ECHO  Press Ctrl+C to stop process.
PAUSE															
CALL ftp -i -v -n -s:%l_temp%\ftpMcp.txt %l_ipAdress%
ECHO.
ECHO --------------------------------------------------
ECHO During reboot, you may be asked to update MCP firmware.
ECHO Press a key to start new firmware installation, and wait as indicated, then reboot.
ECHO --------------------------------------------------		
ECHO.															
GOTO END														
																
:ERROR															
ECHO.															
ECHO ERROR IN PARAMETER. Format is :	
ECHO mcpUpdate ip_adress login ftpPassword
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
																
