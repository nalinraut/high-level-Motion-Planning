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
IF NOT EXIST flash\sys\starc\versions.cfx set l_file=flash\sys\starc\versions.cfx
IF NOT EXIST flash\sys\starc\drive\dsp_firm.emb set l_file=flash\sys\starc\drive\dsp_firm.emb
IF NOT EXIST flash\sys\starc\drive\dsp_firm_38.emb set l_file=flash\sys\starc\drive\dsp_firm_38.emb
IF NOT EXIST flash\sys\starc\drive\dsp_firm_40.emb set l_file=flash\sys\starc\drive\dsp_firm_40.emb
IF NOT EXIST flash\sys\starc\drive\dsp_firm_144.emb set l_file=flash\sys\starc\drive\dsp_firm_144.emb
IF NOT EXIST flash\sys\starc\drive\dsp_firm_146.emb set l_file=flash\sys\starc\drive\dsp_firm_146.emb
IF NOT EXIST flash\sys\starc\drive\dsp_firm_151.emb set l_file=flash\sys\starc\drive\dsp_firm_151.emb
IF NOT EXIST flash\sys\starc\drive\dsp_firm_238.emb set l_file=flash\sys\starc\drive\dsp_firm_238.emb
IF NOT EXIST flash\sys\starc\drive\dsp_firm_240.emb set l_file=flash\sys\starc\drive\dsp_firm_240.emb
IF NOT EXIST flash\sys\starc\drive\dsp_fir2.emb set l_file=flash\sys\starc\drive\dsp_fir2.emb
IF NOT EXIST flash\sys\starc\drive\dsp_fir2_38.emb set l_file=flash\sys\starc\drive\dsp_fir2_38.emb
IF NOT EXIST flash\sys\starc\drive\dsp_fir2_40.emb set l_file=flash\sys\starc\drive\dsp_fir2_40.emb
IF NOT EXIST flash\sys\starc\drive\dsp_fir2_144.emb set l_file=flash\sys\starc\drive\dsp_fir2_144.emb
IF NOT EXIST flash\sys\starc\drive\dsp_fir2_146.emb set l_file=flash\sys\starc\drive\dsp_fir2_146.emb
IF NOT EXIST flash\sys\starc\drive\dsp_fir2_151.emb set l_file=flash\sys\starc\drive\dsp_fir2_151.emb
IF NOT EXIST flash\sys\starc\drive\dsp_fir2_238.emb set l_file=flash\sys\starc\drive\dsp_fir2_238.emb
IF NOT EXIST flash\sys\starc\drive\dsp_fir2_240.emb set l_file=flash\sys\starc\drive\dsp_fir2_240.emb
IF NOT EXIST flash\sys\starc\drive\pld1708v0.emb set l_file=flash\sys\starc\drive\pld1708v0.emb
IF NOT EXIST flash\sys\starc\drive\pld1708v0_7.emb set l_file=flash\sys\starc\drive\pld1708v0_7.emb
IF NOT EXIST flash\sys\starc\drive\pld1708v0_10.emb set l_file=flash\sys\starc\drive\pld1708v0_10.emb
IF NOT EXIST flash\sys\starc\drive\pld1708v0_2.emb set l_file=flash\sys\starc\drive\pld1708v0_2.emb
IF NOT EXIST flash\sys\starc\drive\pld1708v0_1.emb set l_file=flash\sys\starc\drive\pld1708v0_1.emb
IF NOT EXIST flash\sys\starc\dsi\dsp_fw.hex set l_file=flash\sys\starc\dsi\dsp_fw.hex
IF NOT EXIST flash\sys\starc\dsi\fpga_fw.hex set l_file=flash\sys\starc\dsi\fpga_fw.hex
IF NOT EXIST flash\sys\starc\dsp1\boottable.hex set l_file=flash\sys\starc\dsp1\boottable.hex
IF NOT EXIST flash\sys\starc\dsp1\program.hex set l_file=flash\sys\starc\dsp1\program.hex
IF NOT EXIST flash\sys\starc\dsp2\boottable.hex set l_file=flash\sys\starc\dsp2\boottable.hex
IF NOT EXIST flash\sys\starc\dsp2\program.hex set l_file=flash\sys\starc\dsp2\program.hex
IF NOT EXIST flash\sys\starc\dsp2\memtest.hex set l_file=flash\sys\starc\dsp2\memtest.hex
IF NOT EXIST flash\sys\starc\fpga\fpga_fw.hex set l_file=flash\sys\starc\fpga\fpga_fw.hex
IF NOT EXIST flash\sys\starc\starc2\boottable.hex set l_file=flash\sys\starc\starc2\boottable.hex
IF NOT EXIST flash\sys\starc\starc2\program.hex set l_file=flash\sys\starc\starc2\program.hex
IF NOT EXIST flash\sys\starc\starc2\fpga_fw.hex set l_file=flash\sys\starc\starc2\fpga_fw.hex
IF NOT %l_file% == "" GOTO INVALID

REM Build directories needed for backup
IF NOT EXIST %l_temp%       MD %l_temp%
IF NOT EXIST %l_temp%	GOTO ERROR_TEMP
IF NOT EXIST %l_temp%\sys MD %l_temp%\sys				
IF EXIST %l_temp%\options.cfx DEL %l_temp%\options.cfx

:TARGET
type _starcFtp.txt | _replaceStr #TEMP# %l_temp% | _replaceStr #LOGIN# %l_login% | _replaceStr #PASSWORD# %l_password% > %l_temp%\starcFtp.txt

:FTP
REM Launch update script if bootline.dat file has been built successfully
ECHO.															
ECHO  Use backup.bat command to first backup controller version.
ECHO  New files will be send to the target %l_ipAdress%.				
ECHO IMPORTANT - After installation, an adjustment of arm position will be required.
ECHO IMPORTANT - Make sure the arm is stopped at a known position.
ECHO  Press Ctrl+C to stop process.
PAUSE															
CALL ftp -i -v -n -s:%l_temp%\starcFtp.txt %l_ipAdress%
IF NOT "%l_robot%"==""	CALL ftp -i -v -n -s:%l_temp%\ftpOptions.txt %l_ipAdress%
ECHO.
ECHO --------------------------------------------------
ECHO  Please check that there was no network error during update before reboot.
ECHO --------------------------------------------------		
ECHO.															
GOTO END														
																
:ERROR															
ECHO.															
ECHO ERROR IN PARAMETER. Format is :	
ECHO starcUpdate ip_adress login ftpPassword
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
																
