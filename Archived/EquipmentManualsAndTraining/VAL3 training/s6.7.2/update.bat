@ECHO OFF
set l_ipAdress=%1
set l_login=%2
set l_password=%3
set l_target=%4
set l_keep=false

REM Management of optional target parameter
if "%l_target%"=="teknor" GOTO ORDER
if "%l_target%"=="men" GOTO ORDER
if "%l_target%"=="men_hp" GOTO ORDER
if "%l_target%"=="men007" GOTO ORDER
if "%l_target%"=="men07n" GOTO ORDER
REM if target = keep, bootline.dat will be kept unchanged
if "%l_target%"=="keep" set l_keep=true
if "%l_target%"=="keep" set l_target=
if NOT "%l_target%"=="" GOTO ERROR

REM Set temp directory
:ORDER
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
IF NOT EXIST _bootline.bat set l_file=_bootline.bat
IF NOT EXIST _ftpBootline.txt set l_file=_ftpBootline.txt
IF NOT EXIST _ftp.txt set l_file=_ftp.txt
IF NOT EXIST _ftpCS8C.txt set l_file=_ftpCS8C.txt
IF NOT EXIST flash\sys\men007 set l_file=flash\sys\men007
IF NOT EXIST flash\sys\men07n set l_file=flash\sys\men07n
IF NOT EXIST flash\sys\men_hp set l_file=flash\sys\men_hp
IF NOT EXIST flash\sys\men set l_file=flash\sys\men
IF NOT EXIST flash\sys\teknor set l_file=flash\sys\teknor
IF NOT EXIST flash\sys\cs8.out set l_file=flash\sys\cs8.out
IF NOT EXIST flash\sys\men007.sys set l_file=flash\sys\men007.sys
IF NOT EXIST flash\sys\men07n.sys set l_file=flash\sys\men07n.sys
IF NOT EXIST flash\sys\men_hp.sys set l_file=flash\sys\men_hp.sys
IF NOT EXIST flash\sys\men.sys set l_file=flash\sys\men.sys
IF NOT EXIST flash\sys\teknor.sys set l_file=flash\sys\teknor.sys
IF NOT EXIST flash\sys\app\1_ep.o set l_file=flash\sys\app\1_ep.o
IF NOT EXIST flash\sys\app\2_sca.o set l_file=flash\sys\app\2_sca.o
IF NOT EXIST flash\sys\app\3_plc.o set l_file=flash\sys\app\3_plc.o
IF NOT EXIST flash\sys\app\4_cds.o set l_file=flash\sys\app\4_cds.o
IF NOT EXIST flash\sys\app\5_opt.o set l_file=flash\sys\app\5_opt.o
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
IF NOT EXIST flash\sys\starc\lib\sta_config.o set l_file=flash\sys\starc\lib\sta_config.o
IF NOT EXIST flash\sys\starc\starc2\boottable.hex set l_file=flash\sys\starc\starc2\boottable.hex
IF NOT EXIST flash\sys\starc\starc2\program.hex set l_file=flash\sys\starc\starc2\program.hex
IF NOT EXIST flash\sys\starc\starc2\fpga_fw.hex set l_file=flash\sys\starc\starc2\fpga_fw.hex
IF NOT EXIST flash\sys\cds\hildpm3s.out set l_file=flash\sys\cds\hildpm3s.out
IF NOT EXIST flash\sys\cds\plcvxw.ini set l_file=flash\sys\cds\plcvxw.ini
IF NOT EXIST flash\sys\cds\plcvxw.o set l_file=flash\sys\cds\plcvxw.o
IF NOT EXIST flash\sys\cds\stblconf.out set l_file=flash\sys\cds\stblconf.out
IF NOT EXIST flash\sys\can\driver.o set l_file=flash\sys\can\driver.o
IF NOT EXIST flash\sys\can\canio.o set l_file=flash\sys\can\canio.o
IF NOT EXIST flash\sys\configs\system.cf set l_file=flash\sys\configs\system.cf
IF NOT EXIST flash\sys\configs\fontsize.cfx set l_file=flash\sys\configs\system.cfx
IF NOT EXIST flash\sys\configs\resources\english.cfx set l_file=flash\sys\configs\resources\english.cfx
IF NOT EXIST flash\sys\configs\resources\deutsch.cfx set l_file=flash\sys\configs\resources\deutsch.cfx
IF NOT EXIST flash\sys\configs\resources\francais.cfx set l_file=flash\sys\configs\resources\francais.cfx
IF NOT EXIST flash\sys\configs\resources\italiano.cfx set l_file=flash\sys\configs\resources\italiano.cfx
IF NOT EXIST flash\sys\configs\resources\espanol.cfx set l_file=flash\sys\configs\resources\espanol.cfx
IF NOT EXIST flash\sys\configs\resources\chinese.cfx set l_file=flash\sys\configs\resources\chinese.cfx
IF NOT EXIST flash\sys\configs\resources\japanese.cfx set l_file=flash\sys\configs\resources\japanese.cfx
IF NOT EXIST flash\sys\templates\versions.cfx set l_file=flash\sys\templates\versions.cfx
IF NOT EXIST flash\sys\templates\profile.cfx set l_file=flash\sys\templates\profile.cfx
IF NOT EXIST flash\usr\configs\templates\iomap\iomap.cf set l_file=flash\usr\configs\templates\iomap\iomap.cf
IF NOT EXIST flash\usr\recorder\records.cfx set l_file=flash\usr\recorder\records.cfx
IF NOT EXIST flash\usr\configs\profiles\default.cfx set l_file=flash\usr\configs\profiles\default.cfx
IF NOT EXIST flash\usr\configs\profiles\maintenance.cfx set l_file=flash\usr\configs\profiles\maintenance.cfx
IF NOT EXIST flash\usr\templates\default\default.pjx set l_file=flash\usr\templates\default\default.pjx
IF NOT EXIST flash\usr\templates\default\default.dtx set l_file=flash\usr\templates\default\default.dtx
IF NOT EXIST flash\usr\templates\default\start.pgx set l_file=flash\usr\templates\default\start.pgx
IF NOT EXIST flash\usr\templates\default\stop.pgx set l_file=flash\usr\templates\default\stop.pgx
IF NOT %l_file% == "" GOTO INVALID

REM Build directories needed for backup
IF NOT EXIST %l_temp%       MD %l_temp%
IF NOT EXIST %l_temp%	GOTO ERROR_TEMP
IF NOT EXIST %l_temp%\sys MD %l_temp%\sys				
IF EXIST %l_temp%\options.cfx DEL %l_temp%\options.cfx

:TARGET
REM Compute target using bootline.dat if target is still unknown
IF NOT "%l_target%"=="" GOTO BOOTLINE
REM Retrieve current bootline.dat
IF EXIST %l_temp%\sys\bootline.dat DEL %l_temp%\sys\bootline.dat
type _ftpBootline.txt | _replaceStr #TEMP# %l_temp% | _replaceStr #LOGIN# %l_login% | _replaceStr #PASSWORD# %l_password% > %l_temp%\ftpBootline.txt
CALL ftp -i -n -v -s:%l_temp%\ftpBootline.txt %l_ipAdress% > %l_temp%\backup.txt
IF NOT EXIST %l_temp%\sys\bootline.dat GOTO ERROR_BOOTLINE
REM Download current bootline to determine target if target is still unknown
type %l_temp%\sys\bootline.dat | _replaceStr "ata=1,0(0,0):/ata0/" "CALL %l_temp%\bootline.bat %l_temp% " > %l_temp%\bootline2.bat
type _bootline.bat > %l_temp%\bootline.bat
CALL %l_temp%\bootline2.bat %l_temp%

:BOOTLINE
REM Create new bootline.dat unless 'keep' was requested
if "%l_keep%"=="true" GOTO FTP
REM Create the bootline.dat that matches the specified target
IF EXIST %l_temp%\sys\bootline.dat DEL %l_temp%\sys\bootline.dat
IF "%l_target%"=="men" ECHO ata=1,0(0,0):/ata0/men f=0xa o=gei > %l_temp%\sys\bootline.dat
IF "%l_target%"=="men_hp" ECHO ata=1,0(0,0):/ata0/men_hp f=0xa o=gei > %l_temp%\sys\bootline.dat
IF "%l_target%"=="men007" ECHO ata=1,0(0,0):/ata0/men007 f=0xa o=fei > %l_temp%\sys\bootline.dat
IF "%l_target%"=="men07n" ECHO ata=1,0(0,0):/ata0/men07n f=0xa o=fei > %l_temp%\sys\bootline.dat
IF "%l_target%"=="teknor" ECHO ata=1,0(0,0):/ata0/teknor f=0xa o=fei > %l_temp%\sys\bootline.dat

:FTP
REM Launch update script if bootline.dat file has been built successfully
IF NOT EXIST %l_temp%\sys\bootline.dat GOTO ERROR_BOOTLINE

REM Update ftp script with target name
IF "%l_target%"=="" GOTO ERROR_BOOTLINE
IF "%l_target%"=="teknor" type _ftp.txt | _replaceStr #TEMP# %l_temp% | _replaceStr #LOGIN# %l_login% | _replaceStr #PASSWORD# %l_password% | _replaceStr #TARGET# %l_target% > %l_temp%\ftp.txt
IF NOT "%l_target%"=="teknor" type _ftpCS8C.txt | _replaceStr #TEMP# %l_temp% | _replaceStr #LOGIN# %l_login% | _replaceStr #PASSWORD# %l_password% | _replaceStr #TARGET# %l_target% > %l_temp%\ftp.txt

ECHO.
ECHO  Use backup.bat command to first backup controller version.
ECHO  New files will be send to the target %l_ipAdress%.
ECHO  Press Ctrl+C to stop process.
PAUSE
CALL ftp -i -v -n -s:%l_temp%\ftp.txt %l_ipAdress%
ECHO.
ECHO --------------------------------------------------
ECHO  Please check that there was no network error during update before reboot.
ECHO --------------------------------------------------		
ECHO.															
GOTO END														
																
:ERROR															
ECHO.															
ECHO ERROR IN PARAMETER. Format is :	
ECHO update ip_adress login ftpPassword [target]
ECHO target= teknor (CS8) / men (CS8C, CS8HP) / men007 (CS8, CS8C, CS8HP) / men07n (CS8, CS8C, CS8HP)
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
ECHO ERROR: Unable to connect or retrieve target type.
ECHO ERROR: (network error, invalid login or password,
ECHO ERROR:  or /sys/bootline.dat missing or unexpected)
ECHO ERROR: Check network and login.
ECHO ERROR: If correct, specify a CPU type as 4th parameter.
GOTO END
																
:END															
																
