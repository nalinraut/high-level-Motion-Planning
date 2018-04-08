set l_temp=%1
set l_ipAdress=%2
set l_bootFile=%3
set l_target=""
REM Bootline is refreshed only for teknor targets
DEL %l_temp%\sys\bootline.dat
IF NOT "%l_bootFile%"=="/ata0/teknor" GOTO END
ECHO ata=1,0(0,0):/ata0/vxworks e=%l_ipAdress% f=0xa o=fei > %l_temp%\sys\bootline.dat
:END
