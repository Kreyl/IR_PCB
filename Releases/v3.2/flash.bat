:start

"C:\ST\CubeProg\bin\STM32_Programmer_CLI.exe" -c port=SWD -w IR_PCB_Full_202408312328.hex 0x08000000 -v  -rst

@if %ERRORLEVEL% EQU 0 echo Done

@REM pause
@REM @ping 10.25.11.254 -n 1 -w 2000 > nul

@REM goto start