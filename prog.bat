@echo off
REM First parameter = Target RPM (18k RPM = 0x4650, 13k RPM = 0x32C8)
REM Second parameter = Fan RPM (4800 RPM = 0x12C0)
REM Third parameter = Initial delay in ms (5000ms = 0x1388)
STM32_Programmer_CLI -c port=SWD reset=HWrst -e all
STM32_Programmer_CLI -c port=SWD reset=HWrst -w .\FanControl\build\VisualGDB\Release\FanControl.bin 0x08000000 -v
STM32_Programmer_CLI -c port=SWD reset=HWrst -w32 0x08007800 0xAABBCCDD %1 %2 %3 -v
