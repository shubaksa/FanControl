@echo off
STM32_Programmer_CLI -c port=SWD reset=HWrst -e all
STM32_Programmer_CLI -c port=SWD reset=HWrst -w .\FanControl\build\VisualGDB\Release\FanControl.bin 0x08000000 -v
STM32_Programmer_CLI -c port=SWD reset=HWrst -w32 0x08007800 0xAABBCC%1 -v
