@echo off
STM32_Programmer_CLI -c port=SWD ap=0 -e all
STM32_Programmer_CLI -c port=SWD ap=0 -w .\FanControl\build\VisualGDB\Release\FanControl.bin
STM32_Programmer_CLI -c port=SWD ap=0 -v fast
STM32_Programmer_CLI -c port=SWD ap=0 -w32 0x08007800 0xAABBCC%1
