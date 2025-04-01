cmake_minimum_required(VERSION 3.15)

project(FanControl LANGUAGES C CXX ASM)

find_bsp(ID com.sysprogs.project_importers.stm32.cubemx SOURCE_PROJECT FanControl.ioc)

add_bsp_based_executable(NAME FanControl
	GENERATE_BIN
	GENERATE_MAP)

