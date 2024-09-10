cmake_minimum_required(VERSION 3.21)
include(FetchContent)
# Includes toolchain, linker scripts, and CLI tools
FetchContent_Declare(
	TeensyTools
	GIT_REPOSITORY https://github.com/jmjohnson1/TeensyTools.git
	GIT_TAG v1.0.1
)
FetchContent_MakeAvailable(TeensyTools)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_TOOLCHAIN_FILE ${teensytools_SOURCE_DIR}/cmake/mcu_toolchain.cmake)
project(
	DRONE_Code
	VERSION 1.1
	LANGUAGES C CXX
)
# Sets compiler and linker options as well as some definitions
include("${teensytools_SOURCE_DIR}/cmake/mcu_config.cmake")
configMCU(${teensytools_SOURCE_DIR})

FetchContent_Declare(
	Teensy4
	GIT_REPOSITORY https://github.com/jmjohnson1/Teensy4.git
	GIT_TAG v1.0.0
)
FetchContent_MakeAvailable(Teensy4)

# FetchContent_Declare(
# 	TeensyLog
# 	GIT_REPOSITORY https://github.com/jmjohnson1/TeensyLog.git
# 	GIT_TAG v1.0.0
# )
# FetchContent_MakeAvailable(TeensyLog)

# Libraries
add_subdirectory(lib)

# Flight code source files
add_subdirectory(src)

# main program
include(${CMAKE_SOURCE_DIR}/cmake/mcu_upload.cmake)

add_executable(flightCode
	apps/flightCode.cpp
	src/datalogger.cpp
	src/EKF.cpp
	src/IMU.cpp
	src/madgwick.cpp
	src/motors.cpp
	src/nav-functions.cpp
	src/navHandler.cpp
	src/radio.cpp
	src/serialDebug.cpp
	src/statFun.cpp
	src/telemetry.cpp
	src/testing.cpp
)

target_include_directories(flightCode PUBLIC
	${CMAKE_SOURCE_DIR}/include	
	${CMAKE_SOURCE_DIR}/config
)

target_link_libraries(flightCode
	Teensy4
	SdFat
	LIS3MDL
	Wire
	SPI
	bmi088
	BFSChecksum
	mavlink
	# EEPROM
	Eigen
	MPU6050
	MPU9250
	PWMServo
	SBUS
	TeensyTimerTool
	libFilter
	PIDController
)

flashMcu(flightCode ${CMAKE_SOURCE_DIR})

add_executable(radioTest
	test/radioTest.cpp
	src/radio.cpp
	src/datalogger.cpp
)

target_include_directories(radioTest PUBLIC
	${CMAKE_SOURCE_DIR}/include	
)

target_link_libraries(radioTest
	Teensy4
	SdFat
	LIS3MDL
	Wire
	SPI
	bmi088
	BFSChecksum
	mavlink
	# EEPROM
	Eigen
	MPU6050
	MPU9250
	PWMServo
	SBUS
	TeensyTimerTool
)

flashMcu(radioTest ${CMAKE_SOURCE_DIR})
