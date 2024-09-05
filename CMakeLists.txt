cmake_minimum_required(VERSION 3.14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(
	DRONE_Code
	VERSION 1.0
	LANGUAGES C CXX
)

set(MCU IMXRT1062)
set(MCU_LD imxrt1062.ld)
set(MCU_DEF ARDUINO_TEENSY40)
set(TEENSYDUINO_VERSION 159)
set(ARDUINO_VERSION 10813)

# MCU configuration
add_definitions(
	-D__${MCU}__
	-DARDUINO=${ARDUINO_VERSION}
	-DTEENSYDUINO=${TEENSYDUINO_VERSION}
	-D${MCU_DEF}
	-DF_CPU=600000000
	-DUSB_SERIAL
	-DLAYOUT_US_ENGLISH
	-DUSING_MAKEFILE
)
# Compile options
add_compile_options(
	$<$<COMPILE_LANGUAGE:C>:-std=gnu11>
	$<$<COMPILE_LANGUAGE:CXX>:-std=gnu++20>
	$<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
	$<$<COMPILE_LANGUAGE:CXX>:-felide-constructors>
	$<$<COMPILE_LANGUAGE:CXX>:-Wno-error=narrowing>
	$<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
	$<$<COMPILE_LANGUAGE:CXX>:-Wno-volatile>
	$<$<COMPILE_LANGUAGE:CXX>:-fpermissive>
	-g 
	-Os 
	-Wno-psabi 
	-mthumb 
	-ffunction-sections 
	-fdata-sections 
	-MMD
	-mcpu=cortex-m7
	-mfloat-abi=hard 
	-mfpu=fpv5-d16
)
# Linking
set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/teensy4/imxrt1062.ld")
add_link_options(
	-Os
	-Wl,--gc-sections,--relax
	--specs=nano.specs
	-mcpu=cortex-m7
	-mfloat-abi=hard
	-mfpu=fpv5-d16
	-mthumb
	-T${LINKER_SCRIPT}
)
link_libraries(
	-lm
	-lstdc++
)

# Teensy files
add_subdirectory(teensy4)

# Libraries
add_subdirectory(lib)

# Flight code source files
add_subdirectory(src)

# main program
include(${CMAKE_SOURCE_DIR}/cmake/mcu_upload.cmake)

add_executable(flightCode
	apps/flightCode.cpp
	src/controller.cpp
	src/datalogger.cpp
	src/EKF.cpp
	src/filter.cpp
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
	TeensyLib
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

flashMcu(flightCode)

add_executable(radioTest
	test/radioTest.cpp
	src/radio.cpp
	src/datalogger.cpp
)

target_include_directories(radioTest PUBLIC
	${CMAKE_SOURCE_DIR}/include	
)

target_link_libraries(radioTest
	TeensyLib
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

flashMcu(radioTest)
