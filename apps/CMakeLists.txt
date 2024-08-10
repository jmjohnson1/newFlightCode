include(${CMAKE_SOURCE_DIR}/cmake/mcu_upload.cmake)

add_executable(main
	main.cpp
)

target_include_directories(main PUBLIC
	${CMAKE_SOURCE_DIR}/include	
)

target_link_libraries(main
	TeensyLib
	SdFat
	LIS3MDL
	Wire
	SPI
	bmi088
	BFSChecksum
	mavlink
	EEPROM
	Eigen
	MPU6050
	MPU9250
	PWMServo
	SBUS
	TeensyTimerTool
	flightSource
)

flashMcu(main)
