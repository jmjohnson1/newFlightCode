add_library(MPU9250
	MPU9250.cpp

	MPU9250.h
)

target_include_directories(MPU9250 PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

target_link_libraries(MPU9250 PUBLIC
	Teensy4
	Wire
	SPI
)
