file(GLOB HEADER_LIST CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

add_library(MPU6050
	I2Cdev.cpp
	MPU6050.cpp

	${HEADER_LIST}
)

target_include_directories(MPU6050 PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

target_link_libraries(MPU6050 PRIVATE
	Wire
	TeensyLib
)
