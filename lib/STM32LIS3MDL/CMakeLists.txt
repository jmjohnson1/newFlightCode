file(GLOB HEADER_LIST CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

add_library(LIS3MDL
	LIS3MDL_MAG_Driver.c
	LIS3MDLSensor.cpp
	
	# Headers (for IDE)
	${HEADER_LIST}
)

target_include_directories(LIS3MDL PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>	
)

target_link_libraries(LIS3MDL PRIVATE 
	TeensyLib
	Wire
	SPI
)
