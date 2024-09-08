add_library(bmi088
	BMI088.cpp
	BMI088.h
)

target_include_directories(bmi088 PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

target_link_libraries(bmi088 PRIVATE
	Wire
	SPI
	TeensyLib
)
