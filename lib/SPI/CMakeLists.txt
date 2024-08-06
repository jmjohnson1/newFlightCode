add_library(SPI
	SPI.cpp
	# Headers (for IDE)
	SPI.h
)

target_include_directories(SPI PUBLIC 
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

target_link_libraries(SPI PRIVATE TeensyLib)
