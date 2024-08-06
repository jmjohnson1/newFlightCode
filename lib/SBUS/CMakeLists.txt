add_library(SBUS
	SBUS.cpp

	# Headers (for IDE)
	SBUS.h
	elapsedMillis.h
)

target_include_directories(SBUS PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

target_link_libraries(SBUS PRIVATE TeensyLib)
