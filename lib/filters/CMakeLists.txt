add_library(libFilter
	filter.cpp
	filter.h
)

target_include_directories(libFilter PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
