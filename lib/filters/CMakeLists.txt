add_library(Filter
	filter.cpp
	filter.h
)

target_include_directories(Filter PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
