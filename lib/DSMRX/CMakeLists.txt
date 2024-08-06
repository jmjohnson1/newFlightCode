add_library(DSMRX
	DSMRX.cpp

	DSMRX.h
)

target_include_directories(DSMRX PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
