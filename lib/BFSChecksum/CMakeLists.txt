add_library(BFSChecksum
	checksum.cpp

	checksum.h
)

target_include_directories(BFSChecksum PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

target_link_libraries(BFSChecksum PRIVATE
	TeensyLib
)
