file(GLOB_RECURSE HEADER_LIST "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

add_library(Eigen INTERFACE
	${HEADER_LIST}
)

target_include_directories(Eigen INTERFACE
	${CMAKE_CURRENT_SOURCE_DIR}
)
