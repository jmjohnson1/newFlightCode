file(GLOB HEADER_LIST CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

add_library(Wire
	Wire.cpp
	WireIMXRT.cpp
	WireKinetis.cpp
	
	# Headers (for IDE)
	${HEADER_LIST}
)

target_include_directories(Wire PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>	
)

target_link_libraries(Wire PRIVATE TeensyLib)
