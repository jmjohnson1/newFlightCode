file(GLOB_RECURSE HEADER_LIST CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

add_library(mavlink
	heartbeat.cpp
	mission.cpp
	rtcm.cpp
	telemetry.cpp

	${HEADER_LIST}
)

target_include_directories(mavlink PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

target_link_libraries(mavlink PRIVATE
	Teensy4
	BFSUnits
)

target_link_libraries(mavlink PUBLIC
	BFSUnits
)
