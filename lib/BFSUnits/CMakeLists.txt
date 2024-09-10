file(GLOB HEADER_LIST CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

add_library(BFSUnits
	convacc.cpp
	convang.cpp
	convangacc.cpp
	convangvel.cpp
	convdensity.cpp
	convforce.cpp
	convlength.cpp
	convmass.cpp
	convpres.cpp
	convtemp.cpp
	convvel.cpp

	${HEADER_LIST}
)

target_include_directories(BFSUnits PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

target_link_libraries(BFSUnits PUBLIC
	Teensy4
)
