add_library(TFMPlus
	TFMPlus.cpp

	# Headers (for IDE)
	TFMPlus.h
)

target_include_directories(TFMPlus PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

target_link_libraries(TFMPlus PUBLIC Teensy4)
