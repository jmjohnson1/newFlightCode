file(GLOB_RECURSE HEADER_LIST CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

add_library(TeensyTimerTool
	config.cpp
	
	# API
	API/baseTimer.cpp
	API/Timer.cpp

	# ErrorHandling
	ErrorHandling/error_handler.cpp

	# TimerModules
	TimerModules/GPT/GPTChannel.cpp
	TimerModules/PIT4/PIT.cpp
	TimerModules/RTC/RTC.cpp
	TimerModules/TCK/TCK.cpp
	TimerModules/TCK/TckChannelBase.cpp
	TimerModules/TCK/tickCounters.cpp

	# Headers (for IDE)
	${HEADER_LIST}
)

target_include_directories(TeensyTimerTool PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

target_link_libraries(TeensyTimerTool PUBLIC Teensy4)
