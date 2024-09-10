add_library(PWMServo
	PWMServo.cpp
	
	PWMServo.h
)

target_include_directories(PWMServo PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

target_link_libraries(PWMServo PUBLIC Teensy4)
