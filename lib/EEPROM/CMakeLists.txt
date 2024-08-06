add_library(EEPROM
	EEPROM.cpp
	
	EEPROM.h
)

target_include_directories(EEPROM PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

target_link_libraries(EEPROM PRIVATE
	TeensyLib
)
