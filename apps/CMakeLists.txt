include(${CMAKE_SOURCE_DIR}/cmake/mcu_upload.cmake)

add_executable(main
	main.cpp
)

target_link_libraries(main PRIVATE
	flightSource
)

flashMcu(main)
