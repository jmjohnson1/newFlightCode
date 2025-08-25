cmake_minimum_required(VERSION 3.21)
project(QuadcopterSITL VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(GSL REQUIRED)

# Optional visualization
find_package(OpenGL QUIET)
find_package(glfw3 QUIET)
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(MATPLOTLIB_CPP QUIET matplotlib-cpp)
endif()

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../config
    ${CMAKE_CURRENT_SOURCE_DIR}/../lib/filters
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sensorFusion
)

# Hardware Abstraction Layer
set(HAL_SOURCES
    src/hal/teensy_hal.cpp
    # src/hal/timer.cpp
    # src/hal/serial.cpp
)

# Physics Engine
set(PHYSICS_SOURCES
    # src/physics/quadcopter_dynamics.cpp
    src/sensor_models.cpp
    src/dynamics/quadcopter.cpp
    # src/physics/environment.cpp
)

# SITL Core
set(SITL_SOURCES
    src/sitl_main.cpp
    # src/data_logger.cpp
    # src/visualization.cpp
)

# Flight code sources
set(FLIGHT_SOURCES
    ../src/controllers/eulerPID.cpp
    ../src/controllers/dcmPID.cpp
    ../src/sensorFusion/madgwick.cpp
    ../src/sensorFusion/nav-functions.cpp
    ../lib/filters/filter.cpp
)

# Create SITL executable
add_executable(sitl_quadcopter
    ${HAL_SOURCES}
    ${PHYSICS_SOURCES}
    ${SITL_SOURCES}
    ${FLIGHT_SOURCES}
)

# Link libraries
target_link_libraries(sitl_quadcopter
    Eigen3::Eigen
    Threads::Threads
    GSL::gsl
    GSL::gslcblas
    ${CMAKE_DL_LIBS}
)

# Optional visualization
if(OpenGL_FOUND AND glfw3_FOUND)
    target_compile_definitions(sitl_quadcopter PRIVATE ENABLE_3D_VIZ)
    target_link_libraries(sitl_quadcopter OpenGL::GL glfw)
endif()

if(MATPLOTLIB_CPP_FOUND)
    target_compile_definitions(sitl_quadcopter PRIVATE ENABLE_MATPLOTLIB)
    target_link_libraries(sitl_quadcopter ${MATPLOTLIB_CPP_LIBRARIES})
endif()

# Compiler-specific options
target_compile_definitions(sitl_quadcopter PRIVATE SITL_BUILD UNIX_COMPILE)
target_compile_options(sitl_quadcopter PRIVATE -Wall -Wextra -O2)

# Copy config files
configure_file(config/sitl_config.yaml ${CMAKE_BINARY_DIR}/sitl_config.yaml COPYONLY)
