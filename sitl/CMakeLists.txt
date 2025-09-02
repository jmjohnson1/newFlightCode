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
    ${CMAKE_CURRENT_SOURCE_DIR}/../lib/BFSChecksum
    ${CMAKE_CURRENT_SOURCE_DIR}/../lib/BFSMavlink
    ${CMAKE_CURRENT_SOURCE_DIR}/../lib/BFSUnits
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/sensorFusion
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/controllers
)

# Hardware Abstraction Layer
set(HAL_SOURCES
    src/hal/teensy_hal.cpp
    src/hal/NetworkSerial.cpp
)

# Physics Engine
set(PHYSICS_SOURCES
    src/sensor_models.cpp
    src/dynamics/quadcopter.cpp
)

# SITL Core
set(SITL_SOURCES
    src/sitl_main.cpp
)

# Flight code sources
set(FLIGHT_SOURCES
    ../src/controllers/eulerPID.cpp
    ../src/controllers/dcmPID.cpp
    ../src/sensorFusion/madgwick.cpp
    ../src/sensorFusion/nav-functions.cpp
    ../src/sensorFusion/EKF.cpp

    ../lib/filters/filter.cpp
    ../lib/BFSChecksum/checksum.cpp

    ../lib/BFSMavlink/heartbeat.cpp
    ../lib/BFSMavlink/mission.cpp
    ../lib/BFSMavlink/rtcm.cpp
    ../lib/BFSMavlink/telemetry.cpp
    
    ../lib/BFSUnits/convacc.cpp
    ../lib/BFSUnits/convang.cpp
    ../lib/BFSUnits/convangacc.cpp
    ../lib/BFSUnits/convangvel.cpp
    ../lib/BFSUnits/convdensity.cpp
    ../lib/BFSUnits/convforce.cpp
    ../lib/BFSUnits/convlength.cpp
    ../lib/BFSUnits/convmass.cpp
    ../lib/BFSUnits/convpres.cpp
    ../lib/BFSUnits/convtemp.cpp
    ../lib/BFSUnits/convvel.cpp

    ../src/navHandler.cpp
    ../src/radio.cpp
    ../src/statFun.cpp
    ../src/telemetry.cpp
    ../src/IMU.cpp
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
