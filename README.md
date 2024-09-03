# Build Instructions
## Prerequisites
- [Arm Embedded Toolchain](https://developer.arm.com/downloads/-/gnu-rm/10-3-2021-10) needs to be installed on the system and present in Path
- CMake

## Building the flight code
From the project root directory, run `cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=cmake/mcu_toolchain.cmake`
Change to the build directory and run `make 
