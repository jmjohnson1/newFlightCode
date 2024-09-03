# Optionally glob, but only for CMake 3.12 or later:
file(GLOB HEADER_LIST CONFIGURE_DEPENDS "${DRONE_Code_SOURCE_DIR}/include/*.h")

# Make an automatic library - will be static or dynamic based on user setting
add_library(flightLib 
	controller.cpp
	datalogger.cpp
	EKF.cpp
	filter.cpp
	IMU.cpp
	madgwick.cpp
	motors.cpp
	nav-functions.cpp
	navHandler.cpp
	radio.cpp
	serialDebug.cpp
	statFun.cpp
	telemetry.cpp
	testing.cpp

	${HEADER_LIST}
)

# We need this directory, and users of our library will need it too
target_include_directories(flightLib PUBLIC 
	"${DRONE_Code_SOURCE_DIR}/include"
)

target_link_libraries(flightLib PUBLIC
	TeensyLib
	SdFat
	LIS3MDL
	Wire
	SPI
	bmi088
	BFSChecksum
	mavlink
	# EEPROM
	Eigen
	MPU6050
	MPU9250
	PWMServo
	SBUS
	TeensyTimerTool
)

