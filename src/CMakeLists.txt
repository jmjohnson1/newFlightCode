list(APPEND COMMON_LIBS
	TeensyLib
	SdFat
	LIS3MDL
	Wire
	SPI
	bmi088
	BFSChecksum
	mavlink
	Eigen
	MPU6050
	MPU9250
	PWMServo
	SBUS
	TeensyTimerTool
)


## PID Controllers ##
add_library(pidController
	controller.cpp
	${DRONE_Code_SOURCE_DIR}/include/controller.h
)

target_include_directories(pidController PUBLIC
	"${DRONE_Code_SOURCE_DIR}/include"
)

target_link_libraries(pidController PRIVATE
	${COMMON_LIBS}
)

## Filtering ##
add_library(sensorFusion
	EKF.cpp
	madgwick.cpp
	nav-functions.cpp
)

target_include_directories(sensorFusion PUBLIC
	"${DRONE_Code_SOURCE_DIR}/include"
)

target_link_libraries(sensorFusion PRIVATE
	${COMMON_LIBS}
)
