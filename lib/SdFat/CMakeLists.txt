# These are useful for IDE
file(GLOB HEADER_LIST CONFIGURE_DEPENDS "${DRONE_Code_SOURCE_DIR}/lib/SdFat/*.h")
file(GLOB HEADER_LIST CONFIGURE_DEPENDS "${DRONE_Code_SOURCE_DIR}/lib/SdFat/common/*.h")
file(GLOB HEADER_LIST CONFIGURE_DEPENDS "${DRONE_Code_SOURCE_DIR}/lib/SdFat/DigitalIO/*.h")
file(GLOB HEADER_LIST CONFIGURE_DEPENDS "${DRONE_Code_SOURCE_DIR}/lib/SdFat/ExFatLib/*.h")
file(GLOB HEADER_LIST CONFIGURE_DEPENDS "${DRONE_Code_SOURCE_DIR}/lib/SdFat/FatLib/*.h")
file(GLOB HEADER_LIST CONFIGURE_DEPENDS "${DRONE_Code_SOURCE_DIR}/lib/SdFat/FsLib/*.h")
file(GLOB HEADER_LIST CONFIGURE_DEPENDS "${DRONE_Code_SOURCE_DIR}/lib/SdFat/iostream/*.h")
file(GLOB HEADER_LIST CONFIGURE_DEPENDS "${DRONE_Code_SOURCE_DIR}/lib/SdFat/SdCard/*.h")
file(GLOB HEADER_LIST CONFIGURE_DEPENDS "${DRONE_Code_SOURCE_DIR}/lib/SdFat/SpiDriver/*.h")

add_library(SdFat 
	FreeStack.cpp
	MinimumSerial.cpp
	# common
	common/FmtNumber.cpp
	common/FsCache.cpp
	common/FsDateTime.cpp
	common/FsName.cpp
	common/FsStructs.cpp
	common/FsUtf.cpp
	common/upcase.cpp
	# ExFatLib
	ExFatLib/ExFatDbg.cpp
	ExFatLib/ExFatFile.cpp
	ExFatLib/ExFatFilePrint.cpp
	ExFatLib/ExFatFileWrite.cpp
	ExFatLib/ExFatFormatter.cpp
	ExFatLib/ExFatName.cpp
	ExFatLib/ExFatPartition.cpp
	ExFatLib/ExFatVolume.cpp
	# FatLib
	FatLib/FatDbg.cpp
	FatLib/FatFile.cpp
	FatLib/FatFileLFN.cpp
	FatLib/FatFilePrint.cpp
	FatLib/FatFileSFN.cpp
	FatLib/FatFormatter.cpp
	FatLib/FatName.cpp
	FatLib/FatPartition.cpp
	FatLib/FatVolume.cpp
	# FsLib
	FsLib/FsFile.cpp
	FsLib/FsNew.cpp
	FsLib/FsVolume.cpp
	# iostream
	iostream/istream.cpp
	iostream/ostream.cpp
	iostream/StdioStream.cpp
	iostream/StreamBaseClass.cpp
	# SdCard
	SdCard/SdCardInfo.cpp
	SdCard/SdioTeensy.cpp
	SdCard/SdSpiCard.cpp
	# SpiDriver
	SpiDriver/SdSpiArtemis.cpp
	SpiDriver/SdSpiChipSelect.cpp
	SpiDriver/SdSpiDue.cpp
	SpiDriver/SdSpiESP.cpp
	SpiDriver/SdSpiParticle.cpp
	SpiDriver/SdSpiSTM32.cpp
	SpiDriver/SdSpiSTM32Core.cpp
	SpiDriver/SdSpiTeensy3.cpp

)

target_include_directories(SdFat PUBLIC 
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

target_link_libraries(SdFat PUBLIC 
	Teensy4
	SPI
)
