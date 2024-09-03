function (flashMcu TARGET)
  string(APPEND HEX_TARGET ${TARGET} _hex)
  string(APPEND HEX_CMD ${TARGET} .hex)
  string(APPEND UPLOAD_TARGET ${TARGET} _upload)
  # Add 'make hex' target
  add_custom_command(OUTPUT ${HEX_CMD}
    DEPENDS ${TARGET}
    COMMAND ${CMAKE_SIZE} ${TARGET}
    COMMAND ${CMAKE_OBJCOPY} -O ihex -R .eeprom ${TARGET} ${HEX_CMD}
  )
  add_custom_target(${HEX_TARGET}
    DEPENDS ${HEX_CMD}
  )


	add_custom_target(${UPLOAD_TARGET}
		COMMAND ${CMAKE_SOURCE_DIR}/tools/${CMAKE_HOST_SYSTEM_NAME}/teensy_post_compile -file=${TARGET} -path=${CMAKE_CURRENT_BINARY_DIR} -tools=${CMAKE_SOURCE_DIR}/tools/${CMAKE_HOST_SYSTEM_NAME} -reboot
		DEPENDS ${HEX_CMD}
	)
  # Linker
  set_property(TARGET ${TARGET} PROPERTY LINK_DEPENDS ${LINKER_SCRIPT})
endfunction ()
