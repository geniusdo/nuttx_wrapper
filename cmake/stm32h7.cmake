#config file  for stm32H7 series MCU

#enable hardware floating point

add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING;STM32H7)
add_compile_options(-mfloat-abi=hard -mfpu=fpv5-d16)
add_compile_options(-mcpu=cortex-m7 -mthumb -mthumb-interwork -pipe -isystem)

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m7 -mthumb -mthumb-interwork)
add_link_options(-mfloat-abi=hard -mfpu=fpv5-d16)


execute_process(
  COMMAND ${CMAKE_C_COMPILER} -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -print-file-name=libgcc.a
  OUTPUT_VARIABLE LIBGCC
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
get_filename_component(LIBGCC_PATH ${LIBGCC} DIRECTORY)