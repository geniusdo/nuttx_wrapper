
cmake_minimum_required(VERSION 3.17)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(misc)
include(toolchain)
project(prototype)
enable_language(ASM)
enable_language(C)
enable_language(CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_SKIP_RPATH ON)
set(BUILD_SHARED_LIBS OFF)

#-------------------------------------------------------------------------
# Macros
#-------------------------------------------------------------------------
if(NOT DEFINED NUTTX_BOARD)
  set(NUTTX_BOARD "prototype" CACHE STRING "" FORCE)
endif()
if(NOT DEFINED NUTTX_BOARD_CONFIG)
  set(NUTTX_BOARD_CONFIG "nsh" CACHE STRING "" FORCE)
endif()
set(NUTTX_BOARD_PATH "${CMAKE_SOURCE_DIR}/boards" CACHE STRING "" FORCE)
message(STATUS "${BOLD_GREEN}Selecting board config ${NUTTX_BOARD}:${NUTTX_BOARD_CONFIG}${END}")

set(MCU_LINKER_SCRIPT "${NUTTX_BOARD_PATH}/${NUTTX_BOARD}/scripts/flash.ld" CACHE STRING "" FORCE)

define_property(GLOBAL PROPERTY CUSTOM_MODULE_LIBRARIES
  BRIEF_DOCS "module libs"
  FULL_DOCS "List of all module libraries"
)
define_property(GLOBAL PROPERTY CUSTOM_MODULE_PATHS
  BRIEF_DOCS "module paths"
  FULL_DOCS "List of all paths for modules"
)
define_property(GLOBAL PROPERTY CUSTOM_SRC_FILES
  BRIEF_DOCS "all source file for modules and libs"
  FULL_DOCS "all source file from add_{module, library}"
)

include(stm32h7)
include(common_flags)
include(add_module)
include(fetch_third_library)

#-------------------------------------------------------------------------
# Nuttx
#-------------------------------------------------------------------------
include(config_nuttx)

include_directories(
  ${NUTTX_SOURCE_DIR}/include
  ${NUTTX_SOURCE_DIR}/include/cxx
  ${NUTTX_SOURCE_DIR}/arch/arm/src/armv7-m
  ${NUTTX_SOURCE_DIR}/arch/arm/src/chip
  ${NUTTX_SOURCE_DIR}/arch/arm/src/common
  ${NUTTX_APP_SOURCE_DIR}/include
)
# Build and config Nuttx and Nuttx Apps
set(NUTTX_LIBS)
list(APPEND NUTTX_LIBS nuttx_sched nuttx_drivers nuttx_boards nuttx_c nuttx_mm nuttx_arch nuttx_xx nuttx_apps nuttx_fs nuttx_binfmt)

#-------------------------------------------------------------------------
# modules and libraries
#-------------------------------------------------------------------------

set(config_module_list)
# Custom board
file(RELATIVE_PATH board_src_rel ${CMAKE_SOURCE_DIR} ${NUTTX_BOARD_PATH}/${NUTTX_BOARD})
if(EXISTS ${NUTTX_BOARD_PATH}/${NUTTX_BOARD}/include)
  include_directories(${NUTTX_BOARD_PATH}/${NUTTX_BOARD}/include)
endif()

list(APPEND config_module_list ${board_src_rel}/src)
list(APPEND config_module_list src/modules/hello_nuttx)
list(APPEND config_module_list src/modules/imu)

# Add all modules
foreach(module ${config_module_list}) 
  add_subdirectory(${module})
endforeach()

include(config_app)

#-------------------------------------------------------------------------
# Build empty exe
#-------------------------------------------------------------------------
set(EXE_NAME ${PROJECT_NAME})
add_executable(${EXE_NAME} empty.cc)

get_property(module_libraries GLOBAL PROPERTY CUSTOM_MODULE_LIBRARIES)
target_link_libraries(${EXE_NAME}
  -nostartfiles
  -nodefaultlibs
  -nostdlib
  -nostdinc++
  -fno-exceptions
  -fno-rtti
  -Wl,--print-memory-usage
  -Wl,--script=${MCU_LINKER_SCRIPT}
  -Wl,-Map=${EXE_NAME}.map
  -Wl,--output=${EXE_NAME}.elf
  -Wl,--warn-common
  -Wl,--gc-sections
)
target_link_libraries(${EXE_NAME} -Wl,--start-group)
target_link_libraries(${EXE_NAME} gcc)
target_link_libraries(${EXE_NAME} ${NUTTX_LIBS})
target_link_libraries(${EXE_NAME} drivers_board)
target_link_libraries(${EXE_NAME} ${module_libraries})
target_link_libraries(${EXE_NAME} -Wl,--end-group)

set(ELF_FILE ${CMAKE_BINARY_DIR}/${EXE_NAME}.elf)
set(BIN_FILE ${CMAKE_BINARY_DIR}/${EXE_NAME}.bin)
set(HEX_FILE ${CMAKE_BINARY_DIR}/${EXE_NAME}.hex)

add_custom_command(
    TARGET ${EXE_NAME}
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${ELF_FILE} ${BIN_FILE}
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${ELF_FILE} ${HEX_FILE}
    DEPENDS ${ELF_FILE}
    COMMENT "Converting ${EXE_NAME}.elf to ${EXE_NAME}.bin and ${EXE_NAME}.hex"
)

# add_custom_target(generate_bin_hex ALL
#     DEPENDS ${BIN_FILE} ${HEX_FILE}
# )
