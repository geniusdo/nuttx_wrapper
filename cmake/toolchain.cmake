set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Toolchain
set(COMPILER_PREFIX "arm-none-eabi")

# find arm-none-eabi-gcc binary
execute_process(
  COMMAND which ${COMPILER_PREFIX}-gcc
  OUTPUT_VARIABLE TOOLCHAIN_GCC_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
# extract toolchain path
get_filename_component(TOOLCHAIN_PATH ${TOOLCHAIN_GCC_PATH} DIRECTORY)
message(STATUS "${BOLD_GREEN}Toolchain path: ${TOOLCHAIN_PATH}${END}")

# cmake-format: off
find_program(CMAKE_C_COMPILER NAMES ${COMPILER_PREFIX}-gcc HINTS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_CXX_COMPILER NAMES ${COMPILER_PREFIX}-g++ HINTS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_AR NAMES ${COMPILER_PREFIX}-ar HINTS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_RANLIB NAMES ${COMPILER_PREFIX}-ranlib HINTS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_LINKER NAMES ${COMPILER_PREFIX}-ld HINTS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_ASM_COMPILER NAMES ${COMPILER_PREFIX}-gcc HINTS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_OBJCOPY NAMES ${COMPILER_PREFIX}-objcopy HINTS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_OBJDUMP NAMES ${COMPILER_PREFIX}-objdump HINTS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_SIZE NAMES ${COMPILER_PREFIX}-size HINTS ${TOOLCHAIN_BIN_PATH})

# https://cmake.org/cmake/help/latest/manual/cmake-toolchains.7.html#cmake-toolchains-7
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Define the compiler to be used for C and C++
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)

# Define the assembler to be used
set(CMAKE_ASM_COMPILER arm-none-eabi-as)

# Define the tool for archiving libraries
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_NM arm-none-eabi-nm)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(CMAKE_RANLIB arm-none-eabi-ranlib)
set(CMAKE_SIZE arm-none-eabi-size)

# Define the linker
set(CMAKE_EXE_LINKER arm-none-eabi-gcc)
