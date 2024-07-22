# C++ and C flags
# ref: cmake/px4_add_common_flags.cmake
# https://gcc.gnu.org/onlinedocs/gcc/Warning-Options.html
add_compile_options(
  -g # always build debug symbols
  # optimization options
  -fdata-sections
  -ffunction-sections
  -fomit-frame-pointer
  -fmerge-all-constants
  #-funsafe-math-optimizations # Enables -fno-signed-zeros, -fno-trapping-math, -fassociative-math and -freciprocal-math
  -fno-signed-zeros	# Allow optimizations for floating-point arithmetic that ignore the signedness of zero
  -fno-trapping-math	# Compile code assuming that floating-point operations cannot generate user-visible traps
  #-fassociative-math	# Allow re-association of operands in series of floating-point operations
  -freciprocal-math	# Allow the reciprocal of a value to be used instead of dividing by the value if this enables optimizations
  -fno-math-errno		# Do not set errno after calling math functions that are executed with a single instruction, e.g., sqrt
  -fno-strict-aliasing
  # Warnings
  -Wall
  -Wextra
  # -Werror # TODO(llhuang) fix Nuttx build
  -Wno-sign-compare # fix Nuttx build
  -Wno-address # fix Nuttx build
  -Warray-bounds
  -Wcast-align
  -Wdisabled-optimization
  -Wdouble-promotion
  -Wfatal-errors
  -Wfloat-equal
  -Wformat-security
  -Winit-self
  -Wlogical-op
  -Wpointer-arith
  -Wshadow
  -Wuninitialized
  -Wunknown-pragmas
  -Wunused-variable
  # disabled warnings
  -Wno-missing-field-initializers
  -Wno-missing-include-dirs # TODO: fix and enable
  -Wno-unused-parameter
)
# force color for gcc > 4.9
add_compile_options(-fdiagnostics-color=always)
add_compile_options(
  -fno-builtin-printf
  -fno-strength-reduce
  -Wformat=1
  -Wunused-but-set-variable
  -Wno-format-truncation # TODO: fix
)

# C only flags
set(C_FLAGS)
list(APPEND c_flags
  -fno-common
  -Wbad-function-cast
  -Wnested-externs
  -Wstrict-prototypes
)
foreach(flag ${C_FLAGS})
  add_compile_options($<$<COMPILE_LANGUAGE:C>:${flag}>)
endforeach()

# CXX only flags
set(CXX_FLAGS)
list(APPEND CXX_FLAGS
  -fno-exceptions
  -fno-threadsafe-statics
  -Wreorder
  # disabled warnings
  -Wno-overloaded-virtual # TODO: fix and remove
)
foreach(flag ${CXX_FLAGS})
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:${flag}>)
endforeach()


# Enable assembler files preprocessing
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)
