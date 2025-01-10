add_module(
  MODULE modules__serial
  MAIN serial
  SRCS
    serial_main.cc
  STACK_MAIN 16384
  DEPENDS nuttx_arch nuttx_m
)
