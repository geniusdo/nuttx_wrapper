add_module(
  MODULE modules__dshot
  MAIN dshot
  SRCS
  dshot_main.cc
  STACK_MAIN 16384
  DEPENDS nuttx_arch nuttx_m
)
