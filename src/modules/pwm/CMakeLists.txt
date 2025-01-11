add_module(
  MODULE modules__pwm
  MAIN pwm
  SRCS
  pwm_main.cc
  STACK_MAIN 16384
  DEPENDS nuttx_arch nuttx_m
)
