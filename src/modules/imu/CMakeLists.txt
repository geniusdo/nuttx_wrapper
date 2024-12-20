add_module(
  MODULE modules__imu
  MAIN imu
  SRCS
    imu_main.cc
    bmi08_port.c
    bmi08a.c
    bmi08g.c 
    bmi08xa.c
  INCLUDES
    .
  STACK_MAIN 16384
  DEPENDS nuttx_arch nuttx_m
)
