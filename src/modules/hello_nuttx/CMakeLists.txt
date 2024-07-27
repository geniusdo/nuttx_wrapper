add_module(
  MODULE modules__hello_nuttx
  MAIN hello_nuttx
  SRCS
    hello_nuttx_main.cc
    hello_nuttx.cc
    hello_nuttx.h
  DEPENDS nuttx_arch
)
