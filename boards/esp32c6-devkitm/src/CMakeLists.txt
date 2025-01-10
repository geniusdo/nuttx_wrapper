
file(GLOB SRC_FILES *.c *.cc *.h)
add_library(drivers_board ${SRC_FILES})
target_link_libraries(drivers_board nuttx_arch nuttx_drivers)

