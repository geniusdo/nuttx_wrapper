# Get all modules and libraries
set(builtin_apps_string)
set(builtin_apps_decl_string)
get_property(module_libraries GLOBAL PROPERTY CUSTOM_MODULE_LIBRARIES)
message(STATUS "Found ${module_libraries}")
list(SORT module_libraries)
foreach(module ${module_libraries})
  get_target_property(MAIN ${module} MAIN)
  get_target_property(STACK_MAIN ${module} STACK_MAIN)
  get_target_property(PRIORITY ${module} PRIORITY)
  message(STATUS "Adding ${module} to builtin apps")
  if(MAIN)
    set(builtin_apps_string "${builtin_apps_string}{ \"${MAIN}\", ${PRIORITY}, ${STACK_MAIN}, ${MAIN}_main},\n")
    set(builtin_apps_decl_string "${builtin_apps_decl_string}int ${MAIN}_main(int argc, char *argv[]);\n")
  endif()
endforeach()

configure_file(${CMAKE_SOURCE_DIR}/cmake/app.bdat.in ${CMAKE_CURRENT_BINARY_DIR}/app.bdat)
configure_file(${CMAKE_SOURCE_DIR}/cmake/app.pdat.in ${CMAKE_CURRENT_BINARY_DIR}/app.pdat)


# #-------------------------------------------------------
# # ROMFS
# #-------------------------------------------------------
add_custom_command( 
  OUTPUT  rc.sysinit.template
  OUTPUT  rcS.template
  COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_BOARD_PATH}/${NUTTX_BOARD}/src/etc/init.d/rc.sysinit ${CMAKE_BINARY_DIR}/rc.sysinit.template
  COMMAND ${CMAKE_COMMAND} -E copy ${NUTTX_BOARD_PATH}/${NUTTX_BOARD}/src/etc/init.d/rcS ${CMAKE_BINARY_DIR}/rcS.template
  COMMENT "ROMFS: copy sysinit file."
  DEPENDS ${NUTTX_BOARD_PATH}/${NUTTX_BOARD}/src/etc/init.d/rc.sysinit
  DEPENDS ${NUTTX_BOARD_PATH}/${NUTTX_BOARD}/src/etc/init.d/rcS
)

# create etc_romfs.c
find_program(XXD xxd)
if(NOT XXD)
  message(FATAL_ERROR "xxd not found")
endif()

find_program(SED sed)
if(NOT SED)
  message(FATAL_ERROR "sed not found")
endif()

add_custom_command(
    OUTPUT etc_romfs.c
    COMMAND /bin/bash ${NUTTX_SOURCE_DIR}/tools/mkromfsimg.sh ${NUTTX_SOURCE_DIR} rc.sysinit.template rcS.template
    COMMAND ${CMAKE_COMMAND} -E remove ${NUTTX_BOARD_PATH}/${NUTTX_BOARD}/src/etc_romfs.c
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "ROMFS: generating etc_romfs.c"
    DEPENDS rc.sysinit.template
    DEPENDS rcS.template
)

add_custom_command(
  OUTPUT  ${NUTTX_BOARD_PATH}/${NUTTX_BOARD}/src/etc_romfs.c
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/etc_romfs.c ${NUTTX_BOARD_PATH}/${NUTTX_BOARD}/src/etc_romfs.c
  COMMENT "ROMFS: etc_romfs.c."
  DEPENDS ${CMAKE_BINARY_DIR}/etc_romfs.c
)

#-------------------------------------------------------
# Build Apps (libapps.a)
#-------------------------------------------------------
file(GLOB_RECURSE nuttx_apps_files LIST_DIRECTORIES false
  ${NUTTX_APP_SOURCE_DIR}/*.c
  ${NUTTX_APP_SOURCE_DIR}/*.h
)
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/apps/libapps.a
  COMMAND ${CMAKE_COMMAND} -E remove -f ${NUTTX_APP_SOURCE_DIR}/libapps.a ${NUTTX_APP_SOURCE_DIR}/builtin/builtin_list.h ${NUTTX_APP_SOURCE_DIR}/builtin/builtin_proto.h
  COMMAND find ${NUTTX_APP_SOURCE_DIR} -type f \\\( -name "*.o" -o -name "*lib*.a" -o -name "*.depend" -o -name "*.dep" \\\) -delete
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_BINARY_DIR}/app.bdat ${NUTTX_APP_SOURCE_DIR}/builtin/registry/app.bdat
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_BINARY_DIR}/app.pdat ${NUTTX_APP_SOURCE_DIR}/builtin/registry/app.pdat
  COMMAND ${CMAKE_COMMAND} -E touch_nocreate ${NUTTX_APP_SOURCE_DIR}/builtin/registry/.updated
  COMMAND make --no-print-directory --silent TOPDIR="${NUTTX_SOURCE_DIR}" > ${CMAKE_CURRENT_BINARY_DIR}/nuttx_apps.log
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_APP_SOURCE_DIR}/libapps.a ${CMAKE_CURRENT_BINARY_DIR}/apps/libapps.a
  COMMENT "Build Nuttx apps."
  DEPENDS
    ${nuttx_apps_files}
    nuttx_context
    ${NUTTX_SOURCE_DIR}/include/nuttx/config.h
    ${NUTTX_BOARD_PATH}/${NUTTX_BOARD}/src/etc_romfs.c
  WORKING_DIRECTORY ${NUTTX_APP_SOURCE_DIR}
)

add_custom_target(nuttx_apps_build DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/apps/libapps.a)
add_library(nuttx_apps STATIC IMPORTED GLOBAL)
set_property(TARGET nuttx_apps PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/apps/libapps.a)
add_dependencies(nuttx_apps nuttx_apps_build)
