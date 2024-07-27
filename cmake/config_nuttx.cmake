message(STATUS "${BOLD_MAGENTA}Config Nuttx${END}")

#-------------------------------------------------------
# Config Nuttx
#-------------------------------------------------------
add_custom_command(
  OUTPUT ${NUTTX_SOURCE_DIR}/.config
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_BOARD_PATH}/${NUTTX_BOARD}/scripts/Make.defs ${NUTTX_SOURCE_DIR}/Make.defs
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_BOARD_PATH}/${NUTTX_BOARD}/configs/${NUTTX_BOARD_CONFIG}/defconfig ${NUTTX_SOURCE_DIR}/.config
  COMMAND make olddefconfig
  COMMAND ${CMAKE_COMMAND} -E echo "Copying board config"
  DEPENDS
    ${NUTTX_BOARD_PATH}/${NUTTX_BOARD}/scripts/Make.defs
    ${NUTTX_BOARD_PATH}/${NUTTX_BOARD}/configs/${NUTTX_BOARD_CONFIG}/defconfig
  COMMENT "Board configuration for Nuttx."
  WORKING_DIRECTORY ${NUTTX_SOURCE_DIR}
)

#------------------------------------------------------------------------------------------
# Generate config.h (.config -> include/nuttx/config.h, include/nuttx/version.h, dirlinks)
#------------------------------------------------------------------------------------------
add_custom_command(
  OUTPUT ${NUTTX_SOURCE_DIR}/include/nuttx/config.h
  COMMAND make --no-print-directory --silent clean_context
  COMMAND make --no-print-directory --silent pass1dep
  COMMENT "Generate config.h."
  DEPENDS
    ${NUTTX_SOURCE_DIR}/.config
  WORKING_DIRECTORY ${NUTTX_SOURCE_DIR}
)
add_custom_target(nuttx_context
 DEPENDS ${NUTTX_SOURCE_DIR}/include/nuttx/config.h)


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

#-------------------------------------------------------
# Nuttx
#-------------------------------------------------------
# helper for all targets
function(add_nuttx_dir nuttx_lib nuttx_lib_dir kernel extra target)
  set(nuttx_lib_target all)
  file(GLOB_RECURSE nuttx_lib_files LIST_DIRECTORIES false
    ${CMAKE_CURRENT_SOURCE_DIR}/nuttx/${nuttx_lib_dir}/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/nuttx/${nuttx_lib_dir}/*.h
  )
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/nuttx/${nuttx_lib_dir}/lib${nuttx_lib}.a
		COMMAND ${CMAKE_COMMAND} -E remove -f ${NUTTX_SOURCE_DIR}/${nuttx_lib_dir}/lib${nuttx_lib}.a
		COMMAND find ${nuttx_lib_dir} -type f \\\( -name "*.o" -o -name "*lib*.a" -o -name "*.depend" -o -name "*.dep" \\\) -delete
		COMMAND make -C ${nuttx_lib_dir} --no-print-directory --silent ${nuttx_lib_target} TOPDIR="${NUTTX_SOURCE_DIR}" KERNEL=${kernel} EXTRAFLAGS=${extra} > ${CMAKE_CURRENT_BINARY_DIR}/nuttx_${nuttx_lib}.log
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_SOURCE_DIR}/${nuttx_lib_dir}/lib${nuttx_lib}.a ${CMAKE_CURRENT_BINARY_DIR}/nuttx/${nuttx_lib_dir}/lib${nuttx_lib}.a
    DEPENDS
      ${nuttx_lib_files}
      nuttx_context ${NUTTX_SOURCE_DIR}/include/nuttx/config.h
    WORKING_DIRECTORY ${NUTTX_SOURCE_DIR}
  )
  add_custom_target(nuttx_${nuttx_lib}_build DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/nuttx/${nuttx_lib_dir}/lib${nuttx_lib}.a)
  add_library(nuttx_${nuttx_lib} STATIC IMPORTED GLOBAL)
	set_property(TARGET nuttx_${nuttx_lib} PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/nuttx/${nuttx_lib_dir}/lib${nuttx_lib}.a)
	add_dependencies(nuttx_${nuttx_lib} nuttx_${nuttx_lib}_build)
endfunction()

# add_nuttx_dir(NAME DIRECTORY KERNEL EXTRA)
add_nuttx_dir(binfmt binfmt y -D__KERNEL__ all)
add_nuttx_dir(boards boards y -D__KERNEL__ all)
add_nuttx_dir(drivers drivers y -D__KERNEL__ all)
add_nuttx_dir(fs fs y -D__KERNEL__ all)
add_nuttx_dir(sched sched y -D__KERNEL__ all)
add_nuttx_dir(xx libs/libxx n "" all)
add_nuttx_dir(crypto crypto y -D__KERNEL__ all)
add_nuttx_dir(arch arch/arm/src y -D__KERNEL__ all)
add_nuttx_dir(c libs/libc n "" all)
add_nuttx_dir(mm mm n "" mm)

