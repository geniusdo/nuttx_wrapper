# Ref: PX4

#=============================================================================
#
#	px4_add_module
#
#	This function builds a static library from a module description.
#
#	Usage:
#		px4_add_module(MODULE <string>
#			MAIN <string>
#			[ STACK_MAIN <string> ]
#			[ STACK_MAX <string> ]
#			[ COMPILE_FLAGS <list> ]
#			[ INCLUDES <list> ]
#			[ DEPENDS <string> ]
#			[ SRCS <list> ]
#			[ MODULE_CONFIG <list> ]
#			[ EXTERNAL ]
#			[ DYNAMIC ]
#			)
#
#	Input:
#		MODULE			: unique name of module
#		MAIN			: entry point
#		STACK			: deprecated use stack main instead
#		STACK_MAIN		: size of stack for main function
#		STACK_MAX		: maximum stack size of any frame
#		COMPILE_FLAGS		: compile flags
#		LINK_FLAGS		: link flags
#		SRCS			: source files
#		MODULE_CONFIG		: yaml config file(s)
#		INCLUDES		: include directories
#		DEPENDS			: targets which this module depends on
#		EXTERNAL		: flag to indicate that this module is out-of-tree
#		DYNAMIC			: don't compile into the px4 binary, but build a separate dynamically loadable module (posix)
#		UNITY_BUILD		: merge all source files and build this module as a single compilation unit
#
#	Output:
#		Static library with name matching MODULE.
#		(Or a shared library when DYNAMIC is specified.)
#
#	Example:
#		px4_add_module(MODULE test
#			SRCS
#				file.cpp
#			STACK_MAIN 1024
#			DEPENDS
#				git_nuttx
#			)
#

include(misc)
function(add_module)
  parse_function_args(
    NAME add_module
    ONE_VALUE MODULE MAIN STACK_MAIN STACK_MAX PRIORITY
    MULTI_VALUE COMPILE_FLAGS LINK_FLAGS SRCS INCLUDES DEPENDS MODULE_CONFIG
    OPTIONS EXTERNAL DYNAMIC UNITY_BUILD
    REQUIRED MODULE MAIN
    ARGN ${ARGN}
  )
  # Nuttx
  add_library(${MODULE} STATIC EXCLUDE_FROM_ALL ${SRCS})
  # all modules can potentially use parameters and uORB
	# add_dependencies(${MODULE} uorb_headers)

  # check if the modules source dir exists in config_kernel_list
  # in this case, treat is as a kernel side compoent for protected build
  # get_target_property(MODULE_SOURCE_DIR ${MODULE} SOURCE_DIR)
  # module relative path = ${MODULE_SOURCE_DIR} - {PROJECT_SOURCE_DIR}/src
  # file(RELATIVE_PATH module ${PROJECT_SOURCE_DIR}/src ${MODULE_SOURCE_DIR})

  # target_link_libraries(${MODULE} PRIVATE prebuild_targets px4_platform systemlib perf)
  # target_link_libraries(${MODULE} PRIVATE parameters_interface px4_layer uORB)
  set_property(GLOBAL APPEND PROPERTY CUSTOM_MODULE_LIBRARIES ${MODULE})
  set_property(GLOBAL APPEND PROPERTY CUSTOM_MODULE_PATHS ${CMAKE_CURRENT_SOURCE_DIR})
  list_make_absolute(ABS_SRCS ${CMAKE_CURRENT_SOURCE_DIR} ${SRCS})
  set_property(GLOBAL APPEND PROPERTY CUSTOM_SRC_FILES ${ABS_SRCS})

  # set defaults if not set
  set(MAIN_DEFAULT MAIN-NOTFOUND)
  set(STACK_MAIN_DEFAULT 2048)
  set(PRIORITY_DEFAULT SCHED_PRIORITY_DEFAULT)
  # set module entry point(main), stack size, priority
  foreach(property MAIN STACK_MAIN PRIORITY)
    if(NOT ${property})
      set(${property} ${${property}_DEFAULT})
    endif()
    set_target_properties(${MOUDLE} PROPERTIES ${property} ${${property}})
  endforeach()
  # default stack max to stack main
  if(NOT STACK_MAX)
    set(STACK_MAX ${STACK_MAIN})
  endif()
	set_target_properties(${MODULE} PROPERTIES STACK_MAX ${STACK_MAX})
  target_compile_options(${MODULE} PRIVATE -Wframe-larger-than=${STACK_MAX})
  # module entry point - MAIN
  if(MAIN)
    target_compile_definitions(${MODULE} PRIVATE DO_MAIN=${MAIN}_app_main)
    target_compile_definitions(${MODULE} PRIVATE MODULE_NAME="{MAIN}")
  else()
    message(FATAL_ERROR "MAIN required")
  endif()

  if(COMPILE_FLAGS)
    target_compile_options(${MODULE} PRIVATE ${COMPILE_FLAGS})
  endif()

  if(INCLUDES)
    target_include_directories(${MODULE} PRIVATE ${INCLUDES})
  endif()

  if(DEPENDS)
    # using target_link_libraries for dependencies provides linking
    # as well as interface include and libraries
    foreach(dep ${DEPENDS})
      get_target_property(dep_type ${dep} TYPE)
      if((${dep_type} STREQUAL "STATIC_LIBRARY") OR (${dep_type} STREQUAL "INTERFACE_LIBRARY"))
        target_link_libraries(${MODULE} PRIVATE ${dep})
      else()
        add_dependencies(${MODULE} ${dep})
      endif()
    endforeach()
  endif()

  foreach(prop LINK_FLAGS STACK_MAIN MAIN PRIORITY)
    if(${prop})
      # set_target_properties(${MOUDLE} PROPERTIES ${property} ${${property}})
      set_target_properties(${MODULE} PROPERTIES ${prop} ${${prop}})
    endif()
  endforeach()

  # if(MODULE_CONFIG)
  #   foreach(module_config ${MODULE_CONFIG})
  #     set_property(GLOBAL APPEND PROPERTY PX4_MODULE_CONFIG_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${module_config})
  #   endforeach()
  # endif()
endfunction()
