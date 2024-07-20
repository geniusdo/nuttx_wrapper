include(FetchContent)

set(RTOS_SOURCE_DIR ${CMAKE_SOURCE_DIR}/rtos)
set(NUTTX_SOURCE_DIR "${RTOS_SOURCE_DIR}/nuttx" CACHE STRING "" FORCE)
set(NUTTX_APP_SOURCE_DIR "${RTOS_SOURCE_DIR}/apps" CACHE STRING "" FORCE)


if(NOT EXISTS ${RTOS_SOURCE_DIR})
  message(STATUS "${BOLD_YELLOW}RTOS directory not found. Creating ${RTOS_SOURCE_DIR}${END}")
  file(MAKE_DIRECTORY ${RTOS_SOURCE_DIR})
endif()

#fetch nuttx
if(NOT EXISTS ${NUTTX_SOURCE_DIR}/LICENSE)
  message(STATUS "${BOLD_BLUE}Fetching nuttx from apache Nuttx github source...${END}")

  FetchContent_Declare(
    nuttx
    GIT_REPOSITORY https://github.com/apache/nuttx.git
    GIT_TAG        master  
    SOURCE_DIR     ${NUTTX_SOURCE_DIR}  
  )

  FetchContent_Populate(nuttx)
else()
  message(STATUS "${BOLD_YELLOW}Using existing nuttx from ${NUTTX_SOURCE_DIR}${END}")
endif()

#fetch nuttx-apps
if(NOT EXISTS ${NUTTX_APP_SOURCE_DIR}/LICENSE)
  message(STATUS "${BOLD_BLUE}Fetching nuttx-apps from apache Nuttx-apps github source...${END}")

  FetchContent_Declare(
    nuttx-apps
    GIT_REPOSITORY https://github.com/apache/nuttx-apps.git
    GIT_TAG        master  
    SOURCE_DIR     ${NUTTX_APP_SOURCE_DIR}  
  )

  FetchContent_Populate(nuttx-apps)
else()
  message(STATUS "${BOLD_YELLOW}Using existing nuttx-apps from ${NUTTX_APP_SOURCE_DIR}${END}")
endif()


