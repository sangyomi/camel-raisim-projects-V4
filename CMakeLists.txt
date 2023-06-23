CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
PROJECT(camel-raisim-projects)

set(RAISIM_PATH "$ENV{HOME}/Library/raisimLib/raisim/linux")
include_directories(${RAISIM_PATH}/include)
link_directories(${RAISIM_PATH}/lib)

ADD_COMPILE_DEFINITIONS (URDF_RSC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/camel-urdf/")

set(CAMEL_CANINE TRUE)
set(CAMEL_CANINE_LEG_LEFT FALSE)
set(CAMEL_THIRDPARTY_QPOASES TRUE)
set(CAME_TOOLS_EXAMPLES FALSE)

if(${CAMEL_THIRDPARTY_QPOASES})
    add_subdirectory(camel-thirdparty/qpOASES)
endif()

if(${CAMEL_CANINE})
    add_subdirectory(camel-canine)
endif()

if(${CAMEL_CANINE_LEG_LEFT})
    add_subdirectory(camel-canine-leg-left)
endif()

if(${CAME_TOOLS_EXAMPLES})
    add_subdirectory(camel-tools)
endif()
