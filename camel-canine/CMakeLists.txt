cmake_minimum_required(VERSION 3.5)
project(camel-canine LANGUAGES CXX)

find_package(Qt5 CONFIG REQUIRED COMPONENTS Widgets)
find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})

find_package(camel-tools-thread REQUIRED)
find_package(camel-tools-sensor REQUIRED)
find_package(RBDL COMPONENTS URDFREADER REQUIRED)

add_subdirectory(canine_util)
add_subdirectory(canine_controller/ControlUtils)

add_subdirectory(canine_controller/convexMPC)
add_subdirectory(canine_controller/PDcontroller)
add_subdirectory(canine_controller/LowController)
add_subdirectory(canine_controller/ControlMain)

add_subdirectory(canine_visualizer/canine_gui)
add_subdirectory(canine_visualizer/canine_raisim)

add_subdirectory(canine_fsm)

add_subdirectory(canine_simulation)

include_directories(
        ${EIGEN3_INCLUDE_DIRS}
        ${qpOASES_SOURCE_DIR}/include

        ${convexMPC_SOURCE_DIR}/include
        ${PDcontroller_SOURCE_DIR}/include
        ${LowController_SOURCE_DIR}/include
        ${ControlUtils_SOURCE_DIR}/include
        ${ControlMain_SOURCE_DIR}/include
        ${PDQPcontrol_SOURCE_DIR}/include

        ${canine_gui_SOURCE_DIR}/include
        ${canine_raisim_SOURCE_DIR}/include

        ${canine_fsm_SOURCE_DIR}/include
        ${canine_util_SOURCE_DIR}/include
        ${canine_simulation_SOURCE_DIR}/include

        ${RBDL_INCULDE_DIR}
)
link_directories(
        ${qpOASES_SOURCE_DIR}/bin
)

include_directories(/usr/share/c++-mscl/source /usr/share/c++-mscl/Boost/include)
link_directories(/usr/share/c++-mscl)
link_libraries(mscl stdc++ pthread)

add_executable(camel-canine canine_demo/CanineMain.cpp)
target_link_libraries(camel-canine
        PUBLIC
        Qt5::Widgets
        PRIVATE
        canine_fsm
        canine_util
        mscl
        ${RBDL_LIBRARY}
        ${RBDL_URDFREADER_LIBRARY}
)

add_executable(camel-canine-simul canine_demo/CanineSimul.cpp)
target_link_libraries(camel-canine-simul
        PUBLIC
        Qt5::Widgets
        PRIVATE
        canine_simulation
        canine_util
        mscl
        ${RBDL_LIBRARY}
        ${RBDL_URDFREADER_LIBRARY}
)