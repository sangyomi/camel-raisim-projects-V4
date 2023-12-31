cmake_minimum_required(VERSION 3.5)
project(canine_simulation LANGUAGES CXX)

find_package(camel-tools-thread REQUIRED)
find_package(Qt5 COMPONENTS Widgets Network REQUIRED)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${qpOASES_SOURCE_DIR}/include

        ${canine_gui_SOURCE_DIR}/include

        ${canine_util_SOURCE_DIR}/include

        ${ControlUtils_SOURCE_DIR}/include
        ${PDcontroller_SOURCE_DIR}/include
        ${convexMPC_SOURCE_DIR}/include
        ${LowController_SOURCE_DIR}/include
)
link_directories(
        ${qpOASES_SOURCE_DIR}/bin
)

link_libraries(pthread)

add_library(${PROJECT_NAME}
        STATIC
        src/SimulCommand.cpp
        src/SimulControlPanel.cpp
        src/SimulControlPanelHigh.cpp
        src/SimulMain.cpp
        src/SimulStateEstimator.cpp
        src/SimulVisualizer.cpp
        src/SimulCommunication.cpp
        )

target_link_libraries(${PROJECT_NAME}
        raisim
        camel-tools-thread
        camel-tools-filter

        canine_visualizer::canine_gui
        canine_controller::ControlUtils
        canine_controller::LowController
        canine_controller::convexMPC
        canine_controller::PDcontroller
        canine_util

        Qt5::Network
)