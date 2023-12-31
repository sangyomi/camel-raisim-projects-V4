cmake_minimum_required(VERSION 3.5)
project(PDcontroller LANGUAGES CXX)

find_package(camel-tools-trajectory REQUIRED)

find_package(camel-tools-thread REQUIRED)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${qpOASES_SOURCE_DIR}/include
        ${canine_util_SOURCE_DIR}/include
        ${ControlUtils_SOURCE_DIR}/include
        ${LowController_SOURCE_DIR}/include
)

link_directories(
        ${qpOASES_SOURCE_DIR}/bin
)

add_library(${PROJECT_NAME} STATIC
        src/JointPDController.cpp
        )

add_library(canine_controller::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}
        qpOASES
        camel-tools-trajectory
        raisim
        canine_util
        canine_controller::ControlUtils
        canine_controller::LowController
        camel-tools-thread
)