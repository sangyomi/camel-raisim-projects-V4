cmake_minimum_required(VERSION 3.5)
project(LowController LANGUAGES CXX)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${qpOASES_SOURCE_DIR}/include
        ${canine_util_SOURCE_DIR}/include
        ${ControlUtils_SOURCE_DIR}/include
)

link_directories(
        ${qpOASES_SOURCE_DIR}/bin
)

add_library(${PROJECT_NAME} STATIC
        src/LowPDcontrol.cpp
        src/LowWholeBodyPDController.cpp
        )

add_library(canine_controller::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}
        qpOASES
        canine_controller::ControlUtils
)