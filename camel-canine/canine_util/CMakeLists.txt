cmake_minimum_required(VERSION 3.5)
project(canine_util LANGUAGES CXX)

find_package(camel-tools-thread)
find_package(RBDL COMPONENTS URDFREADER REQUIRED)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${RBDL_INCULDE_DIR}
)

link_libraries(stdc++fs)

add_library(canine_util
        SHARED
        src/CanMotorLF.cpp
        src/CanMotorRF.cpp
        src/CanMotorLB.cpp
        src/CanMotorRB.cpp
        src/Command.cpp
        src/StateEstimator.cpp
        src/RobotMath.cpp
        src/Filter.cpp
        src/DataAnalysis.cpp)
target_link_libraries(canine_util
        PUBLIC
        raisim
        PRIVATE
        pthread
        camel-tools-thread
        ${RBDL_LIBRARY}
        ${RBDL_URDFREADER_LIBRARY}
)