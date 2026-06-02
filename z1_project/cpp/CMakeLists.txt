cmake_minimum_required(VERSION 3.10)
project(z1_pure_torque_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT DEFINED UNITREE_ARM_SDK_PATH)
    if(DEFINED ENV{UNITREE_ARM_SDK_PATH})
        set(UNITREE_ARM_SDK_PATH "$ENV{UNITREE_ARM_SDK_PATH}")
    endif()
endif()

if(NOT UNITREE_ARM_SDK_PATH)
    message(FATAL_ERROR "Unitree Arm SDK not found. Set UNITREE_ARM_SDK_PATH, for example: cmake .. -DUNITREE_ARM_SDK_PATH=/path/to/unitree_arm_sdk")
endif()

set(UNITREE_ARM_SDK_PATH "${UNITREE_ARM_SDK_PATH}" CACHE PATH "Path to Unitree Arm SDK")

find_path(UNITREE_ARM_INCLUDE_DIR
    NAMES unitree_arm_sdk/control/unitreeArm.h
    PATHS
        "${UNITREE_ARM_SDK_PATH}/include"
        "${UNITREE_ARM_SDK_PATH}"
    NO_DEFAULT_PATH
)

find_library(UNITREE_ARM_LIBRARY
    NAMES
        Z1_SDK_${CMAKE_SYSTEM_PROCESSOR}
        unitree_arm_interface
        unitree_arm_sdk
        unitree_arm
        unitreeArm
    PATHS
        "${UNITREE_ARM_SDK_PATH}/lib"
        "${UNITREE_ARM_SDK_PATH}/lib64"
        "${UNITREE_ARM_SDK_PATH}/build"
        "${UNITREE_ARM_SDK_PATH}/build/lib"
    NO_DEFAULT_PATH
)

if(NOT UNITREE_ARM_INCLUDE_DIR)
    message(FATAL_ERROR "Could not find unitree_arm_sdk/control/unitreeArm.h under UNITREE_ARM_SDK_PATH=${UNITREE_ARM_SDK_PATH}")
endif()

if(NOT UNITREE_ARM_LIBRARY)
    message(FATAL_ERROR "Could not find Unitree Arm SDK library under UNITREE_ARM_SDK_PATH=${UNITREE_ARM_SDK_PATH}")
endif()

add_executable(pure_torque_bridge pure_torque_bridge.cpp)
target_include_directories(pure_torque_bridge PRIVATE "${UNITREE_ARM_INCLUDE_DIR}")
target_link_libraries(pure_torque_bridge PRIVATE "${UNITREE_ARM_LIBRARY}" pthread)

add_executable(sdk_pure_torque_augpd_test sdk_pure_torque_augpd_test.cpp)
target_include_directories(sdk_pure_torque_augpd_test PRIVATE "${UNITREE_ARM_INCLUDE_DIR}")
target_link_libraries(sdk_pure_torque_augpd_test PRIVATE "${UNITREE_ARM_LIBRARY}" pthread)
