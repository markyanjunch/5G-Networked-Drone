cmake_minimum_required(VERSION 2.8)
project(VE450)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -g -O0")

include_directories(${ONBOARDSDK_SOURCE}/api/inc)
include_directories(${ONBOARDSDK_SOURCE}/utility/inc)
include_directories(${ONBOARDSDK_SOURCE}/hal/inc)
include_directories(${ONBOARDSDK_SOURCE}/protocol/inc)
include_directories(${ONBOARDSDK_SOURCE}/platform/linux/inc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../core/inc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../common)
FILE(GLOB SOURCE_FILES *.hpp *.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../common/dji_linux_environment.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../common/dji_linux_helpers.cpp

        ${CMAKE_CURRENT_SOURCE_DIR}/../../../../osdk-core/api/inc/*.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../../osdk-core/hal/inc/*.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../../osdk-core/modules/inc/*.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../../osdk-core/platform/linux/inc/*.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../../osdk-core/platform/manifold/inc/*.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../../osdk-core/protocol/inc/*.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../../osdk-core/utility/inc/*.hpp
        )

add_executable(${PROJECT_NAME} ${SOURCE_FILES} VE450.hpp VE450.cpp)
target_link_libraries(${PROJECT_NAME} djiosdk-core)
