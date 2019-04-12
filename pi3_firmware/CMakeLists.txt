
cmake_minimum_required(VERSION 2.8)
project(gimbalTelemetryDataToMicro)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -g -O0")

if(NOT ONBOARDSDK_SOURCE)
   set(ONBOARDSDK_SOURCE "/home/pi/Desktop/Onboard-SDK-3.8/osdk-core/")
endif()

include_directories(${ONBOARDSDK_SOURCE}/api/inc)
include_directories(${ONBOARDSDK_SOURCE}/utility/inc)
include_directories(${ONBOARDSDK_SOURCE}/hal/inc)
include_directories(${ONBOARDSDK_SOURCE}/protocol/inc)
include_directories(${ONBOARDSDK_SOURCE}/platform/linux/inc)

if(NOT ONBOARDSDK_LINUX_HELPERS)
   set(ONBOARDSDK_LINUX_HELPERS "/home/pi/Desktop/Onboard-SDK-3.8/sample/linux/common")
endif()

include_directories(${ONBOARDSDK_LINUX_HELPERS})
FILE(GLOB SOURCE_FILES *.hpp main.cpp *.h
        ${ONBOARDSDK_LINUX_HELPERS}/dji_linux_environment.cpp
        ${ONBOARDSDK_LINUX_HELPERS}/dji_linux_helpers.cpp
        )
     configure_file(UserConfig.txt UserConfig.txt COPYONLY)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} djiosdk-core)
