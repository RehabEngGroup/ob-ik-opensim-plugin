cmake_minimum_required(VERSION 2.8.12)

project(offlineOrientationSensorsIK)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(OpenSim REQUIRED)

# OpenSim uses C++11 language features.
if("${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU" OR
        "${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    # Using C++11 on OSX requires using libc++ instead of libstd++.
    # libc++ is an implementation of the C++ standard library for OSX.
    if(APPLE)
        if(XCODE)
            set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "c++11")
            set(CMAKE_XCODE_ATTRIBUTE_CLANG_CXX_LIBRARY "libc++")
        else()
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
            if(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
                set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
            endif()
        endif()
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    endif()
endif()

include_directories(${OPENSIMSIMBODY_INCLUDE_DIRS} include)
set (IKextended_HEADERS include/OpenSim/Simulation/OrientationSensor.h
                        include/OpenSim/Simulation/osimExtendedIKDLL.h)

set (IKextended_SOURCES src/OrientationSensor.cpp)

add_executable(offlineOrientationSensorsIK offlineOrientationSensorsIK.cpp ${IKextended_HEADERS} ${IKextended_SOURCES})
target_link_libraries(offlineOrientationSensorsIK ${OPENSIMSIMBODY_LIBRARIES})
target_compile_definitions(offlineOrientationSensorsIK PRIVATE OSIMEXTENDEDIK_EXPORTS)
