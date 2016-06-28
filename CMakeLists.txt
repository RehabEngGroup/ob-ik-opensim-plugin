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

set(BUILD_SHARED_LIBS "TRUE" CACHE BOOL "Build shared libraries." FORCE)

include_directories(${OPENSIMSIMBODY_INCLUDE_DIRS} include)
set (IKextended_HEADERS include/OpenSim/Simulation/Model/OrientationSensor.h
                        include/OpenSim/Simulation/Model/OrientationSensorSet.h
                        include/OpenSim/Common/OrientationSensorFrame.h
                        include/OpenSim/Common/OrientationSensorData.h
                        include/OpenSim/Simulation/OrientationSensorsReference.h
                        include/OpenSim/Simulation/InverseKinematicsExtendedSolver.h
                        include/OpenSim/Simulation/osimExtendedIKDLL.h
                        include/OpenSim/Tools/IKExtendedTaskSet.h
                        include/OpenSim/Tools/IKOrientationSensorTask.h
                        include/OpenSim/Tools/InverseKinematicsExtendedTool.h)

set (IKextended_SOURCES src/OrientationSensor.cpp
                        src/OrientationSensorSet.cpp
                        src/OrientationSensorData.cpp
                        src/OrientationSensorFrame.cpp
                        src/OrientationSensorsReference.cpp
                        src/InverseKinematicsExtendedSolver.cpp
                        src/IKOrientationSensorTask.cpp
                        src/InverseKinematicsExtendedTool.cpp)

add_library(IKextended ${IKextended_HEADERS} ${IKextended_SOURCES})
target_link_libraries(IKextended ${OPENSIMSIMBODY_LIBRARIES})
target_compile_definitions(IKextended PRIVATE OSIMEXTENDEDIK_EXPORTS)

install (TARGETS IKextended DESTINATION lib)
install (FILES ${IKextended_HEADERS}        
         DESTINATION include)

add_executable(oSensorsIK testOSensorsIK.cpp )
target_link_libraries(oSensorsIK ${OPENSIMSIMBODY_LIBRARIES} IKextended)
#target_compile_definitions(oSensorsIK PRIVATE OSIMEXTENDEDIK_EXPORTS)

add_executable(testModelIO testModelIO.cpp)
target_link_libraries(testModelIO IKextended ${OPENSIMSIMBODY_LIBRARIES})
#target_compile_definitions(testModelIO PRIVATE OSIMEXTENDEDIK_EXPORTS)