cmake_minimum_required(VERSION 2.8.12)

project(offlineOrientationSensorsIK)

# Include GNUInstallDirs to get canonical paths
include(GNUInstallDirs)

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

# Choose common build locations for executables and libraries
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_DEBUG_POSTFIX "d")

include_directories(${OPENSIMSIMBODY_INCLUDE_DIRS} include)

set (IKExtended_OpenSim_Simulation_Model_HEADERS
                        include/OpenSim/Simulation/Model/OrientationSensor.h
                        include/OpenSim/Simulation/Model/OrientationSensorSet.h)

set (IKExtended_OpenSim_Simulation_HEADERS
                        include/OpenSim/Simulation/OrientationSensorsReference.h
                        include/OpenSim/Simulation/InverseKinematicsExtendedSolver.h
                        include/OpenSim/Simulation/osimExtendedIKDLL.h
                        include/OpenSim/Simulation/RegisterTypes_osimExtendedIK.h)
                        
set (IKExtended_OpenSim_Common_HEADERS
                        include/OpenSim/Common/OrientationSensorFrame.h
                        include/OpenSim/Common/OrientationSensorData.h)

set (IKExtended_OpenSim_Tools_HEADERS
                        include/OpenSim/Tools/IKExtendedTaskSet.h
                        include/OpenSim/Tools/IKOrientationSensorTask.h
                        include/OpenSim/Tools/InverseKinematicsExtendedTool.h)

set (IKExtended_HEADERS ${IKExtended_OpenSim_Simulation_HEADERS}
                        ${IKExtended_OpenSim_Simulation_Model_HEADERS}
                        ${IKExtended_OpenSim_Common_HEADERS}
                        ${IKExtended_OpenSim_Tools_HEADERS})

set (IKExtended_SOURCES src/OrientationSensor.cpp
                        src/OrientationSensorSet.cpp
                        src/OrientationSensorData.cpp
                        src/OrientationSensorFrame.cpp
                        src/OrientationSensorsReference.cpp
                        src/InverseKinematicsExtendedSolver.cpp
                        src/IKOrientationSensorTask.cpp
                        src/InverseKinematicsExtendedTool.cpp
                        src/RegisterTypes_osimExtendedIK.cpp)

add_library(IKExtended ${IKExtended_HEADERS} ${IKExtended_SOURCES})
target_link_libraries(IKExtended ${OPENSIMSIMBODY_LIBRARIES})
target_compile_definitions(IKExtended PRIVATE OSIMEXTENDEDIK_EXPORTS)

install(TARGETS IKExtended
        EXPORT IKExtendedTargets
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT Applications
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Development
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Development)

install (FILES ${IKExtended_OpenSim_Common_HEADERS}
         DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/OpenSim/Common COMPONENT Development)
install (FILES ${IKExtended_OpenSim_Simulation_HEADERS}
         DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/OpenSim/Simulation COMPONENT Development)
install (FILES ${IKExtended_OpenSim_Simulation_Model_HEADERS}
         DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/OpenSim/Simulation/Model COMPONENT Development)
install (FILES ${IKExtended_OpenSim_Tools_HEADERS}
         DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/OpenSim/Tools COMPONENT Development)

add_executable(oSensorsIK testOSensorsIK.cpp )
target_link_libraries(oSensorsIK IKExtended ${OPENSIMSIMBODY_LIBRARIES})

add_executable(testModelIO testModelIO.cpp)
target_link_libraries(testModelIO IKExtended ${OPENSIMSIMBODY_LIBRARIES})

# Create IKExtentedConfig.cmake etc.
set(ConfigPackageLocation ${CMAKE_INSTALL_LIBDIR}/IKExtended)
install(EXPORT IKExtendedTargets
        FILE IKExtendedTargets.cmake
        DESTINATION ${ConfigPackageLocation})

include(CMakePackageConfigHelpers)
configure_package_config_file(cmake/IKExtendedConfig.cmake.in
                              ${CMAKE_CURRENT_BINARY_DIR}/IKExtendedConfig.cmake
                              INSTALL_DESTINATION ${ConfigPackageLocation}
                              PATH_VARS CMAKE_INSTALL_INCLUDEDIR)

#write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/IKExtendedConfigVersion.cmake
#                                    VERSION ${_MAJOR_VERSION}.${_MINOR_VERSION}.${_PATCH_VERSION}
#                                    COMPATIBILITY ExactVersion)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/IKExtendedConfig.cmake
#              ${CMAKE_CURRENT_BINARY_DIR}/IKExtendedConfigVersion.cmake
              DESTINATION ${ConfigPackageLocation})

# Uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(UNINSTALL
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
