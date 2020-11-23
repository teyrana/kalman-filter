CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
PROJECT( tracker )

include(${CMAKE_ROOT}/Modules/ExternalProject.cmake)

#=============================================================================
# Set the output directories for the binary and library files
#=============================================================================

#=============================================================================
# unknown block purpose
#=============================================================================
# set(CMakeScripts_MODULE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
# set(CMAKE_MODULE_PATH ${CMakeScripts_MODULE_DIR})

# ============= Conan-Managed Dependencies =================
include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()
SET(LIBRARY_LINKAGE ${LIBRARY_LINKAGE} ${CONAN_LIBS})

# Set Compile Flags
#=============================================================================
# specify as CMAKE_CXX_FLAGS, instead of 'TARGET_COMPILE_OPTIONS(...' because we have multiple compilation targets.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-rtti")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic ")

# set(CMAKE_CXX_CLANG_TIDY "clang-tidy;-checks=*")
# # Clang-Tidy doesn't check headers by default.  Enable that:
# set(CMAKE_CXX_CLANG_TIDY clang-tidy;-header-filter=.;-checks=*;)

set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} -Og -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -DNDEBUG")

MESSAGE( STATUS ">> Configuring Build Type: ${CMAKE_BUILD_TYPE}")
IF(CMAKE_BUILD_TYPE STREQUAL "Debug")
    MESSAGE( STATUS ".... Configuring Debug Mode.")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -g -DDEBUG")
ELSEIF(CMAKE_BUILD_TYPE STREQUAL "Release")
    MESSAGE( STATUS ".... Configuring Release Mode.")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -O2")
ENDIF(CMAKE_BUILD_TYPE STREQUAL "Debug")

# ============= Enable Testing  =================
#ExternalProject_Add(GTest
#    TMP_DIR ${PROJECT_SOURCE_DIR}/vendor/googletest 
#    GIT_REPOSITORY https://github.com/google/googletest.git) 
#
# INCLUDE_DIRECTORIES(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR}) 

enable_testing()

INCLUDE_DIRECTORIES(${CONAN_INCLUDE_DIRS})

# INCLUDE_DIRECTORIES(extern)

# ============= Eigen Library =================
# Math library
# find_package(eigen REQUIRED)
# list(APPEND LIBRARY_LINKAGE ${EIGEN_LIBRARY})

# # Generate the static library from the sources
# add_library(${CHART_BASE_LIB_NAME} STATIC ${LIB_HEADERS} ${LIB_SOURCES})
# TARGET_LINK_LIBRARIES(${CHART_BASE_LIB_NAME} PRIVATE ${LIBRARY_LINKAGE})
# SET(LIBRARY_LINKAGE ${LIBRARY_LINKAGE} ${CHART_BASE_LIB_NAME})

# ============= IMU Interface Library =================
SET(IMU_INTERFACE_LIB_NAME imu-interface-module)
SET(IMU_INTERFACE_LIB_HEADERS 
            lib/imu/src/command.hpp
            lib/imu/src/connection.hpp )
SET(IMU_INTERFACE_LIB_SOURCES
            lib/imu/src/connection.cpp )

add_library(${IMU_INTERFACE_LIB_NAME} STATIC ${IMU_INTERFACE_LIB_HEADERS} ${IMU_INTERFACE_LIB_SOURCES})

TARGET_LINK_LIBRARIES(${IMU_INTERFACE_LIB_NAME} PRIVATE ${LIBRARY_LINKAGE})

list(APPEND LIBRARY_LINKAGE ${IMU_INTERFACE_LIB_NAME})
INCLUDE_DIRECTORIES(lib/imu/src/)

# ============= Kalman Filter Library =================
SET(FILTER_LIB_NAME filter-module)
SET(FILTER_LIB_HEADERS lib/filter/src/integrator.hpp)
SET(FILTER_LIB_SOURCES lib/filter/src/integrator.cpp)
add_library(${FILTER_LIB_NAME} STATIC ${FILTER_LIB_HEADERS} ${FILTER_LIB_SOURCES})

# TARGET_LINK_LIBRARIES(${IMU_INTEGRATOR_LIB_NAME} PRIVATE ${LIBRARY_LINKAGE})

list(APPEND LIBRARY_LINKAGE ${IMU_INTEGRATOR_LIB_NAME})
INCLUDE_DIRECTORIES(lib/filter/src/)

# ============= Text UI Module =================
# this is as simple text program to run the above libraries
SET(CLI_EXE_NAME tracker)

# SET(EXE_HEADERS tools/cli/src/???
#     )

SET(EXE_SOURCES tools/logger/src/main.cpp )


ADD_EXECUTABLE(${CLI_EXE_NAME} ${EXE_HEADERS} ${EXE_SOURCES})

TARGET_INCLUDE_DIRECTORIES( ${CLI_EXE_NAME} PUBLIC lib/filter/src/ )
TARGET_LINK_LIBRARIES(${CLI_EXE_NAME} PRIVATE 
                            # filter-module
                            imu-interface-module )


# ============= Build Project Tests  =================
# https://cmake.org/cmake/help/v3.0/module/FindGTest.html 
# SET(TEST_EXE testall)
# SET(TEST_SOURCES    src/???
#     )

# SET(TEST_LINKAGE ${CHART_GRID_LIB_NAME} ${LIBRARY_LINKAGE} ${CONAN_LIBS} gtest gtest_main pthread)

# MESSAGE( STATUS "Generating all tests ...")
# MESSAGE( STATUS "    with name: ${TEST_EXE}")
# MESSAGE( STATUS "    with sources: ${TEST_SOURCES}")
# MESSAGE( STATUS "    with linkage: ${TEST_LINKAGE}")

# ADD_EXECUTABLE( ${TEST_EXE} ${TEST_SOURCES}) 
# ADD_TEST(AllTestsInFoo ${TEST_EXE}) 

# TARGET_LINK_LIBRARIES(${TEST_EXE} ${TEST_LINKAGE})
