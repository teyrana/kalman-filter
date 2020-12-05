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
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-rtti -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic ")

# set(CMAKE_CXX_CLANG_TIDY "clang-tidy;-checks=*")
# # Clang-Tidy doesn't check headers by default.  Enable that:
# set(CMAKE_CXX_CLANG_TIDY clang-tidy;-header-filter=.;-checks=*;)

set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} -Og -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -DNDEBUG")

ADD_COMPILE_DEFINITIONS(_GLIBCXX_USE_CXX11_ABI=0)

# Debug only:
MESSAGE( STATUS ">> Configuring Build Type: ${CMAKE_BUILD_TYPE}")
# IF(CMAKE_BUILD_TYPE STREQUAL "Debug")
#     MESSAGE( STATUS ".... Configuring Debug Mode.")
# ELSEIF(CMAKE_BUILD_TYPE STREQUAL "Release")
#     MESSAGE( STATUS ".... Configuring Release Mode.")
# ENDIF(CMAKE_BUILD_TYPE STREQUAL "Debug")


# ============= Enable Testing  =================
#ExternalProject_Add(GTest
#    TMP_DIR ${PROJECT_SOURCE_DIR}/vendor/googletest 
#    GIT_REPOSITORY https://github.com/google/googletest.git) 
#
# INCLUDE_DIRECTORIES(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR}) 

enable_testing()

INCLUDE_DIRECTORIES(${CONAN_INCLUDE_DIRS})
LINK_DIRECTORIES(${CONAN_LIB_DIRS})

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
add_subdirectory( lib/imu )

# ============= Kalman Filter Library =================
# add_subdirectory( lib/filter ${CMAKE_SOURCE_DIR}/lib/ )

# ============= Text UI Tools =================
# this is as simple program to pull single commands from the imu:
add_subdirectory( tools/ping )

# streams data from the IMU:
add_subdirectory( tools/stream )

# streams & filters data from the IMU:
add_subdirectory( tools/tracker )

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
