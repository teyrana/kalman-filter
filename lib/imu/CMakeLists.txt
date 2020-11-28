# ============= IMU Interface Library =================
SET(IMU_DRIVER_LIB_NAME imu-driver-module)

SET(IMU_DRIVER_LIB_HEADERS
            ../serial/src/connection.hpp
            ../serial/src/connection.cpp
            src/command.hpp
            src/response.hpp
            src/driver.hpp
            src/driver.inl )

SET(IMU_DRIVER_LIB_SOURCES
            src/driver.cpp )

add_library(${IMU_DRIVER_LIB_NAME} STATIC ${IMU_DRIVER_LIB_HEADERS} ${IMU_DRIVER_LIB_SOURCES})

TARGET_INCLUDE_DIRECTORIES(${IMU_DRIVER_LIB_NAME} PUBLIC src/ )

TARGET_LINK_LIBRARIES(${IMU_DRIVER_LIB_NAME} PRIVATE ${LIBRARY_LINKAGE} )

target_compile_options(${IMU_DRIVER_LIB_NAME} PRIVATE -DREMAP_AXES )

