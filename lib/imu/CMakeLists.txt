# ============= IMU Interface Library =================
SET(IMU_INTERFACE_LIB_NAME imu-interface-module)

SET(IMU_INTERFACE_LIB_HEADERS 
            src/command.hpp
            src/connection.hpp )

SET(IMU_INTERFACE_LIB_SOURCES
            src/connection.cpp )

add_library(${IMU_INTERFACE_LIB_NAME} STATIC ${IMU_INTERFACE_LIB_HEADERS} ${IMU_INTERFACE_LIB_SOURCES})

TARGET_INCLUDE_DIRECTORIES( ${IMU_INTERFACE_LIB_NAME} PUBLIC src/ )
