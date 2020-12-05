# ============= IMU Interface Library =================
SET(IMU_INTERFACE_LIB_NAME imu-driver-module)

SET(IMU_INTERFACE_LIB_HEADERS
            ../serial/src/connection.hpp
            ../serial/src/connection.cpp
            ../serial/src/message.hpp
            src/command.hpp
            src/driver.hpp )

SET(IMU_INTERFACE_LIB_SOURCES
            src/driver.cpp )

add_library(${IMU_INTERFACE_LIB_NAME} STATIC ${IMU_INTERFACE_LIB_HEADERS} ${IMU_INTERFACE_LIB_SOURCES})

TARGET_INCLUDE_DIRECTORIES( ${IMU_INTERFACE_LIB_NAME} PUBLIC src/ )

TARGET_LINK_LIBRARIES(${IMU_INTERFACE_LIB_NAME} PRIVATE ${LIBRARY_LINKAGE} )
