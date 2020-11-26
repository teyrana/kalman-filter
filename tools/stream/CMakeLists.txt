# ============= Headless Data Streamer Library =================
SET(CLI_EXE_NAME streamer)

SET(EXE_HEADERS 
        # src/???
        )

SET(EXE_SOURCES 
        src/main.cpp )

ADD_EXECUTABLE(${CLI_EXE_NAME} ${EXE_HEADERS} ${EXE_SOURCES})

TARGET_INCLUDE_DIRECTORIES( ${CLI_EXE_NAME} PRIVATE
                            ${CMAKE_SOURCE_DIR}/lib/imu/src/ )

TARGET_LINK_LIBRARIES(${CLI_EXE_NAME} PRIVATE
                            imu-driver-module )
