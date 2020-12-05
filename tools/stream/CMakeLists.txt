# ============= Headless Data Streamer Library =================
SET(CLI_EXE_NAME imustream)

SET(EXE_HEADERS
        )

SET(EXE_SOURCES 
        src/main.cpp )

ADD_EXECUTABLE(${CLI_EXE_NAME} ${EXE_HEADERS} ${EXE_SOURCES})

TARGET_INCLUDE_DIRECTORIES( ${CLI_EXE_NAME} PRIVATE
                            ${CMAKE_SOURCE_DIR}/lib/imu/src/ )

TARGET_LINK_LIBRARIES(${CLI_EXE_NAME} PRIVATE
                            ${LIBRARY_LINKAGE}
                            imu-driver-module )
