# ============= Text UI Module =================
# this is as simple program to pull single-shot data from the imu:

SET(PING_EXE_NAME imuping)

SET(EXE_HEADERS
        )

SET(EXE_SOURCES 
        src/main.cpp )

ADD_EXECUTABLE( ${PING_EXE_NAME} ${EXE_HEADERS} ${EXE_SOURCES})

# TARGET_COMPILE_OPTIONS( ${PING_EXE_NAME} -DREMAP_AXES)

TARGET_INCLUDE_DIRECTORIES( ${PING_EXE_NAME} PRIVATE
                        ${CMAKE_SOURCE_DIR}/lib/imu/src/ )

TARGET_LINK_LIBRARIES( ${PING_EXE_NAME} PRIVATE
                        ${LIBRARY_LINKAGE}
                        imu-driver-module )

