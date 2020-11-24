# ============= IMU Interface Library =================
SET(FILTER_LIB_NAME filter-module)
SET(FILTER_LIB_HEADERS
            src/integrator.hpp)
SET(FILTER_LIB_SOURCES
            src/integrator.cpp)

add_library(${FILTER_LIB_NAME} STATIC ${FILTER_LIB_HEADERS} ${FILTER_LIB_SOURCES})

TARGET_LINK_LIBRARIES(${FILTER_LIB_NAME} PRIVATE ${IMU_INTERFACE_LIB_NAME})
