############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
############################################################################

project(SystemIdentificationLoggerPlugin)

add_library(${PROJECT_NAME} SHARED
    SystemIdentificationLogger.cpp
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC px4_gz_msgs
    PUBLIC ${GZ_SENSORS_TARGET}
    PUBLIC ${GZ_PLUGIN_TARGET}
    PUBLIC ${GZ_SIM_TARGET}
    PUBLIC ${GZ_TRANSPORT_TARGET}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
    PUBLIC px4_gz_msgs
)
