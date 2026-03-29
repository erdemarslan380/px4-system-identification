px4_add_module(
    MODULE modules_trajectory_reader
    MAIN trajectory_reader
    SRCS
        trajectory_reader.cpp
    MODULE_CONFIG
        module.yaml
    DEPENDS
        px4_work_queue
)

if(BUILD_TESTING)
    px4_add_unit_gtest(
        SRC
            trajectory_reader_offboard_transition_test.cpp
    )
endif()
