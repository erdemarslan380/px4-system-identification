px4_add_module(
    MODULE modules_trajectory_reader
    MAIN trajectory_reader
    SRCS
        trajectory_reader.cpp
    DEPENDS
        px4_work_queue
)
