px4_add_module(
    MODULE modules_custom_pos_control
    MAIN custom_pos_control
    SRCS
        custom_pos_control.cpp
    DEPENDS
        px4_work_queue
)
