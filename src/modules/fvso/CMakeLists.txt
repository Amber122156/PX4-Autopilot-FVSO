px4_add_module(
    MODULE modules__fvso
    MAIN fvso
    SRCS
        Fvso.cpp
        Fvso.hpp
        ServoModel/ServoModel.cpp
    DEPENDS
        px4_work_queue
)
