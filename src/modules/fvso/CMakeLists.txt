px4_add_module(
    MODULE modules__fvso
    MAIN fvso
    SRCS
        Fvso.cpp
        Fvso.hpp
    DEPENDS
        px4_work_queue
)

add_subdirectory(ServoModel)
add_subdirectory(FVSOController)
