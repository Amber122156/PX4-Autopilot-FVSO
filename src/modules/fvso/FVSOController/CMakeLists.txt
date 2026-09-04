px4_add_module(
    MODULE modules__fvso__FVSOController
    MAIN fvso_controller
    SRCS
        FVSOController.cpp
	FVSOController.hpp
    DEPENDS
        px4_work_queue
)
