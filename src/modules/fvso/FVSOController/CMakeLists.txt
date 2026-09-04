px4_add_module(
	MODULE modules
	MAIN servo_model

	SRCS
		FVSOController.cpp
		FVSOController.hpp

	DEPENDS
		px4_work_queue
)
