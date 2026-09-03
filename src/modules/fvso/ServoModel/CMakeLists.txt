px4_add_module(
	MODULE modules__servo_model
	MAIN servo_model

	SRCS
		ServoModel.cpp
		ServoModel.hpp

	DEPENDS
		px4_work_queue
)
