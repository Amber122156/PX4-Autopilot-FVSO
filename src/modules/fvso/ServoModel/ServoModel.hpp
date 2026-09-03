#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>

#include <uORB/topics/servo_angle_setpoint.h>
#include <uORB/topics/servo_angle_state.h>

class Servo_model :
	public ModuleBase<Servo_model>,
	public ModuleParams,
	public px4::ScheduledWorkItem
{
private:
	uORB::Subscription _servo_angle_setpoint_sub{ORB_ID(servo_angle_setpoint)};

	uORB::Publication<servo_angle_state_s>_servo_angle_state_pub{ORB_ID(servo_angle_state)};
public:
	Servo_model();
	~Servo_model() override;
};
