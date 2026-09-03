#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/servo_angle_setpoint.h>
#include <uORB/topics/servo_angle_state.h>

#include "../Params/ServoModelParams.hpp"

class ServoModel :
	public ModuleBase<ServoModel>,
	public ModuleParams,
	public px4::ScheduledWorkItem
{
private:
	void Run() override;

	/* ----------------------------- uROB Interface ----------------------------- */
	uORB::SubscriptionCallbackWorkItem _servo_angle_setpoint_sub{this,ORB_ID(servo_angle_setpoint)};
	uORB::Publication<servo_angle_state_s>_servo_angle_state_pub{ORB_ID(servo_angle_state)};

	/* ---------------------------------- Input --------------------------------- */
	float _left_angle_setpoint{0.0f};
	float _right_angle_setpoint{0.0f};

	bool _have_setpoint{false};

	/* --------------------------------- Output --------------------------------- */
	float _left_angle_state{0.0f};
	float _right_angle_state{0.0f};

	/* ---------------------------------- Time ---------------------------------- */
	hrt_abstime _last_update_time{0};
	static constexpr uint32_t SERVO_MODEL_INTERVAL_US{1000};

	/* ------------------------------- Convergence ------------------------------ */
	/*Numerical convergence threshold.Unit: rad */
	static constexpr float SERVO_CONVERGENCE_EPS{1e-4f};

public:
	ServoModel();
	~ServoModel() override;
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;
};
