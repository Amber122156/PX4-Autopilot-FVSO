#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <matrix/matrix/math.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/longitudinal_state.h>
#include <uORB/topics/longitudinal_state_setpoint.h>
#include <uORB/topics/servo_angle_setpoint.h>


class FVSOController :
	public ModuleBase<FVSOController>,
	public ModuleParams,
	public px4::ScheduledWorkItem
{
private:

	void Run() override;

	/* -------------------------- Controller Algorithm ------------------------- */
	void Initialization();
	float Control();

	/* ----------------------------- Runtime States ---------------------------- */
	/* Current longitudinal state:
	x_l = [u, w, q, theta]^T */
	matrix::Vector<float, 4> x_l{};

	/* Integral state in Eq.(67) */
	matrix::Vector<float, 4> x_in{};

	/* Desired longitudinal state:x_0 = [u_ref, w_ref, q_ref, theta_ref]^T */
	matrix::Vector<float, 4> x_0{};

	/* Equivalent pitch control input u(k) */
	float _equ_pitch_input{0.0f};

	bool _initialized{false};

	/* ------------------------------ uORB Input ------------------------------- */
	/* Controller is triggered whenever a new longitudinal state is published */
	uORB::SubscriptionCallbackWorkItem _longitudinal_state_sub{this,ORB_ID(longitudinal_state)};
	/* Latest reference state.Updating this topic does not trigger the controller. */
	uORB::Subscription _longitudinal_state_setpoint_sub{ORB_ID(longitudinal_state_setpoint)};

	bool _setpoint_received{false};
	/* ------------------------------ uORB Output ------------------------------ */
	uORB::Publication<servo_angle_setpoint_s> _servo_angle_setpoint_pub{ORB_ID(servo_angle_setpoint)};

public:

	FVSOController();
	~FVSOController() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;
};
