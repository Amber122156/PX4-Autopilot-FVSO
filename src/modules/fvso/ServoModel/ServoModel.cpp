#include "ServoModel.hpp"

#include <cmath>

#include <drivers/drv_hrt.h>

#include <modules/fvso/Params/ServoModelParams.hpp>


ServoModel::ServoModel() :
	ModuleParams(nullptr),
	ScheduledWorkItem("ServoModel", px4::wq_configurations::test1)
{
}

ServoModel::~ServoModel()
{
	ScheduleClear();
}

bool ServoModel::init()
{
	/* ------------------------- Validate Model Params ------------------------- */

	if (!std::isfinite(servo_params::K_s) || !std::isfinite(servo_params::w_s) || servo_params::w_s <= 0.0f) {
		PX4_ERR("invalid servo model parameters");
		return false;
	}

	/* -------------------------- Initialize States ---------------------------- */

	_left_angle_setpoint = 0.0f;
	_right_angle_setpoint = 0.0f;

	_left_angle_state = 0.0f;
	_right_angle_state = 0.0f;

	_have_setpoint = false;

	/* ------------------------- Start Periodic Update ------------------------- */

	if (!_servo_angle_setpoint_sub.registerCallback()) {
		PX4_ERR("failed to register servo setpoint callback");
		return false;
	}
	return true;
}

int ServoModel::task_spawn(int argc, char *argv[])
{
	ServoModel *instance = new ServoModel();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;

	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int ServoModel::print_status()
{
	PX4_INFO("Running");

	PX4_INFO("left setpoint: %.4f rad", (double)_left_angle_setpoint);
	PX4_INFO("right setpoint: %.4f rad", (double)_right_angle_setpoint);

	return PX4_OK;
}

int ServoModel::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ServoModel::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_USAGE_NAME("servo_model", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int servo_model_main(int argc, char *argv[])
{
	return ServoModel::main(argc, argv);
}

/* ------------------------------- Main Loop -------------------------------- */
void ServoModel::Run()
{
	/* --------------------------- Check Module Exit --------------------------- */
	if (should_exit()) {
		_servo_angle_setpoint_sub.unregisterCallback();
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	/* Current execution time */
	const hrt_abstime now = hrt_absolute_time();

	if (_have_setpoint && (_last_update_time != 0)) {

		const float dt = static_cast<float>(now - _last_update_time) * 1e-6f;

		if (dt > 0.0f) {

			const float a = expf(-servo_params::w_s * dt);
			const float b = (servo_params::K_s / servo_params::w_s) * (1.0f - a);

			_left_angle_state = a * _left_angle_state + b * _left_angle_setpoint;
			_right_angle_state = a * _right_angle_state + b * _right_angle_setpoint;
		}
	}

	_last_update_time = now;

	servo_angle_setpoint_s setpoint{};

	if (_servo_angle_setpoint_sub.update(&setpoint)) {

		if (PX4_ISFINITE(setpoint.left_angle) && PX4_ISFINITE(setpoint.right_angle)) {

			_left_angle_setpoint = setpoint.left_angle;
			_right_angle_setpoint = setpoint.right_angle;

			_have_setpoint = true;
		}
	}

	if (!_have_setpoint) {
		return;
	}

	const float left_steady_state = (servo_params::K_s / servo_params::w_s) * _left_angle_setpoint;
	const float right_steady_state = (servo_params::K_s / servo_params::w_s) * _right_angle_setpoint;

	const bool left_converged = fabsf(_left_angle_state - left_steady_state) < SERVO_CONVERGENCE_EPS;
	const bool right_converged = fabsf(_right_angle_state - right_steady_state) < SERVO_CONVERGENCE_EPS;

	servo_angle_state_s servo_state{};

	servo_state.timestamp = now;
	servo_state.timestamp_sample = now;

	if (left_converged && right_converged) {

		_left_angle_state = left_steady_state;
		_right_angle_state = right_steady_state;

		servo_state.left_angle = _left_angle_state;
		servo_state.right_angle = _right_angle_state;

		_servo_angle_state_pub.publish(servo_state);

		return;
	}

	servo_state.left_angle = _left_angle_state;
	servo_state.right_angle = _right_angle_state;

	_servo_angle_state_pub.publish(servo_state);

	ScheduleDelayed(SERVO_MODEL_INTERVAL_US);
}
