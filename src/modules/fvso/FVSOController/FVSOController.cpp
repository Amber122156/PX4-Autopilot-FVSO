#include <cmath>

#include <drivers/drv_hrt.h>

#include "FVSOController.hpp"
#include <modules/fvso/Params/FVSOControllerParams.hpp>


FVSOController::FVSOController() :
	ModuleParams(nullptr),
	ScheduledWorkItem("FVSOController", px4::wq_configurations::test1)
{
}

FVSOController::~FVSOController()
{
}

bool FVSOController::init()
{
	if (!_longitudinal_state_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

int FVSOController::task_spawn(int argc, char *argv[])
{
	FVSOController *instance = new FVSOController();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int FVSOController::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FVSOController::print_usage(const char *reason)
{
	return 0;
}

int FVSOController::print_status()
{
	PX4_INFO("Running");
	return PX4_OK;
}

extern "C" __EXPORT int fvso_controller_main(int argc, char *argv[])
{
	return FVSOController::main(argc, argv);
}

/* -------------------------------- Main Loop ------------------------------- */
void FVSOController::Run()
{
	if (should_exit()) {
		_longitudinal_state_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	/* --------------------- Get Current Longitudinal State --------------------- */
	longitudinal_state_s state{};

	if (!_longitudinal_state_sub.update(&state)) {
		return;
	}

	if (!std::isfinite(state.u) || !std::isfinite(state.w) || !std::isfinite(state.q) || !std::isfinite(state.theta)) {
		return;
	}

	/* x_l(k) = [u, w, q, theta]^T */
	x_l(0) = state.u;
	x_l(1) = state.w;
	x_l(2) = state.q;
	x_l(3) = state.theta;

	/* ----------------------- Update Reference State x_0 ----------------------- */
	longitudinal_state_setpoint_s setpoint{};

	if (_longitudinal_state_setpoint_sub.update(&setpoint)) {

		if (!std::isfinite(setpoint.u) || !std::isfinite(setpoint.w) || !std::isfinite(setpoint.q) || !std::isfinite(setpoint.theta)) {
			return;
		}

		x_0(0) = setpoint.u;
		x_0(1) = setpoint.w;
		x_0(2) = setpoint.q;
		x_0(3) = setpoint.theta;

		_setpoint_received = true;
	}

	if (!_setpoint_received) {
		return;
	}

	/* ------------------------ Controller Initialization ----------------------- */
	if (!_initialized){
		Initialization();
	}

	/* -------------------------- Controller Algorithm -------------------------- */
	_equ_pitch_input = Control();

	if (!std::isfinite(_equ_pitch_input)) {
		return;
	}

	/* -------------------------- Publish Servo Command ------------------------- */
	servo_angle_setpoint_s servo_setpoint{};

	servo_setpoint.timestamp = hrt_absolute_time();

	servo_setpoint.timestamp_sample = state.timestamp_sample;

	servo_setpoint.left_angle = _equ_pitch_input;
	servo_setpoint.right_angle = _equ_pitch_input;

	_servo_angle_setpoint_pub.publish(servo_setpoint);
}

void FVSOController::Initialization()
{
	x_in.setZero();

	_equ_pitch_input = 0.0f;

	_initialized = true;
}

float FVSOController::Control()
{
	/* Eq.(68): u(k) = -K_c * x_l(k) + K_in * x_in(k) */
	float equ_pitch_input = 0.0f;

	for (int i = 0; i < 4; i++) {
		equ_pitch_input -= fvso_controller_params::K_c[i] * x_l(i);
		equ_pitch_input += fvso_controller_params::K_in[i] * x_in(i);
	}

	/* Eq.(67): x_in(k+1) = x_in(k) + x_0(k) - x_l(k) */
	x_in += x_0 - x_l;

	return equ_pitch_input;
}
