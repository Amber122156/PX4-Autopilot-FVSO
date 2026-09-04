#include "FVSOController.hpp"


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


void FVSOController::Run()
{
	if (should_exit()) {
		_longitudinal_state_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}
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
