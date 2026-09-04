#include <cmath>

#include <drivers/drv_hrt.h>>

#include <modules/fvso/Fvso.hpp>
#include <modules/fvso/Params/FvsoParams.hpp>

Fvso::Fvso() :
	ModuleParams(nullptr),
	ScheduledWorkItem("Fvso", px4::wq_configurations::test1)
{
}

Fvso::~Fvso()
{
}

bool Fvso::init()
{
	/* Load original FVSO parameters */
	InitializeParams();

	/* Check all original parameters before building matrices */
	if (!ValidateParams()) {
		PX4_ERR("FVSO parameter validation failed");
		return false;
	}

	/* Build model matrices */
	BuildFlappingVibrationModel();
	BuildExtendedModel();
	BuildDiscreteFormMatrices();

	FVSO_InnerVariables.Init = false;


	/* Run FVSO periodically */
	ScheduleOnInterval(
		static_cast<uint32_t>(FVSO_SAMPLE_TIME * 1e6f)
	);

	return true;
}

int Fvso::task_spawn(int argc, char *argv[])
{
	Fvso *instance = new Fvso();

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

int Fvso::print_status()
{
	PX4_INFO("Running");
	PX4_INFO("Observer initialized: %s", FVSO_InnerVariables.Init ? "yes" : "no");

	return PX4_OK;
}

int Fvso::custom_command(int argc, char *argv[])
{
	/* Apart from the default command, other commands are not supported */
	return print_usage("unknown command");
}

int Fvso::print_usage(const char *reason)
{
	return 0;
}

extern "C" __EXPORT int fvso_main(int argc, char *argv[])
{
    return Fvso::main(argc, argv);
}

/* -------------------------------- Main Loop ------------------------------- */
void Fvso::Run()
{
	/* Check whether module should exit */
	if (should_exit()){
		ScheduleClear();
		exit_and_cleanup();
		return;
	}
	/* ----------------------------- Get FVSO Inputs ---------------------------- */
	/* z(k) = [u, w, q, theta]^T */
	MeasurementStates z{};

	/* delta_e(k): equivalent pitch control input */
	float equ_pitch_input{0.0f};

	/* Obtain current measurement z */
	if (!UpdateSensorDatas(z)) {
		/* Required sensor data is not ready in this cycle */
		return;
	}

	/* Obtain current equivalent pitch input */
	if (!UpdateEquPitchInput(equ_pitch_input)) {
		/* Required control input is not ready in this cycle */
		return;
	}

	/* ----------------------------- FVSO Initialize ---------------------------- */
	/* Initialize FVSO if this is the first valid cycle */
	if (!FVSO_InnerVariables.Init) {

		Initialization(z);

		/* Publish the initialized posterior state x_est(0|0) */
		PublishState();

		/* Prepare the predicted state for the next cycle */
		Prediction(equ_pitch_input);

		return;
	}

	/* ---------------------------- FVSO Normal Cycle --------------------------- */
	/* At the beginning of cycle k:
	   x_pred(k|k-1) and p_pred(k|k-1) have already been calculated by Prediction() in cycle k-1 */
	/* Correct the predicted state using the current measurement z(k) */
	if (!Estimation(z)) {
		/* The current estimation is invalid.Restart FVSO initialization when valid data is available again */
		FVSO_InnerVariables.Init = false;
		return;
	}

	/* Publish the posterior estimated state x_est(k|k) to the controller */
	PublishState();

	/* Predict x_pred(k+1|k) and p_pred(k+1|k) for the next cycle */
	Prediction(equ_pitch_input);
}

/* ----------------------------- Fvso Algorithm ----------------------------- */
void Fvso::Initialization(const MeasurementStates &z)
{
	/* According to Eq.(55)
	   x_ex = [u, w, q, theta,
	           x_u1, x_u2,
	           x_w1, x_w2,
	           x_q1, x_q2,
	           x_theta1, x_theta2]^T */
	FVSO_InnerVariables.x_est.setZero();
	/* Use the sensor's initial value at the moment the FVSO algorithm starts as X_0 */
	FVSO_InnerVariables.x_est(0) = z.u;
	FVSO_InnerVariables.x_est(1) = z.w;
	FVSO_InnerVariables.x_est(2) = z.q;
	FVSO_InnerVariables.x_est(3) = z.theta;
	/* Initial estimation covariance: P_est(0) = P_0 */
	FVSO_InnerVariables.p_est = FVSO_InnerVariables.p_0;

	FVSO_InnerVariables.x_pred = FVSO_InnerVariables.x_est;
	FVSO_InnerVariables.p_pred = FVSO_InnerVariables.p_est;
	FVSO_InnerVariables.K.setZero();

	/* FVSO initialization has been completed */
	FVSO_InnerVariables.Init = true;
}

void Fvso::Prediction(float equ_pitch_input)
{
	/* According to Eq.(56) and Eq.(57)
	   x_pred = A_ex_s * x_est + B_ex_s * delta_e
	   p_pred = A_ex_s * p_est * A_ex_s^T + Q_ex_s */
	FVSO_InnerVariables.x_pred = FVSO_DiscreteFormMatrices.A_ex_s * FVSO_InnerVariables.x_est + FVSO_DiscreteFormMatrices.B_ex_s * equ_pitch_input;
	FVSO_InnerVariables.p_pred = FVSO_DiscreteFormMatrices.A_ex_s * FVSO_InnerVariables.p_est * FVSO_DiscreteFormMatrices.A_ex_s_T + FVSO_DiscreteFormMatrices.Q_ex_s;
}

bool Fvso::Estimation(const MeasurementStates &z)
{
	/* According to Eq.(58),Eq.(59) and Eq.(60) */
	matrix::Vector<float, 4> z_vec;
	z_vec(0) = z.u;
	z_vec(1) = z.w;
	z_vec(2) = z.q;
	z_vec(3) = z.theta;

	/* K = p_pred * C_ex_s^T  * (C_ex_s * p_pred * C_ex_s^T + R_l_s)^(-1) */
	_CP = FVSO_DiscreteFormMatrices.C_ex_s * FVSO_InnerVariables.p_pred;
	_innovation_cov = _CP * FVSO_DiscreteFormMatrices.C_ex_s_T;
	_innovation_cov += FVSO_DiscreteFormMatrices.R_l_s;

	if (!matrix::inv(_innovation_cov, _innovation_cov_inv)) {
		PX4_ERR("FVSO innovation covariance inversion failed");
		return false;
	}

	_PCt = FVSO_InnerVariables.p_pred * FVSO_DiscreteFormMatrices.C_ex_s_T;
	FVSO_InnerVariables.K = _PCt * _innovation_cov_inv;

	/* innovation = z- C_ex_s * x_pred
	   X_est = x_pred + k * innovation */
	_innovation =  z_vec - FVSO_DiscreteFormMatrices.C_ex_s * FVSO_InnerVariables.x_pred;

	FVSO_InnerVariables.x_est = FVSO_InnerVariables.x_pred + FVSO_InnerVariables.K * _innovation;

	/* p_est = (I - k * C_ex_s ) * p_pred */
	for (int i = 0; i < 12; i++) {

		for (int j = 0; j < 12; j++) {

			float correction = 0.0f;

			for (int k = 0; k < 4; k++) {
				correction +=
				FVSO_InnerVariables.K(i, k) * _CP(k, j);
			}

			FVSO_InnerVariables.p_est(i, j) = FVSO_InnerVariables.p_pred(i, j) - correction;
		}
	}

	return true;
}

/* --------------------------- PX4 Data Interface --------------------------- */
bool Fvso::UpdateSensorDatas(MeasurementStates &z)
{
	/* -------------------------- Update PX4 Topic Data ------------------------- */
	if (_vehicle_local_position_sub.update(&_vehicle_local_position)) {
		_local_position_updated = true;
	}

	if (_vehicle_angular_velocity_sub.update(&_vehicle_angular_velocity)) {
		_angular_velocity_updated = true;
	}

	if (_vehicle_attitude_sub.update(&_vehicle_attitude)) {
		_attitude_updated = true;
	}

	/* ------------------------- Check Data Completeness ------------------------- */
	/* A complete z(k) can only be constructed after all required topics
	   have been updated at least once since the last complete z(k-1) was used */

	if (!_local_position_updated || !_angular_velocity_updated || !_attitude_updated) {
		return false;
	}

	/* --------------------------- Check Data Validity -------------------------- */
	if (!_vehicle_local_position.v_xy_valid || !_vehicle_local_position.v_z_valid) {
		_local_position_updated = false;
		return false;
	}

	if (!std::isfinite(_vehicle_local_position.vx) || !std::isfinite(_vehicle_local_position.vy) || !std::isfinite(_vehicle_local_position.vz)) {
		_local_position_updated = false;
		return false;
	}

	/* q comes from the processed body angular velocity */
	if (!std::isfinite(_vehicle_angular_velocity.xyz[1])) {
		_angular_velocity_updated = false;
		return false;
	}

	/* Check the attitude quaternion */
	for (int i = 0; i < 4; i++) {

		if (!std::isfinite(_vehicle_attitude.q[i])) {
			_attitude_updated = false;
			return false;
		}
	}

	/* ------------------------- Construct Measurement z ------------------------ */
	const matrix::Vector3f velocity_ned(
		_vehicle_local_position.vx,
		_vehicle_local_position.vy,
		_vehicle_local_position.vz
	);

	/* vehicle_attitude.q describes the rotation from body frame to NED frame */
	const matrix::Quatf attitude_q(_vehicle_attitude.q);
	const matrix::Dcmf R_body_to_ned(attitude_q);

	/* Transform NED velocity into body-frame velocity:
	   velocity_body = R_body_to_ned^T * velocity_ned
	   velocity_body = [u, v, w]^T */

	const matrix::Vector3f velocity_body = R_body_to_ned.transpose() * velocity_ned;

	/* u: body x-axis velocity
	   w: body z-axis velocity */
	z.u = velocity_body(0);
	z.w = velocity_body(2);

	/* q: body pitch angular velocity */
	z.q = _vehicle_angular_velocity.xyz[1];

	/* theta: pitch angle obtained from attitude quaternion */
	const matrix::Eulerf euler(attitude_q);
	z.theta = euler.theta();

	/* --------------------------- Check Final states --------------------------- */
	if (!std::isfinite(z.u) || !std::isfinite(z.w) || !std::isfinite(z.q) || !std::isfinite(z.theta)) {

		_local_position_updated = false;
		_angular_velocity_updated = false;
		_attitude_updated = false;

		return false;
	}

	/* ----------------------- Complete Measurement Used ----------------------- */
	/* The current complete z(k) has been constructed.
	   Each required topic must be updated again before constructing z(k+1). */
	_local_position_updated = false;
	_angular_velocity_updated = false;
	_attitude_updated = false;


	return true;
}

bool Fvso::UpdateEquPitchInput(float &equ_pitch_input)
{
	/* Obtain the latest real left/right servo angles */
	if (!_servo_angle_real_state_sub.update(&_servo_angle_real_state)) {
		return false;
	}

	const float left_angle = _servo_angle_real_state.left_angle;
	const float right_angle = _servo_angle_real_state.right_angle;

	/* Check whether both servo angles are valid */
	if (!std::isfinite(left_angle) || !std::isfinite(right_angle)) {
		return false;
	}

	equ_pitch_input = 0.5f * (left_angle + right_angle);

	if (!std::isfinite(equ_pitch_input)) {
		return false;
	}

	return true;
}

void Fvso::PublishState()
{
	fvso_state_s msg{};

	/* Publication timestamp */
	msg.timestamp = hrt_absolute_time();

	msg.timestamp_sample = msg.timestamp;

	/* x_est = [x_l, x_n]^T
	   x_l = [u, w, q, theta]^T */
	msg.u = FVSO_InnerVariables.x_est(0);
	msg.w = FVSO_InnerVariables.x_est(1);
	msg.q = FVSO_InnerVariables.x_est(2);
	msg.theta = FVSO_InnerVariables.x_est(3);

	_fvso_state_pub.publish(msg);
}

/* ---------------------------- Model Preparation --------------------------- */
void Fvso::BuildFlappingVibrationModel()
{
	/* Build A_n,B_n,C_n by GFV models from four channels,for each channel:
		A_i = [      0              1        ]
	       	      [ -omega^2   -2*epsilon*omega  ]
		B_i = [0]
		      [1]
		C_i = [b a]
	*/

	/* GFV parameters of the four output channels */
	const GFVChannelParams *gfv_channels[4] = {
		&FVSO_FlappingModel.GFV_u,
		&FVSO_FlappingModel.GFV_w,
		&FVSO_FlappingModel.GFV_q,
		&FVSO_FlappingModel.GFV_theta
	};

	/* Considering the number of zeros in the matrix,it's best to just zero out the whole matrix from the start */
	FVSO_ContinuousMatrices.A_n.setZero();
	FVSO_ContinuousMatrices.B_n.setZero();
	FVSO_ContinuousMatrices.C_n.setZero();

	/* An is an 8 x 8 state space matrix */
	/* Bn is an 8 x 4 input matrix */
	/* Cn is a 4 x 8 state observation matrix */
	/* ------------------ Build A_n,B_n,C_n channel by channel ------------------ */
	for(int channel = 0; channel < 4; channel++){

		const GFVChannelParams &gfv = *gfv_channels[channel];
		/*
		 * Each GFV channel occupies two states:
		 * channel = 0 -> state 0, 1 -> u
		 * channel = 1 -> state 2, 3 -> w
		 * channel = 2 -> state 4, 5 -> q
		 * channel = 3 -> state 6, 7 -> theta
		 */
		const int state_index = 2 * channel;

		/* ------------------------- Build A_n ------------------------- */

		FVSO_ContinuousMatrices.A_n(state_index, state_index + 1) = 1.0f;
		FVSO_ContinuousMatrices.A_n(state_index + 1, state_index) = -gfv.omega * gfv.omega;
		FVSO_ContinuousMatrices.A_n(state_index + 1, state_index + 1) = -2.0f * gfv.epsilon * gfv.omega;

		/* ------------------------- Build B_n ------------------------- */

		FVSO_ContinuousMatrices.B_n(state_index + 1, channel) = 1.0f;

		/* ------------------------- Build C_n ------------------------- */

		FVSO_ContinuousMatrices.C_n(channel, state_index) = gfv.b;
		FVSO_ContinuousMatrices.C_n(channel, state_index + 1) = gfv.a;
	}
}

void Fvso::BuildExtendedModel()
{
	/* Considering the number of zeros in the matrix,it's best to just zero out the whole matrix from the start */
	FVSO_ContinuousMatrices.A_ex.setZero();
	FVSO_ContinuousMatrices.B_ex.setZero();
	FVSO_ContinuousMatrices.C_ex.setZero();
	FVSO_ContinuousMatrices.Gamma_ex.setZero();
	FVSO_ContinuousMatrices.Q_ex.setZero();

	/* -------------------------------- Build A_ex ------------------------------- */
	/* A_ex = [A_l   0]
		  [0   A_n]*/
	for (int i = 0; i < 4; i++) {

		for (int j = 0; j < 4; j++) {

			FVSO_ContinuousMatrices.A_ex(i, j) = FVSO_DynamicModel.A_l(i, j);
		}
	}

	for (int i = 0; i < 8; i++) {

		for (int j = 0; j < 8; j++) {

			FVSO_ContinuousMatrices.A_ex(i + 4, j + 4) = FVSO_ContinuousMatrices.A_n(i, j);
		}
	}

	/* ------------------------------- Build B_ex ------------------------------- */
	/* B_ex = [B_e]
		  [0]  */
	for (int i = 0; i < 4; i++) {

		FVSO_ContinuousMatrices.B_ex(i, 0) = FVSO_DynamicModel.B_e(i, 0);
	}

	/* ------------------------------- Build C_ex ------------------------------- */
	/* C_ex = [C_l  C_n] */
	for (int i = 0; i < 4; i++) {
		/* C_l = I_4 */
		FVSO_ContinuousMatrices.C_ex(i, i) = 1.0f;

		for (int j = 0; j < 8; j++) {

			FVSO_ContinuousMatrices.C_ex(i, j + 4) = FVSO_ContinuousMatrices.C_n(i, j);
		}
	}

	/* ----------------------------- Build Gamma_ex ----------------------------- */
	/* Gamma_ex = [Gamma_l   0]
		      [0       B_n]*/
	for (int i = 0; i < 4; i++) {

		for (int j = 0; j < 4; j++) {

			FVSO_ContinuousMatrices.Gamma_ex(i, j) = FVSO_DynamicModel.Gamma_l(i, j);
		}
	}

	for (int i = 0; i < 8; i++) {

		for (int j = 0; j < 4; j++) {

			FVSO_ContinuousMatrices.Gamma_ex(i + 4, j + 4) = FVSO_ContinuousMatrices.B_n(i, j);
		}
	}

	/* ----------------------------- Build Q_ex ----------------------------- */
	/* Q_ex = [Q_l    0]
		  [0    Q_n]*/
	for (int i = 0; i < 4; i++) {

		FVSO_ContinuousMatrices.Q_ex(i, i) = FVSO_DynamicModel.Q_l_diag(i);
		FVSO_ContinuousMatrices.Q_ex(i + 4, i + 4) = FVSO_FlappingModel.Q_n_diag(i);
	}
}

void Fvso::BuildDiscreteFormMatrices()
{
	/* Build A_ex_s, B_ex_s, C_ex_s, Q_ex_s, R_l_s*/
	/* ------------------------------ Build A_ex_s ------------------------------ */
	/* A_ex_s = I + A_ex * T */
	FVSO_DiscreteFormMatrices.A_ex_s.setZero();

	for (int i = 0; i < 12; i++) {

		for (int j = 0; j < 12; j++) {

			FVSO_DiscreteFormMatrices.A_ex_s(i, j) = FVSO_ContinuousMatrices.A_ex(i, j) * FVSO_SAMPLE_TIME;
		}

		FVSO_DiscreteFormMatrices.A_ex_s(i, i) += 1.0f;
	}

	FVSO_DiscreteFormMatrices.A_ex_s_T = FVSO_DiscreteFormMatrices.A_ex_s.transpose();

	/* ------------------------------ Build B_ex_s ------------------------------ */
	/* B_ex_s = B_ex * T */
	for (int i = 0; i < 12; i++) {

		FVSO_DiscreteFormMatrices.B_ex_s(i, 0) = FVSO_ContinuousMatrices.B_ex(i, 0) * FVSO_SAMPLE_TIME;
	}

	/* ------------------------------ BUild C_ex_s ------------------------------ */
	/* C_ex_s = C_ex */
	for (int i = 0; i < 4; i++) {

		for (int j = 0; j < 12; j++) {

			FVSO_DiscreteFormMatrices.C_ex_s(i, j) = FVSO_ContinuousMatrices.C_ex(i, j);
		}
	}

	FVSO_DiscreteFormMatrices.C_ex_s_T = FVSO_DiscreteFormMatrices.C_ex_s.transpose();

	/* ------------------------------ Build Q_ex_s ------------------------------ */
	/* Q_ex_s = Gamma_ex * Q_ex * Gamma_ex^T * T */
	matrix::Matrix<float, 12, 8> gamma_q;

	gamma_q = FVSO_ContinuousMatrices.Gamma_ex * FVSO_ContinuousMatrices.Q_ex;
	FVSO_DiscreteFormMatrices.Q_ex_s = gamma_q * FVSO_ContinuousMatrices.Gamma_ex.transpose() * FVSO_SAMPLE_TIME;

	/* ------------------------------- Build R_l_s ------------------------------ */
	/* R_l_s = R_l / T */
	FVSO_DiscreteFormMatrices.R_l_s.setZero();

	for (int i = 0; i < 4; i++) {

		FVSO_DiscreteFormMatrices.R_l_s(i, i) = FVSO_DynamicModel.R_l_diag(i) / FVSO_SAMPLE_TIME;
	}

}

/* --------------------------- Params Preparation --------------------------- */
void Fvso::InitializeParams()
{
	/* Load offline identified model params */
	for (int i = 0; i < 4; i++) {

		FVSO_DynamicModel.B_e(i, 0) = fvso_params::offline_model::B_e[i];

		for (int j = 0; j < 4; j++) {

			FVSO_DynamicModel.A_l(i, j) = fvso_params::offline_model::A_l[i][j];
			FVSO_DynamicModel.Gamma_l(i, j) = fvso_params::offline_model::Gamma_l[i][j];
		}
	}

	/* Load tunable FVSO params */
	for (int i = 0; i < 4; i++) {

		FVSO_DynamicModel.Q_l_diag(i) = fvso_params::tunable::Q_l_diag[i];
		FVSO_DynamicModel.R_l_diag(i) = fvso_params::tunable::R_l_diag[i];
		FVSO_FlappingModel.Q_n_diag(i) = fvso_params::tunable::Q_n_diag[i];
	}

	for (int i = 0; i < 12; i++) {

		for (int j = 0; j < 12; j++) {

			FVSO_InnerVariables.p_0(i, j) = fvso_params::tunable::P0[i][j];
		}
	}

	/* Load GFV params for four channels*/
	LoadGFVParams(
		FVSO_FlappingModel.GFV_u,
		fvso_params::tunable::gfv_u::a,
		fvso_params::tunable::gfv_u::b,
		fvso_params::tunable::gfv_u::epsilon,
		fvso_params::tunable::gfv_u::omega
	);

	LoadGFVParams(
		FVSO_FlappingModel.GFV_w,
		fvso_params::tunable::gfv_w::a,
		fvso_params::tunable::gfv_w::b,
		fvso_params::tunable::gfv_w::epsilon,
		fvso_params::tunable::gfv_w::omega
	);

	LoadGFVParams(
		FVSO_FlappingModel.GFV_q,
		fvso_params::tunable::gfv_q::a,
		fvso_params::tunable::gfv_q::b,
		fvso_params::tunable::gfv_q::epsilon,
		fvso_params::tunable::gfv_q::omega
	);

	LoadGFVParams(
		FVSO_FlappingModel.GFV_theta,
		fvso_params::tunable::gfv_theta::a,
		fvso_params::tunable::gfv_theta::b,
		fvso_params::tunable::gfv_theta::epsilon,
		fvso_params::tunable::gfv_theta::omega
	);
}

void Fvso::LoadGFVParams(GFVChannelParams &target,
			 float a,
			 float b,
			 float epsilon,
			 float omega)
{
	target.a = a;
	target.b = b;
	target.epsilon = epsilon;
	target.omega = omega;
}

bool Fvso::ValidateParams() const
{
	constexpr float symmetry_tolerance = 1e-5f;

	/* Check A_l */

	for (int i = 0; i < 4; i++) {

		for (int j = 0; j < 4; j++) {

			if (!std::isfinite(FVSO_DynamicModel.A_l(i, j))) {
				PX4_ERR("A_l contains invalid value at [%d][%d]", i, j);
				return false;
			}
		}
	}

	/* Check B_e */

	for (int i = 0; i < 4; i++) {

		if (!std::isfinite(FVSO_DynamicModel.B_e(i, 0))) {
			PX4_ERR("B_e contains invalid value at [%d]", i);
			return false;
		}
	}

	/* Check Gamma_l */

	for (int i = 0; i < 4; i++) {

		for (int j = 0; j < 4; j++) {

			if (!std::isfinite(FVSO_DynamicModel.Gamma_l(i, j))) {
				PX4_ERR("Gamma_l contains invalid value at [%d][%d]", i, j);
				return false;
			}
		}
	}

	/* Check Q_l,every element must be >= 0 */

	for (int i = 0; i < 4; i++) {

		const float value = FVSO_DynamicModel.Q_l_diag(i);

		if (!std::isfinite(value) || value < 0.0f) {
			PX4_ERR("invalid Q_l_diag[%d]", i);
			return false;
		}
	}

	/* Check Q_n */

	for (int i = 0; i < 4; i++) {

		const float value = FVSO_FlappingModel.Q_n_diag(i);

		if (!std::isfinite(value) || value < 0.0f) {
			PX4_ERR("invalid Q_n_diag[%d]", i);
			return false;
		}
	}

	/* Check R_l */

	for (int i = 0; i < 4; i++) {

		const float value = FVSO_DynamicModel.R_l_diag(i);

		if (!std::isfinite(value) || value <= 0.0f) {
			PX4_ERR("invalid R_l_diag[%d]", i);
			return false;
		}
	}

	/* Check P0,basic validity check of P0 */

	for (int i = 0; i < 12; i++) {

		for (int j = 0; j < 12; j++) {

			const float value = FVSO_InnerVariables.p_0(i, j);

			if (!std::isfinite(value)) {
				PX4_ERR("P0 contains invalid value at [%d][%d]", i, j);
				return false;
			}

			if (std::fabs(
				FVSO_InnerVariables.p_0(i, j) - FVSO_InnerVariables.p_0(j, i)) > symmetry_tolerance) {
				PX4_ERR("P0 is not symmetric at [%d][%d]", i, j);
				return false;
			}
		}

		if (FVSO_InnerVariables.p_0(i, i) < 0.0f) {
			PX4_ERR("P0 diagonal must be non-negative at [%d]", i);
			return false;
		}
	}


	/* Check GFV parameters */

	const GFVChannelParams *gfv_channels[4] = {
		&FVSO_FlappingModel.GFV_u,
		&FVSO_FlappingModel.GFV_w,
		&FVSO_FlappingModel.GFV_q,
		&FVSO_FlappingModel.GFV_theta
	};

	const char *channel_names[4] = {
		"u",
		"w",
		"q",
		"theta"
	};

	for (int i = 0; i < 4; i++) {

		const GFVChannelParams &gfv = *gfv_channels[i];

		if (!std::isfinite(gfv.a) || !std::isfinite(gfv.b) || !std::isfinite(gfv.epsilon) || !std::isfinite(gfv.omega)) {
			PX4_ERR("GFV_%s contains invalid value", channel_names[i]);
			return false;
		}

		if (gfv.epsilon <= 0.0f) {
			PX4_ERR("GFV_%s epsilon must be positive", channel_names[i]);
			return false;
		}

		if (gfv.omega <= 0.0f) {
			PX4_ERR("GFV_%s omega must be positive", channel_names[i]);
			return false;
		}
	}


	return true;
}
