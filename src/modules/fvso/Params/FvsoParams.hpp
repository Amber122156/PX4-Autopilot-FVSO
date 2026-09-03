#pragma once

namespace fvso_params
{
	/* --------------------- Offline identified model params -------------------- */
	/* These params describe the longitudinal gynamic model */
	namespace offline_model
	{
		static constexpr float A_l[4][4] = {
			/**/
		};

		static constexpr float B_e[4] = {
			/**/
		};

		static constexpr float Gamma_l[4][4] = {
			/**/
		};
	}

	/* --------------------------- Tunable FVSO params -------------------------- */
	/* These params depend on actual sensor noise,flapping vibration characteristics and observer performance */
	namespace tunable
	{
		static constexpr float P0[12][12] = {
			/**/
		};

		static constexpr float Q_l_diag[4] = {
			/**/
		};

		static constexpr float Q_n_diag[4] = {
			/**/
		};

		static constexpr float R_l_diag[4] = {
			/**/
		};

		/* ---------------------------- GFV_Model params ---------------------------- */
		namespace gfv_u
		{
		static constexpr float a       = 0.0f;
		static constexpr float b       = 0.0f;
		static constexpr float epsilon = 0.0f;
		static constexpr float omega   = 0.0f;
		}

		namespace gfv_w
		{
		static constexpr float a       = 0.0f;
		static constexpr float b       = 0.0f;
		static constexpr float epsilon = 0.0f;
		static constexpr float omega   = 0.0f;
		}

		namespace gfv_q
		{
		static constexpr float a       = 0.0f;
		static constexpr float b       = 0.0f;
		static constexpr float epsilon = 0.0f;
		static constexpr float omega   = 0.0f;
		}

		namespace gfv_theta
		{
		static constexpr float a       = 0.0f;
		static constexpr float b       = 0.0f;
		static constexpr float epsilon = 0.0f;
		static constexpr float omega   = 0.0f;
		}
	}
}
