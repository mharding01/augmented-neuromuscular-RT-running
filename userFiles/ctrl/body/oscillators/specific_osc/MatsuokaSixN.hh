/*! \brief MatsuokaSixN class
 */

#ifndef _MATSUOKA_SIX_N_HH_
#define _MATSUOKA_SIX_N_HH_

#include "Oscillators.hh"
#include "WalkStates.hh"
#include "SwingStanceState.hh"
#include "MainState.hh"
#include "CtrlInputs.hh"
#include "UserCtrl.hh"
#include "AverageInc.hh"
#include "CtrlOptions.hh"

/*! \brief Matsuoka oscillators with six neurons
 */
class MatsuokaSixN: public Oscillators
{
	public:
		MatsuokaSixN(int nb_neurons, int cur_t, WalkStates *ws, CtrlInputs *inputs, CtrlOptions *options);
		virtual ~MatsuokaSixN();

		virtual void update(double cur_t);

		void Matsuoka_six_neurons();
		void update_speed_oscillos();
		void update_speed_oscillos(double v_request);
		void integrate_fatigue(double cur_t);
		void check_osc_strike();
		void compute_osc_excitation();
		void oscillator_prediction_error(double cur_t);

		double get_k_GLU()     const { return k_GLU;     }
		double get_k_HFL()     const { return k_HFL;     }
		double get_k_HFLrun1()     const { return k_HFLrun1;     }
		double get_k_HFLrun2()     const { return k_HFLrun2;     }
		double get_k_HAM1()    const { return k_HAM1;    }
		double get_k_HAM2()    const { return k_HAM2;    }
		double get_theta_ref() const { return theta_ref; }

		double get_t_osc_error_mean() const { return t_osc_error_mean; }

		void set_v_request(double value) { v_request = value; }

		void set_beta_A(double value) { beta_A = value; }
		void set_beta_B(double value) { beta_B = value; }
		void set_beta_C(double value) { beta_C = value; }

		void set_gamma_A(double value) { gamma_A = value; }
		void set_gamma_B(double value) { gamma_B = value; }
		void set_gamma_C(double value) { gamma_C = value; }

		void set_eta_A(double value) { eta_A = value; }
		void set_eta_B(double value) { eta_B = value; }
		void set_eta_C(double value) { eta_C = value; }
		void set_eta_D(double value) { eta_D = value; }
		void set_eta_E(double value) { eta_E = value; }
		void set_eta_F(double value) { eta_F = value; }
		void set_eta_G(double value) { eta_G = value; }

		void set_P_theta(double value) { P_theta = value; }
		void set_P_tau(double value)   { P_tau = value; }
		void set_P_GLU(double value)   { P_GLU = value; }
		void set_P_HFL(double value)   { P_HFL = value; }
		void set_P_HAM1(double value)  { P_HAM1 = value; }
		void set_P_HAM2(double value)  { P_HAM2 = value; }

		void set_p_theta(double value) { p_theta = value; }
		void set_p_tau(double value)   { p_tau = value; }
		void set_p_HFL(double value)   { p_HFL = value; }
		void set_p_HAM1(double value)  { p_HAM1 = value; }
		void set_p_HAM2(double value)  { p_HAM2 = value; }

	private:
		std::vector<double> v;  ///< fatigue for neurons
		std::vector<double> vd; ///< fatigue for neurons (derivative)

		WalkStates *ws;          ///< walk states
		SwingStanceState *sw_st; ///< swing-stance state
		MainState *m_st;         ///< main state

		CtrlInputs *inputs;     ///< controller inputs
		UserCtrl *user_ctr;     ///< user control
		AverageInc *osc_err_av; ///< mean errors on the oscillators

		int flag_strike_err;          ///< flag to compute strike error
		int flag_strike_leg[NB_LEGS]; ///< flag corresponding to a strike for a precise leg

		int r_first_swing; ///< 1 if right leg first in swing, 0 otherwise

		// oscillators main parameters
		double beta_A;
		double beta_B;
		double beta_C;

		double gamma_A;
		double gamma_B;
		double gamma_C;

		double eta_A;
		double eta_B;
		double eta_C;
		double eta_D;
		double eta_E;
		double eta_F;
		double eta_G;

		double tau_inv;
		double tau_A_inv;
		double tau_B_inv;
		double tau_C_inv;

		// velocity adaptation parameters
		double P_theta;
		double P_tau;
		double P_GLU;
		double P_HFL;
		double P_HAM1;
		double P_HAM2;

		double p_theta;
		double p_tau;
		double p_HFL;
		double p_HAM1;
		double p_HAM2;

		// velocity tracking
		double v_star;
		double v_request;

		double theta_ref;
		double tau;


		double k_GLU;
		double k_HFL;
		double k_HAM1;
		double k_HAM2;
        double k_HFLrun1;   
        double k_HFLrun2;

		// integration
		double last_t;

		// time init oscillo
		double init_t_oscillo;

		// oscillators prediction errors
		int flag_osc_too_fast;
		double t_osc_too_fast;
		double t_osc_error;
		double t_osc_error_mean;

		int flag_range; ///< 1 if CPG range activated, 0 otherwise

        // Running alignment fields
        // TODO: Assumes bipedal
        int flag_last_stance_leg_r;  /// Flag set when r leg was last stance leg
};

#endif
