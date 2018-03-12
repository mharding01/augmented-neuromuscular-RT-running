#include <stdio.h>
#include "MatsuokaSixN.hh"
#include "set_output.h"

#define NB_STEPS_OSC_ERROS 0 ///< number of steps before computing the oscillators prediction errors [-]
#define OSC_EXCITATION 1.0   ///< excitation of the oscillators

#define MIN_THETA_REF 0.005

inline double pos(double x){ return (x > 0.0) ?  x : 0.0; }
inline double neg(double x){ return (x < 0.0) ? -x : 0.0; }

#define NB_Y_OUTPUTS 6 

/*! \brief constructor
 * 
 * \param[in] nb_neurons number of neurons [-]
 * \param[in] cur_t current time [s]
 * \param[in] ws walk states
 * \param[in] inputs controller inputs
 */
MatsuokaSixN::MatsuokaSixN(int nb_neurons, int cur_t, WalkStates *ws, CtrlInputs *inputs, CtrlOptions *options): Oscillators(nb_neurons, cur_t)
{
	osc_err_av = new AverageInc();

	last_t = cur_t;

	this->ws = ws;
	sw_st = static_cast<SwingStanceState*>(ws->get_state(SWING_STANCE_STATE));
	m_st = static_cast<MainState*>(ws->get_state(MAIN_STATE));

	this->inputs = inputs;

	user_ctr = inputs->get_user_ctrl();

	for(int i=0; i<nb_neurons; i++)
	{
		v.push_back(0.0);
		vd.push_back(0.0);
	}

	for(unsigned int i=0; i<NB_Y_OUTPUTS; i++)
	{
		y.push_back(0.0);
	}

	for(int i=0; i<NB_LEGS; i++)
	{
		flag_strike_leg[i] = 0;
	}

	flag_strike_err = 0;

	flag_osc_too_fast = 0;
	t_osc_too_fast = 0.0;

	t_osc_error = 0.0;
	t_osc_error_mean = 0.0;

	r_first_swing = options->is_r_first_swing();

	// oscillos initialization time
	init_t_oscillo = 0.01;

	// oscillators main parameters
	beta_A = 6.043252;
	beta_B = 5.067806;
	beta_C = 4.382409;

	gamma_A = 1.59030347;
	gamma_B = 2.39640033; 
	gamma_C = 2.84350615;

	eta_A =5.46761414 ;
	eta_B =4.88355275 ;
	eta_C =5.66314746 ;
	eta_D =3.77916453 ;
	eta_E =3.70464392 ;

    // Added props 
	
    eta_F = 4.473370;
	eta_G = 3.501070;

	// velocity adaptation parameters
	P_theta_trunk = 0.06586455; // Taken from StimWang new defaults
    P_theta_hip = -0.16986034;	// TODO: 0.60620732;
	P_tau   = 0.04676336;
	P_GLU   = 1.065147;
	P_HFL   = 3.689370;
	P_HAM1  = 2.189173;
	P_HAM2  = 1.686624;
	P_G_SOL = 3.20496133;
	P_G_SOL_TA = 3.15520137;
	P_G_GAS = 9.63917261;
	P_G_VAS = 1.64607262;
	P_k_theta = 6.68941640;

    
    /*
	p_theta = 0.502487;
	p_tau   = -0.058664;
	p_HFL   = 3.057062;
	p_HAM1  = 1.576165;
	p_HAM2  = -2.829820;
    */


    p_theta_trunk = 0.0; // TODO: for now, these are zero'd, come back when speed modulating 
    p_theta_hip = 0.0; // TODO: for now, these are zero'd, come back when speed modulating 

    p_tau   = 0.0; // TODO: for now, these are zero'd, come back when speed modulating 
    p_HFL   = 0.0; // TODO: for now, these are zero'd, come back when speed modulating 
    p_HAM1  = 0.0; // TODO: for now, these are zero'd, come back when speed modulating     
    p_HAM2  = 0.0; // TODO: for now, these are zero'd, come back when speed modulating     
	p_k_HFLrun1 = 0.0;
	p_k_HFLrun2 = 0.0;
	p_k_HAMrun = 0.0;
	p_G_SOL = 0.0;
	p_G_SOL_TA = 0.0;
	p_G_GAS = 0.0;
	p_G_VAS = 0.0;
	p_k_theta = 0.0;

	p2_tau = 0.0;
	p2_theta_hip= 0.0;
	p2_theta_trunk = 0.0;
	p2_k_HFLrun1 = 0.0;
	p2_k_HFLrun2 = 0.0;
	p2_k_HAMrun = 0.0;
	p2_G_SOL = 0.0;
	p2_G_VAS = 0.0;

    // TODO: Default params for running stims' CPG weights
    P_k_HFLrun1 =4.17021254;	//1.59619401;
    P_k_HFLrun2 =9.16549638;	//7.01533279;
    P_k_HAMrun =3.35982891;	//6.20870389; 
    
    // Init delayed "opti_set" params to defaults
    opt_k_HFLrun1 = k_HFLrun1 ;
    opt_k_HFLrun2 = k_HFLrun2 ;
    opt_k_HAMrun = k_HAMrun ;
    opt_P_theta_trunk = P_theta_trunk;
    opt_P_theta_hip = P_theta_hip;
    opt_P_tau = P_tau;
    opt_P_G_SOL = P_G_SOL;
    opt_P_G_SOL_TA = P_G_SOL_TA;
    opt_P_G_GAS = P_G_GAS;
    opt_P_G_VAS = P_G_VAS;
    opt_P_k_theta = P_k_theta;

	// velocity tracking
	v_star = 1.6;
	vel_track_enabled = 0;

	// flag for CPG range
	flag_range = options->is_cpg_range();

    // flag for last stance leg
    flag_last_stance_leg_r = !r_first_swing;
	
	update_speed_oscillos();

	// for opti
	inputs->get_opti_inputs()->set_osc(this);

}

/*! \brief destructor
 */
MatsuokaSixN::~MatsuokaSixN()
{
	delete osc_err_av;
}

/*! \brief Matsuoka oscillators with six neurons structure (Nico version)
 */
void MatsuokaSixN::Matsuoka_six_neurons()
{
	// neurons equations
	xd[0] = tau_inv * ( -x[0] - beta_A*v[0] - eta_A*pos(x[3]) - eta_D*pos(x[1]) - eta_E*pos(x[4]) + u[0] );
	xd[1] = tau_inv * ( -x[1] - beta_B*v[1] - eta_B*pos(x[4]) - eta_D*pos(x[0]) - eta_E*pos(x[3]) + u[1] );
	xd[2] = tau_inv * ( -x[2] - beta_C*v[2] - eta_C*pos(x[5]) - eta_F*pos(x[1]) - eta_G*pos(x[4]) + u[2] );
	xd[3] = tau_inv * ( -x[3] - beta_A*v[3] - eta_A*pos(x[0]) - eta_D*pos(x[4]) - eta_E*pos(x[1]) + u[3] );
	xd[4] = tau_inv * ( -x[4] - beta_B*v[4] - eta_B*pos(x[1]) - eta_D*pos(x[3]) - eta_E*pos(x[0]) + u[4] );
	xd[5] = tau_inv * ( -x[5] - beta_C*v[5] - eta_C*pos(x[2]) - eta_F*pos(x[4]) - eta_G*pos(x[1]) + u[5] );

	// fatigue
	vd[0] = tau_A_inv * ( -v[0] + pos(x[0]) );
	vd[1] = tau_B_inv * ( -v[1] + pos(x[1]) );
	vd[2] = tau_C_inv * ( -v[2] + pos(x[2]) );
	vd[3] = tau_A_inv * ( -v[3] + pos(x[3]) );
	vd[4] = tau_B_inv * ( -v[4] + pos(x[4]) );
	vd[5] = tau_C_inv * ( -v[5] + pos(x[5]) ); 
    
    set_plot(x[0], "x1");
    //set_plot(x[1], "x2");
    //set_plot(x[3] , "x4");
    set_plot(x[4] , "x5");
    //set_plot(x[2] , "x3");
    //set_plot(x[5] , "x6");

    // Plot fatigue
    //set_plot(v[0], "V1");
    //set_plot(v[1], "V2");
    //set_plot(v[3] , "V4");
    //set_plot(v[4] , "V5");
    //set_plot(v[2] , "V3");
    //set_plot(v[5] , "V6");
    
}

/*! \brief 'update_speed_oscillos' with speed reference set to 'v_star'
 */
void MatsuokaSixN::update_speed_oscillos()
{
	update_speed_oscillos(v_star);
}

/*! \brief update oscillators velocity parameters
 * 
 * \param[in] v_request requested speed (m/s)
 */
void MatsuokaSixN::update_speed_oscillos(double v_request)
{
	double v_diff = v_request - v_star;

	this->v_request = v_request;

	set_plot(v_request, "target speed [m/s]");
	set_output(v_request, "target_speed");

    // constant functions
	theta_trunk_ref = P_theta_trunk;
    theta_hip_ref   = P_theta_hip;
	k_theta	        = P_k_theta;

	// linear function	
	double v_diff_sq = v_diff * v_diff;
	k_HFLrun1 = P_k_HFLrun1             + p_k_HFLrun1  * v_diff;
	G_sol_ta  = P_G_SOL_TA 				+ p_G_SOL_TA * v_diff;
	G_gas 	  = P_G_GAS 				+ p_G_GAS * v_diff;
	G_vas	  = P_G_VAS 	            + p_G_VAS * v_diff;
	
	// Quadratic functions
	tau       = P_tau    	+ p_tau * v_diff 		+ p2_tau * v_diff_sq;
	k_HFLrun2 = P_k_HFLrun2	+ p_k_HFLrun2 * v_diff 	+ p2_k_HFLrun2 * v_diff_sq;
	k_HAMrun  = P_k_HAMrun 	+ p_k_HAMrun * v_diff 	+ p2_k_HAMrun * v_diff_sq;
	G_sol	  = P_G_SOL 	+ p_G_SOL * v_diff 		+ p2_G_SOL * v_diff_sq;

	// limiting the interpolations
	theta_trunk_ref = (theta_trunk_ref < MIN_THETA_REF) \
                        ? MIN_THETA_REF : theta_trunk_ref;
//    theta_hip_ref = (theta_hip_ref < MIN_THETA_REF) \
//                        ? MIN_THETA_REF : theta_hip_ref;
	tau       = (tau       < 0.0) ? 0.0 : tau;
	k_HFLrun1 = (k_HFLrun1     < 0.0) ? 0.0 : k_HFLrun1;
	k_HFLrun2 = (k_HFLrun2		< 0.0) ? 0.0 : k_HFLrun2;
	k_HAMrun  = (k_HAMrun    	< 0.0) ? 0.0 : k_HAMrun;
	G_sol	  = (G_sol < 0.0) ? 0.0 : G_sol;
	G_sol_ta  = (G_sol_ta < 0.0) ? 0.0 : G_sol_ta;
	G_gas	  = (G_gas < 0.0) ? 0.0 : G_gas;
	G_vas	  = (G_vas < 0.0) ? 0.0 : G_vas;
	k_theta	  = (k_theta < 0.0) ? 0.0 : k_theta;

	//set_plot(theta_trunk_ref, "trunk ref");
	//set_plot(theta_hip_ref, "hip_ref");
	//set_plot(tau, "tau");
	//set_plot(k_HFLrun1, "kHFL1");
	//set_plot(k_HFLrun2, "kHFL2");
	//set_plot(k_HAMrun, "kHAM");
	//set_plot(G_sol, "G_sol");
	//set_plot(G_sol_ta, "G_sol_ta");
	//set_plot(G_gas, "G_gas");
	//set_plot(G_vas, "G_vas");
	//set_plot(k_theta, "k_theta");
		
	// oscillators period
	tau_inv   = 1.0 / tau;
	tau_A_inv = 1.0 / (gamma_A * tau);
	tau_B_inv = 1.0 / (gamma_B * tau);
	tau_C_inv = 1.0 / (gamma_C * tau);
}

/*! \brief integrate fatigue in neurons
 * 
 * \param[in] cur_t current time [s]
 */
void MatsuokaSixN::integrate_fatigue(double cur_t)
{
	for(unsigned int i=0; i<x.size(); i++)
	{
		v[i] += vd[i] * (cur_t - last_t);
	}

	last_t = cur_t;
}

/*! \brief check when the oscillators are no more too late after the corresponding strike
 */
void MatsuokaSixN::check_osc_strike()
{
	if (sw_st->get_flag_strike())
	{
		flag_strike_err = 1;
	}

	for(int i=0; i<NB_LEGS; i++)
	{
		if (sw_st->get_flag_strike_leg(i))
		{
			flag_strike_leg[i] = 1;
            // Set flag_last_stance_leg 
            flag_last_stance_leg_r = flag_strike_leg[R_ID];
		}
	}


	// after right strike (only x[4] positive)
	if ( flag_strike_leg[R_ID] && (x[0] < 0.0) && (x[1] < 0.0) && (x[3] < 0.0) && (x[4] > 0.0) )
	{
		flag_strike_leg[R_ID] = 0;
	}

	// after left strike (only x[1] positive)
	if ( flag_strike_leg[L_ID] && (x[0] < 0.0) && (x[1] > 0.0) && (x[3] < 0.0) && (x[4] < 0.0) )
	{
		flag_strike_leg[L_ID] = 0;
	}
}

/*! \brief compute oscillators excitation
 */
void MatsuokaSixN::compute_osc_excitation()
{
	int supporting_r, supporting_l;

	// initialization: choose the corresponding intial leg
	if (inputs->get_t() < m_st->get_init_t_walk() + init_t_oscillo)
	{
		if (r_first_swing)
		{
			u[0] = 0.0;
			u[1] = 0.0;  // N2 aligns with left foot-strike 
			u[2] = 0.0;
			u[3] = 0.0;
			u[4] = OSC_EXCITATION;  // Now N5 aligns with right foot-strike
			u[5] = 0.0;
		}
		else
		{
			u[0] = 0.0;
			u[1] = OSC_EXCITATION; // N2 aligns with left foot-strike
			u[2] = 0.0;
			u[3] = 0.0;
			u[4] = 0.0; // N5 aligns with right foot-strike 
			u[5] = 0.0;
		}
	}
	else
	{
		for(unsigned int i=0; i<u.size(); i++)
		{
			u[i] = OSC_EXCITATION;
		}
		
        // cut the excitations if signals too fast
		/*if ( (sw_st->is_swing_leg(L_ID) && (x[1] > 0.0)) || (sw_st->is_swing_leg(L_ID) && (x[4] > 0.0)) )*/
        // TODO: Running code, anticipating next foot-strike after flight phase
        if ( (!flag_last_stance_leg_r && sw_st->is_flight_phase() && (x[4] > 0.0)) 
            ||
             (flag_last_stance_leg_r && sw_st->is_flight_phase() && (x[1] > 0.0)) )
		{
			for(unsigned int i=0; i<u.size(); i++)
			{
				u[i] = 0.0;
			}
		}
		else
		{
			if (sw_st->is_supporting_r_leg())
			{
				supporting_r = 1;
				supporting_l = 0;
			}
            else if (sw_st->is_flight_phase()) 
            {   
                // Added flight phase support
                supporting_r = 0;
                supporting_l = 0;
            }
			else
			{
				supporting_r = 0;
				supporting_l = 1;
			}
	
			// handles oscillators too late (first 2 lines) and oscillators succession (second line)
			// Ordering: N5 -> N1 -> N2 -> N4, N5/N2 are RFS/LFS-aligned (see eta's)
			u[0] -= flag_strike_leg[L_ID]*pos(x[0]);
			
            u[1] += flag_strike_leg[L_ID]*neg(x[1]);

			u[3] -= flag_strike_leg[R_ID]*pos(x[3]);

			u[4] += flag_strike_leg[R_ID]*neg(x[4]);


			if (sw_st->get_nb_strikes() > 0) // first step done
			{
                // Attenuate signals: N1/N3 while L stance, N4/N6 while R stance
				u[0] -= pos(x[0])*supporting_l;
				u[1] -= pos(x[1])*supporting_r;
				u[2] -= pos(x[2])*supporting_l;
				u[3] -= pos(x[3])*supporting_r;
				u[4] -= pos(x[4])*supporting_l;
				u[5] -= pos(x[5])*supporting_r;
			}
		}
	}
}

/*! \brief error on the oscillators prediction
 * 
 * \param[in] cur_t current time [s]
 */
void MatsuokaSixN::oscillator_prediction_error(double cur_t)
{
	// oscillators too fast
    /*if ( (sw_st->is_swing_leg(L_ID) && (x[1] > 0.0)) || (sw_st->is_swing_leg(L_ID) && (x[4] > 0.0)) )*/
    // TODO: Running code, anticipating next foot-strike after flight phase
    if ( (!flag_last_stance_leg_r && sw_st->is_flight_phase() && (x[4] > 0.0)) 
        ||
         (flag_last_stance_leg_r && sw_st->is_flight_phase() && (x[1] > 0.0)) )
    {
		if (!flag_osc_too_fast)
		{
			flag_osc_too_fast = 1;
			t_osc_too_fast    = cur_t;
		}
	}

	// synchronization finished after strike and correction
	if ( flag_strike_err && ( (sw_st->is_supporting_r_leg() && (x[4] > 0.0)) 
        || 
        ((!sw_st->is_supporting_r_leg() && !sw_st->is_flight_phase()) && (x[1] > 0.0)) ) )
	{
		flag_strike_err = 0;

		// oscillators too fast
		if (flag_osc_too_fast)
		{
			flag_osc_too_fast = 0;

			t_osc_error = sw_st->get_t_start_last_stance() - t_osc_too_fast;
		}
		// oscillators too slow
		else
		{
			t_osc_error = cur_t - sw_st->get_t_start_last_stance();
		}

		// mean on these predictions errors after some steps
		if (sw_st->get_nb_strikes() > NB_STEPS_OSC_ERROS)
		{
			t_osc_error_mean = osc_err_av->update_and_get(t_osc_error);
		}
	}
	//set_plot(t_osc_error_mean, "osc_err");
}

/*! \brief update oscillators output
 * 
 * \param[in] cur_t current time [s]
 */
void MatsuokaSixN::update(double cur_t)
{
	// update oscillators velocity parameters at after some initialization
	if ((flag_range) && (sw_st->get_nb_strikes() >= 6)) 
	{
		if (!vel_track_enabled)
		{
			//TODO: enable_velocity_tracking();
			vel_track_enabled = 1;
		}
		update_speed_oscillos(user_ctr->get_v_request());
	}
	else 
	{
		// Or, update for same speed 
		// necessary if opti is different from default values
        update_speed_oscillos();
	}
	// checking for too slow oscillators
	check_osc_strike();

	// start excitations when the robot start to walk
	if (m_st->get_coman_state() != INIT_UPRIGHT_STATE)
	{
		compute_osc_excitation();
	}

	// oscillators differential equations
	Matsuoka_six_neurons();
	
	// Euler explicite integration 
	integrate_neurons(cur_t);
	integrate_fatigue(cur_t);

	// oscillators outputs
	y[0] = pos(x[0]); 
	y[1] = pos(x[1]); 
	y[2] = pos(x[2]); 
	y[3] = pos(x[3]); 
    y[4] = pos(x[4]); 
    y[5] = pos(x[5]); 

	// oscillators prediction error
	oscillator_prediction_error(cur_t);
}

/*! \brief
* Sets oscillator fields manually, allowing for delayed updating of parameters
* during optimization.
*/
void MatsuokaSixN::delayed_opti_set()
{
    // Updates 6 running parameters according to their "opt" (optimized) counterparts
    k_HFLrun1 = opt_k_HFLrun1;
    k_HFLrun2 = opt_k_HFLrun2;
    k_HAMrun = opt_k_HAMrun;
	// Corresponding values of below need to be set by update_speed_oscillos()
    P_theta_trunk = opt_P_theta_trunk;
    P_theta_hip = opt_P_theta_hip;
    P_tau = opt_P_tau;
	P_G_SOL = opt_P_G_SOL;
	P_G_SOL_TA = opt_P_G_SOL_TA;
	P_G_GAS = opt_P_G_GAS;
	P_G_VAS = opt_P_G_VAS;
	P_k_theta = opt_P_k_theta;
}

/*! \brief
 * Sets variables used to compute parameter values according to target velocity
*/
void MatsuokaSixN::enable_velocity_tracking()
{
	// theta_trunk_ref
	p2_theta_trunk = 0.0;	// Linear
	p_theta_trunk = -0.053268754307278801;
	P_theta_trunk = 0.064766798107021073;

	// theta_hip_ref
	p2_theta_hip = -0.0;	// Linear
	p_theta_hip = -0.064716819051908916;
	P_theta_hip = -0.15823267374820044;
	
	// k_theta
	p_k_theta = -1.1240650780815897;
	P_k_theta = 8.4327796131278934;

	// G_gas
	p_G_GAS = -17.539064653262727;
	P_G_GAS = 7.1463689581438032;
	
	// G_sol_ta
	p_G_SOL_TA = 11.609061240492416;
	P_G_SOL_TA = 5.4588662971259367;

	// k_HFLrun1
	p2_k_HFLrun1 = 0.0;	// Linear
	p_k_HFLrun1 = -3.7206222446606367;
	P_k_HFLrun1 = 5.0553508873464876;

	// G_vas
	p2_G_VAS = 0.0; // Linear 6.9194895163924643;
	p_G_VAS = 2.3551627453468131;
	P_G_VAS = 1.6223384239722289;

	// G_sol
	p2_G_SOL = 0.0; // Linear
	p_G_SOL = 0.89076265526480658;
	P_G_SOL = 3.1033301302152658;

	// k_HFLrun2
	p2_k_HFLrun2 = 0.0; // Linear 69.263066403460812;
	p_k_HFLrun2 = 9.0976526263553641; //22.770831475395369;
	P_k_HFLrun2 = 6.258079620370145; //5.8898560925247807;

	// k_HAMrun
	p2_k_HAMrun = 0.0;	// Linear
	p_k_HAMrun = 2.6284766761384448;
	P_k_HAMrun = 4.3145569168227453;

	// Tau
	p2_tau = 0.0;	// Linear
	p_tau = -0.011473110394246446;
	P_tau = 0.046661707239609634;
}
