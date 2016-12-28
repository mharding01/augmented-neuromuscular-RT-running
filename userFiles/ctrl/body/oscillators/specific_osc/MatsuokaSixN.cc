#include <stdio.h>
#include "MatsuokaSixN.hh"

#define NB_STEPS_OSC_ERROS 0 ///< number of steps before computing the oscillators prediction errors [-]
#define OSC_EXCITATION 1.0   ///< excitation of the oscillators

#define MIN_THETA_REF 0.005

inline double pos(double x){ return (x > 0.0) ?  x : 0.0; }
inline double neg(double x){ return (x < 0.0) ? -x : 0.0; }

#define NB_Y_OUTPUTS 8

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

	gamma_A = 1.57553931;
	gamma_B = 2.53633384; // 2.130010;
	gamma_C = 2.65514190;

	eta_A = 5.49681774;
	eta_B = 4.86147345;
	eta_C = 5.68391234;
	eta_D = 3.73377651;
	eta_E = 3.66622041;

    // Added props
	
    eta_F = 4.473370;
	eta_G = 3.501070;

	// velocity adaptation parameters
	P_theta_trunk = 0.02114376; // Taken from StimWang new defaults
    P_theta_hip = 0.60620732;
	P_tau   = 0.04747445;
	P_GLU   = 1.065147;
	P_HFL   = 3.689370;
	P_HAM1  = 2.189173;
	P_HAM2  = 1.686624;
    
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

    // TODO: Default params for running stims' CPG weights
    k_HFLrun1 = 1.59619401;
    k_HFLrun2 = 7.01533279;
    k_HAMrun3 = 6.20870389; 
    
    // Init delayed "opti_set" params to defaults
    opt_k_HFLrun1 = k_HFLrun1 ;
    opt_k_HFLrun2 = k_HFLrun2 ;
    opt_k_HAMrun3 = k_HAMrun3 ;
    opt_P_theta_trunk = P_theta_trunk;
    opt_P_theta_hip = P_theta_hip;
    opt_P_tau = P_tau;

	// velocity tracking
	v_star = 0.6;

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

	//set_plot(v_request, "target speed [m/s]");

	// linear function	
	theta_trunk_ref = P_theta_trunk  + p_theta_trunk * v_diff;
	theta_hip_ref = P_theta_hip  + p_theta_hip * v_diff;
	tau       = P_tau    + p_tau   * v_diff; 
	k_GLU     = P_GLU;
	k_HFL     = P_HFL    + p_HFL  * v_diff;
	k_HAM1    = P_HAM1   + p_HAM1 * v_diff;
	k_HAM2    = P_HAM2   + p_HAM2 * v_diff;

	// limiting the interpolations
	theta_trunk_ref = (theta_trunk_ref < MIN_THETA_REF) \
                        ? MIN_THETA_REF : theta_trunk_ref;
    theta_hip_ref = (theta_hip_ref < MIN_THETA_REF) \
                        ? MIN_THETA_REF : theta_hip_ref;
	tau       = (tau       < 0.0) ? 0.0 : tau;
	k_GLU     = (k_GLU     < 0.0) ? 0.0 : k_GLU;
	k_HFL     = (k_HFL     < 0.0) ? 0.0 : k_HFL;
	k_HAM1    = (k_HAM1    < 0.0) ? 0.0 : k_HAM1;
	k_HAM2    = (k_HAM2    < 0.0) ? 0.0 : k_HAM2;

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


	// after right strike (only x[1] positive)
	if ( flag_strike_leg[R_ID] && (x[0] < 0.0) && (x[1] > 0.0) && (x[3] < 0.0) && (x[4] < 0.0) )
	{
		flag_strike_leg[R_ID] = 0;
	}

	// after left strike (only x[4] positive)
	if ( flag_strike_leg[L_ID] && (x[0] < 0.0) && (x[1] < 0.0) && (x[3] < 0.0) && (x[4] > 0.0) )
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
        if ( (!flag_last_stance_leg_r && sw_st->is_flight_phase() && (x[1] > 0.0)) 
            ||
             (flag_last_stance_leg_r && sw_st->is_flight_phase() && (x[4] > 0.0)) )
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
	
			// handles oscillators too late (first line) and oscillators succession (second line)
			u[0] -= flag_strike_leg[R_ID]*pos(x[0]);
			u[0] -= pos(x[0])*supporting_r;
			
            u[1] += flag_strike_leg[R_ID]*neg(x[1]);

			u[3] -= flag_strike_leg[L_ID]*pos(x[3]);
			u[3] -= pos(x[3])*supporting_l;

			u[4] += flag_strike_leg[L_ID]*neg(x[4]);


			if (sw_st->get_nb_strikes() > 0) // first step done
			{
                // Attenuate signals: N1/N3 while R stance, N4/N6 while L stance
				u[0] -= pos(x[0])*supporting_r;
				u[2] -= pos(x[2])*supporting_r;
				u[3] -= pos(x[3])*supporting_l;
				u[5] -= pos(x[5])*supporting_l;
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
    if ( (!flag_last_stance_leg_r && sw_st->is_flight_phase() && (x[1] > 0.0)) 
        ||
         (flag_last_stance_leg_r && sw_st->is_flight_phase() && (x[4] > 0.0)) )
    {
		if (!flag_osc_too_fast)
		{
			flag_osc_too_fast = 1;
			t_osc_too_fast    = cur_t;
		}
	}

	// synchronization finished after strike and correction
	if ( flag_strike_err && ( (sw_st->is_supporting_r_leg() && (x[1] > 0.0)) 
        || 
        ((!sw_st->is_supporting_r_leg() && !sw_st->is_flight_phase()) && (x[4] > 0.0)) ) )
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
}

/*! \brief update oscillators output
 * 
 * \param[in] cur_t current time [s]
 */
void MatsuokaSixN::update(double cur_t)
{
	// update oscillators velocity parameters at a strike
	/*if (flag_range) TODO: && (sw_st->get_nb_strikes() >= 4)) */
    if (1)  // TODO:
	{
		//update_speed_oscillos(user_ctr->get_v_request());
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
	y[0] = pos(x[0]) - pos(x[1]);
	y[1] = pos(x[2]) - pos(x[1]);
	y[2] = pos(x[3]) - pos(x[4]);
	y[3] = pos(x[5]) - pos(x[4]);
    y[4] = pos(x[1]) - pos(x[0]);
    y[5] = pos(x[4]) - pos(x[3]);
    
    y[6] = pos(x[0]) + pos(x[5]);
    y[7] = pos(x[3]) + pos(x[2]);

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
    k_HAMrun3 = opt_k_HAMrun3;
    P_theta_trunk = opt_P_theta_trunk;
    P_theta_hip = opt_P_theta_hip;
    P_tau = opt_P_tau;
}
