#include "SwingStanceState.hh"

#define FOOT_THRESHOLD 20.0 // [N]
#define FOOT_THRESHOLD_INIT 50.0 // [N]

#define TOE_THRESHOLD 20.0 // [N]
#define TOE_THRESHOLD_INIT 50.0 // [N]

#define MIN_STANCE_TIME 0.2 // [s]
#define MIN_SWING_TIME 0.15 // [s]
#define WAIT_BEFORE_SWING 0.005 // [s]

#define THRESHOLD_FOOT_ABS 0.6 // [rad]

#define T_START_SECOND_CYCLE 1.114125 // [s] = mns_data->tf/2.0 (1.06975 for 4m/s)
 
/*! \brief constructor
 */
SwingStanceState::SwingStanceState(CtrlInputs *inputs, CtrlOptions *options, MainState *main_state): WalkState(inputs, options)
{
	// classes pointers
	this->main_state = main_state;

	// time
	t = inputs->get_t();

	Qq_wang = options->is_Qq_match_wang();
	initial_pos = options->is_initial_pos();
	
	safety_time_walk = 0.2;

	foot_threshold = FOOT_THRESHOLD_INIT;
	toe_threshold = TOE_THRESHOLD_INIT;

	nb_strikes = 0;

	flag_strike = 0;

	for(int i=0; i<NB_LEGS; i++)
	{
		if(initial_pos) //start with flight positions
			swing_leg[i] = 1;
		else
			swing_leg[i] = 0;
		flag_strike_leg[i] = 0;

		last_t_stance[i] = t;

		t_start_last_stance_leg[i] = -MIN_STANCE_TIME;
		t_start_last_swing_leg[i]  = -MIN_SWING_TIME;
	}

	supporting_r_leg = options->is_r_first_swing() ? 0 : 1;

	t_start_last_stance = -MIN_STANCE_TIME;
	t_start_last_swing  = -MIN_SWING_TIME;

	per_swing_wang = 0.4;

	// for optimization
	inputs->get_opti_inputs()->set_sw_st(this);
}

/*! \brief destructor
 */
SwingStanceState::~SwingStanceState()
{

}

/*! \brief compute check if each leg is in swing or in stance phase
 */
void SwingStanceState::compute()
{
	double foot_pitch;

	// set flags to 0
	flag_strike = 0;

	for(int i=0; i<NB_LEGS; i++)
	{
		flag_strike_leg[i] = 0;
	}

	// time
	t = inputs->get_t();

    /* TODO:
	// before initial swing
	if (inputs->get_t() < main_state->get_init_t_walk() + safety_time_walk)
	{
		for(int i=0; i<NB_LEGS; i++)
		{
			last_t_stance[i] = t;
		}

		return;
	}
    */

	// loop on both legs
	for(int i=0; i<NB_LEGS; i++)
	{
		if (i == R_ID)
		{
			foot_pitch = inputs->get_theta_Rfoot(1);
		}
		else
		{
			foot_pitch = inputs->get_theta_Lfoot(1);
		}

		// swing
		if (possible_swing(i))
		{
			// foot off
			if( (!swing_leg[i]) && (t - t_start_last_stance_leg[i] > MIN_STANCE_TIME) && (t - last_t_stance[i] > WAIT_BEFORE_SWING) )
			{
				if (nb_strikes > 0)
				{
					t_start_last_swing = t;
					t_start_last_swing_leg[i] = t;
				}

				swing_leg[i] = 1;

				supporting_r_leg = (i == R_ID) ? 0 : 1;
			}
		}
		else // stance
		{
			// strike
			if ( swing_leg[i] && (foot_pitch < THRESHOLD_FOOT_ABS) && (t - t_start_last_swing_leg[i] > MIN_SWING_TIME) )
			{
				nb_strikes++;

				t_start_last_stance = t;
				t_start_last_stance_leg[i] = t;

				swing_leg[i] = 0;

				supporting_r_leg = (i == R_ID) ? 1 : 0;

				flag_strike = 1;
				flag_strike_leg[i] = 1;
			}

			last_t_stance[i] = t;
		}
	}

	if (Qq_wang)
	{
		swing_leg[R_ID] = ( ((t >= per_swing_wang*T_START_SECOND_CYCLE) && (t < T_START_SECOND_CYCLE)) || (t >= (1.0+per_swing_wang)*T_START_SECOND_CYCLE) );
		nb_strikes = 10;
		swing_leg[L_ID] = 1; //no double support
	}
}

/*! \brief detect if the leg is possibly in swing phase
 * 
 * \param[in] leg_id leg ID
 * \return 1 if possible swing, 0 otherwise
 */
int SwingStanceState::possible_swing(int leg_id)
{
	// lower threshold after the first step
	if (nb_strikes > 0)
	{
		foot_threshold = FOOT_THRESHOLD;
		toe_threshold = TOE_THRESHOLD;
	}

	return ( (inputs->get_mean_Fz_feet(leg_id) < foot_threshold) && (inputs->get_Fz_feet(leg_id) < foot_threshold) && (inputs->get_mean_Fz_toe(leg_id) < toe_threshold) && (inputs->get_Fz_toe(leg_id) < toe_threshold));
}
