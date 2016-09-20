#include "TrailingState.hh"

enum{ONE_IN_SWING, AFTER_LANDING, BEFORE_TAKE_OFF};

/*! \brief constructor
 */
TrailingState::TrailingState(CtrlInputs *inputs, CtrlOptions *options, SwingStanceState *sw_st): WalkState(inputs, options)
{
	this->sw_st   = sw_st;

	initial_pos = options->is_initial_pos();
	Qq_wang = options->is_Qq_match_wang();

	if (initial_pos)
	{
		trailing_leg[R_ID] = 0;
		trailing_leg[L_ID] = 0;

		last_r_swing = 0;
	}
	else if (options->is_r_first_swing())
	{
		trailing_leg[R_ID] = 1;
		trailing_leg[L_ID] = 0;

		last_r_swing = 0;
	}
	else
	{
		trailing_leg[R_ID] = 0;
		trailing_leg[L_ID] = 1;

		last_r_swing = 1;
	}

	trailing_state = BEFORE_TAKE_OFF;
}

/*! \brief destructor
 */
TrailingState::~TrailingState()
{

}

/*! \brief compute the state of each leg to know if it is a trailing leg or not
 */
void TrailingState::compute()
{
	double mean_Fz_R_foot = inputs->get_mean_Fz_feet(R_ID);
	double mean_Fz_L_foot = inputs->get_mean_Fz_feet(L_ID);

	// double support
	if ( (!sw_st->is_swing_leg(R_ID)) && (!sw_st->is_swing_leg(L_ID)) )
	{
		switch (trailing_state)
		{
			case ONE_IN_SWING:
				trailing_state = AFTER_LANDING;
				break;
				
			case AFTER_LANDING:
				if (last_r_swing) // last leg in swing: right leg
				{
					if(mean_Fz_R_foot > mean_Fz_L_foot)
					{
						trailing_state = BEFORE_TAKE_OFF;
						trailing_leg[R_ID] = 0;
						trailing_leg[L_ID] = 1;
					}
				}
				else // last leg in swing: left leg
				{
					if(mean_Fz_L_foot > mean_Fz_R_foot)
					{
						trailing_state = BEFORE_TAKE_OFF;
						trailing_leg[R_ID] = 1;
						trailing_leg[L_ID] = 0;
					}
				}
				break;
				
			default:
				break;
		}
	}
	// one leg in swing phase
	else
	{
		trailing_leg[R_ID] = 0;
		trailing_leg[L_ID] = 0;
		
		trailing_state = ONE_IN_SWING;

		last_r_swing = (mean_Fz_R_foot > mean_Fz_L_foot) ? 0 : 1;
	}	

	if (Qq_wang)
	{
		trailing_leg[R_ID] = 0;
	}
}
