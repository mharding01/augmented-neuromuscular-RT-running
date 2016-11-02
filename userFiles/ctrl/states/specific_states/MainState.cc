#include "MainState.hh"

/*! \brief constructor
 */
MainState::MainState(CtrlInputs *inputs, CtrlOptions *options): WalkState(inputs, options)
{
	t = inputs->get_t();

	coman_state = (options->get_flag_ctrl() == CTRL_STIM_TEST) ? TEST_STATE : INIT_UPRIGHT_STATE;
	coman_state = ( (options->is_Qq_match_wang()) || (options->is_initial_pos()) ) ? WALK_COMAN_STATE : INIT_UPRIGHT_STATE;

	t_walk_start   = 2.0;

	if (coman_state == WALK_COMAN_STATE)
	{
		init_t_walk = 0.0; // for SwingStanceState FSM
	}
	else
	{
		init_t_walk = 0.0;
	}
}

/*! \brief destructor
 */
MainState::~MainState()
{

}

/*! \brief compute the main state
 */
void MainState::compute()
{
	// get timing
	t = inputs->get_t();

	// FSM
	switch (coman_state)
	{
		case TEST_STATE :
		case WALK_COMAN_STATE :
			// no state change
			break;

		case INIT_UPRIGHT_STATE :
			if (t >= t_walk_start)
			{
				coman_state = WALK_COMAN_STATE;
				init_t_walk = t;
			}
			break;
	}
}
