#include "WalkStates.hh"
#include "MainState.hh"
#include "SwingStanceState.hh"
#include "TrailingState.hh"

#include <iostream>
#include <stdlib.h>


/*! \brief constructor
 * 
 * \param[in] inputs controller inputs
 * \param[in] options controller options
 * \param[in] ctrl_index controller index lists
 */
WalkStates::WalkStates(CtrlInputs *inputs, CtrlOptions *options, MotorCtrlIndex *ctrl_index):
	Computation(inputs, options, ctrl_index)
{
	int flag_main, flag_sw_st;

	MainState *main_st;
	SwingStanceState *sw_st;

	flag_main  = 0;
	flag_sw_st = 0;

	for(int i=0; i<NB_WALK_STATES; i++)
	{
		switch (i)
		{
			case MAIN_STATE:

				flag_main = 1;

				main_st = new MainState(inputs, options);
				states.push_back(main_st);
				break;

			case SWING_STANCE_STATE:

				if (!flag_main)
				{
					std::cout << "Error: MainState not created !" << std::endl;
					exit(EXIT_FAILURE);
				}

				flag_sw_st = 1;

				sw_st = new SwingStanceState(inputs, options, main_st);
				states.push_back(sw_st);
				break;

			case TRAILING_STATE:

				if (!flag_sw_st)
				{
					std::cout << "Error: SwingStanceState not created !" << std::endl;
					exit(EXIT_FAILURE);
				}

				states.push_back(new TrailingState(inputs, options, sw_st));
				break;

			default:
				std::cout << "Error: unknown walk state index !" << std::endl;
				exit(EXIT_FAILURE);
				break;
		}
	}
}

/*! \brief destructor
 */
WalkStates::~WalkStates()
{
	for(unsigned int i=0; i<states.size(); i++)
	{
		delete states[i];
	}
}

/*! \brief main computation
 */
void WalkStates::compute()
{
	for(unsigned int i=0; i<states.size(); i++)
	{
		states[i]->compute();
	}
}
