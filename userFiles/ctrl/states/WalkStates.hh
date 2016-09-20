/*! 
 * \author Nicolas Van der Noot
 * \file WalkStates.hh
 * \brief WalkStates class
 */

#ifndef _WALK_STATES_HH_
#define _WALK_STATES_HH_

#include "Computation.hh"
#include "CtrlInputs.hh"
#include "WalkState.hh"

enum {MAIN_STATE, SWING_STANCE_STATE, TRAILING_STATE, NB_WALK_STATES};

/*! \brief States of the walking robot
 */
class WalkStates: public Computation
{
	public:
		WalkStates(CtrlInputs *inputs, CtrlOptions *options, MotorCtrlIndex *ctrl_index);
		virtual ~WalkStates();

		virtual void compute();

		WalkState* get_state(int i) const { return states[i]; }

	private:
		std::vector<WalkState*> states;
};

#endif
