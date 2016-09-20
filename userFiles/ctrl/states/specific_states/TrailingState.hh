/*! 
 * \author Nicolas Van der Noot
 * \file TrailingState.hh
 * \brief TrailingState class
 */

#ifndef _TRAILING_STATE_HH_
#define _TRAILING_STATE_HH_

#include "WalkState.hh"
#include "SwingStanceState.hh"

/*! \brief state of the COMAN legs: trailing or not
 */
class TrailingState: public WalkState
{
	public:
		TrailingState(CtrlInputs *inputs, CtrlOptions *options, SwingStanceState *sw_st);
		virtual ~TrailingState();

		virtual void compute();

		int get_trailing_leg(int i) const { return trailing_leg[i]; }

	private:
		int Qq_wang;
		int initial_pos;
		
		int last_r_swing;   ///< 1 if last leg in swing is the right one
		int trailing_state; ///< general state of the COMAN according to the trailing legs
		int trailing_leg[NB_LEGS]; ///< 1 if corresponding leg (for each leg) is in trailing state, 0 otherwise

		SwingStanceState *sw_st; ///< swing-stance detection
};

#endif
