/*! 
 * \author Nicolas Van der Noot
 * \file WalkState.hh
 * \brief WalkState class
 */

#ifndef _WALK_STATE_HH_
#define _WALK_STATE_HH_

#include "CtrlInputs.hh"

/*! \brief generic state for the COMAN walk
 */
class WalkState
{
	public:
		WalkState(CtrlInputs *inputs, CtrlOptions *options);
		virtual ~WalkState();

		virtual void compute() = 0;

	protected:
		CtrlInputs *inputs; ///< controller inputs
		CtrlOptions *options; ///< options
};

#endif
