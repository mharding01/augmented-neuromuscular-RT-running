/*! 
 * \author Nicolas Van der Noot
 * \file OptiShortWalkFast.hh
 * \brief OptiShortWalkFast
 */

#ifndef _OPTI_SHORT_WALK_FAST_HH_
#define _OPTI_SHORT_WALK_FAST_HH_

#include "OptiInputs.hh"

/*! \brief Inputs of the optimization: walking fast with short feet
 */
class OptiShortWalkFast: public OptiInputs
{
	public:
		OptiShortWalkFast();
		virtual ~OptiShortWalkFast();

		virtual void set_opti();
};

#endif
