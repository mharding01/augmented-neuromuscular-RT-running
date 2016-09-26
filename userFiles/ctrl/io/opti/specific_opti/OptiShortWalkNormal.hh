/*! 
 * \author Nicolas Van der Noot
 * \file OptiShortWalkNormal.hh
 * \brief OptiShortWalkNormal
 */

#ifndef _OPTI_SHORT_WALK_NORMAL_HH_
#define _OPTI_SHORT_WALK_NORMAL_HH_

#include "OptiInputs.hh"

/*! \brief Inputs of the optimization: walking fast with normal feet
 */
class OptiShortWalkNormal: public OptiInputs
{
	public:
		OptiShortWalkNormal();
		virtual ~OptiShortWalkNormal();

		virtual void set_opti();
};

#endif
