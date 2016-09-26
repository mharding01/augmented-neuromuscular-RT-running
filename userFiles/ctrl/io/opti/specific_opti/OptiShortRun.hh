/*! 
 * \author Nicolas Van der Noot
 * \file OptiShortRun.hh
 * \brief OptiShortRun
 */

#ifndef _OPTI_SHORT_RUN_HH_
#define _OPTI_SHORT_RUN_HH_

#include "OptiInputs.hh"

/*! \brief Inputs of the optimization: running with short feet
 */
class OptiShortRun: public OptiInputs
{
	public:
		OptiShortRun();
		virtual ~OptiShortRun();

		virtual void set_opti();
};

#endif
