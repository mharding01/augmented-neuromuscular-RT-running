/*! 
 * \author Nicolas Van der Noot
 * \file OptiLongRun.hh
 * \brief OptiLongRun
 */

#ifndef _OPTI_LONG_RUN_HH_
#define _OPTI_LONG_RUN_HH_

#include "OptiInputs.hh"

/*! \brief Inputs of the optimization: running with long feet
 */
class OptiLongRun: public OptiInputs
{
	public:
		OptiLongRun();
		virtual ~OptiLongRun();

		virtual void set_opti();
};

#endif
