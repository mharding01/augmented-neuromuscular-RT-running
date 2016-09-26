/*! 
 * \author Nicolas Van der Noot
 * \file OptiInit.hh
 * \brief OptiInit
 */

#ifndef _OPTI_INIT_HH_
#define _OPTI_INIT_HH_

#include "OptiInputs.hh"

/*! \brief Inputs of the optimization: initial optimization parameters
 */
class OptiInit: public OptiInputs
{
	public:
		OptiInit();
		virtual ~OptiInit();

		virtual void set_opti();
};

#endif
