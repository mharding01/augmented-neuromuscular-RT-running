/*! 
 * \author Nicolas Van der Noot
 * \file OptiResults.hh
 * \brief OptiResults
 */

#ifndef _OPTI_RESULTS_HH_
#define _OPTI_RESULTS_HH_

#include "OptiInputs.hh"

/*! \brief Inputs of the optimization: results generated after an optimization
 */
class OptiResults: public OptiInputs
{
	public:
		OptiResults();
		virtual ~OptiResults();

		virtual void set_opti();
};

#endif
