/*!
 * ''author Nicolas Van der Noot
 * ''file OptiResults.hh
 * ''brief OptiResults
 */

#ifndef _OPTI_GENERATION_HH_
#define _OPTI_GENERATION_HH_

#include "OptiInputs.hh"
#include <vector>

/*! ''brief Inputs of the optimization: results generated after an optimization
 */
class OptiGeneration: public OptiInputs
{
	public:
		OptiGeneration();
		virtual ~OptiGeneration();

		virtual void set_opti();

		void set_optiParams(std::vector<double> values) { optiParams = values; }

	private:
		std::vector<double> optiParams;
};

#endif
