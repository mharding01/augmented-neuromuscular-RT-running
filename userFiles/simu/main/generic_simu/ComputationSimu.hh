/*! 
 * \author Nicolas Van der Noot
 * \file ComputationSimu.hh
 * \brief ComputationSimu class
 */

#ifndef _COMPUTATION_SIMU_HH_
#define _COMPUTATION_SIMU_HH_

#include "ModelSimuIndex.hh"
#include "SimuOptions.h"
#include <iostream>

extern "C" {
	#include "mbs_data.h"
}

/*! \brief Mother class of the computation instances used in simulation
 */
class ComputationSimu
{
	public:
		ComputationSimu(MbsData *mbs_data, ModelSimuIndex *simu_index);
		virtual ~ComputationSimu();

		virtual void compute() = 0;

	protected:
		MbsData *mbs_data;     ///< Robotran structure
		ModelSimuIndex *simu_index; ///< simulation indexes
		SimuOptions *options;  ///< simulation options
};

#endif
