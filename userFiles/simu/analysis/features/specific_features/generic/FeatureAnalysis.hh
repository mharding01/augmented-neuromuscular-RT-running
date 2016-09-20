/*! 
 * \author Nicolas Van der Noot
 * \file FeatureAnalysis.hh
 * \brief FeatureAnalysis class
 */

#ifndef _FEATURE_ANALYSIS_HH_
#define _FEATURE_ANALYSIS_HH_

#include "user_realtime.h"
#include "mbs_data.h"
#include "ModelSimuIndex.hh"

#include <cmath>
#include <iostream>

enum{RIGHT_ID, LEFT_ID, N_LEGS};

/*! \brief Analysis of the gait feature mother class
 */
class FeatureAnalysis
{
	public:
		FeatureAnalysis(MbsData *mbs_data, ModelSimuIndex *simu_index);
		virtual ~FeatureAnalysis();

		virtual void compute() = 0;

	protected:
		MbsData *mbs_data; ///< Robotran structure
		ModelSimuIndex *simu_index; ///< simulation indexes
};

#endif
