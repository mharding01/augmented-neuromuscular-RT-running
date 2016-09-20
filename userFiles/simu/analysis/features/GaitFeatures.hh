/*! 
 * \author Nicolas Van der Noot
 * \file GaitFeatures.hh
 * \brief GaitFeatures class
 */

#ifndef _GAIT_FEATURES_HH_
#define _GAIT_FEATURES_HH_

#include "ComputationSimu.hh"
#include "SensorsInfo.hh"
#include "FeatureAnalysis.hh"

#include <vector>

enum{SWING_STANCE_FEAT, MEAN_SPEED_FEAT, NB_GAIT_FEATURES};

/*! \brief Gait features analysis
 */
class GaitFeatures: public ComputationSimu
{
	public:
		GaitFeatures(MbsData *mbs_data, ModelSimuIndex *simu_index, SensorsInfo *sens_info);
		virtual ~GaitFeatures();

		virtual void compute();

		double get_x_last_strike_step(int foot_id);

		/// get list_features element
		FeatureAnalysis* get_feature(int index) const { return list_features[index]; }

	private:
		SensorsInfo *sens_info; ///< info from the sensors
		std::vector<FeatureAnalysis*> list_features; ///< list with all the features to anlyze
};

#endif
