
#include "GaitFeatures.hh"
#include "SwingStanceAnalysis.hh"
#include "MeanSpeedAnalysis.hh"

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] sens_info info from the sensors
 */
GaitFeatures::GaitFeatures(MbsData *mbs_data, ModelSimuIndex *simu_index, SensorsInfo *sens_info): ComputationSimu(mbs_data, simu_index)
{
	this->sens_info = sens_info;

	for(int i=0; i<NB_GAIT_FEATURES; i++)
	{
		switch (i)
		{
			case SWING_STANCE_FEAT:
				list_features.push_back(new SwingStanceAnalysis(mbs_data, simu_index, sens_info));
				break;

			case MEAN_SPEED_FEAT:
				list_features.push_back(new MeanSpeedAnalysis(mbs_data, simu_index, sens_info));
				break;
		
			default:
				break;
		}
	}
}

/*! \brief destructor
 */
GaitFeatures::~GaitFeatures()
{
	for(unsigned int i=0; i<list_features.size(); i++)
	{
		delete list_features[i];
	}
}

/*! \brief compute all the gait features
 */
void GaitFeatures::compute()
{
	for(unsigned int i=0; i<list_features.size(); i++)
	{
		list_features[i]->compute();
	}
}

/*! \brief get x position of the last strike
 * 
 * \param[in] foot_id ID of the foot
 * \return x position of the last strike [m]
 */
double GaitFeatures::get_x_last_strike_step(int foot_id)
{
	return static_cast<SwingStanceAnalysis*>(list_features[SWING_STANCE_FEAT])->get_x_last_strike_step(foot_id);
}
