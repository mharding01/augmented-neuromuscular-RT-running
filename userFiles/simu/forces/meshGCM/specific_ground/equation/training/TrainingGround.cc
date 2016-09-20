
#include "TrainingGround.hh"

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] gait_features gait features
 * \param[in] sens_info information coming from the sensors
 */
TrainingGround::TrainingGround(MbsData *mbs_data, GaitFeatures *gait_features, SensorsInfo *sens_info): EquationGround(mbs_data, gait_features, sens_info)
{

}

/*! \brief destructor
 */
TrainingGround::~TrainingGround()
{

}

/*! \brief compute specific values for the ground model
 *
 * nothing to do for this ground
 */
void TrainingGround::compute()
{
	
}
