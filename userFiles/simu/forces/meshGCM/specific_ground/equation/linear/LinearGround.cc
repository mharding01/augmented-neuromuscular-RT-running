
#include "LinearGround.hh"

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] gait_features gait features
 * \param[in] sens_info information coming from the sensors
 */
LinearGround::LinearGround(MbsData *mbs_data, GaitFeatures *gait_features, SensorsInfo *sens_info): EquationGround(mbs_data, gait_features, sens_info)
{

}

/*! \brief destructor
 */
LinearGround::~LinearGround()
{

}

/*! \brief compute specific values for the ground model
 *
 * nothing to do for this ground
 */
void LinearGround::compute()
{
	
}
