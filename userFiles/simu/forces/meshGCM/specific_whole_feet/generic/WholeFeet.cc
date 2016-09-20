
#include "WholeFeet.hh"
#include "SimuIndex.hh"

/*! \brief contructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] gait_features features of the gait
 * \param[in] sens_info information from sensors
 * \param[in] ground ground model
 */
WholeFeet::WholeFeet(MbsData *mbs_data, GaitFeatures *gait_features, SensorsInfo *sens_info, GroundModel *ground)
{
	this->sens_info     = sens_info;
	this->mbs_data      = mbs_data;
	this->gait_features = gait_features;
	this->ground        = ground;
}

/*! \brief destructor
 */
WholeFeet::~WholeFeet()
{
	for (unsigned int i=0; i<bodies.size(); i++)
	{
		delete bodies[i];
	}
}

/*! \brief computation called only once each valid time-step
 */
void WholeFeet::state_compute()
{
	for(unsigned int i=0; i<bodies.size(); i++)
	{
		ground->state_switch(bodies[i]);
	}
}

/*! \brief get feet toes forces. 0 if no toes
 * 
 * \param[in] leg_id ID of the leg
 * \param[in] axis axis requested
 * \return force requested [N]
 */
double WholeFeet::get_leg_feet_dist_forces(int leg_id, int axis)
{
	return 0.0;
}
