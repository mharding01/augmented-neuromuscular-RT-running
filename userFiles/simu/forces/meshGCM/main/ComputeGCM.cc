
#include "ComputeGCM.hh"
#include "WholeFlexFeet.hh"
#include "WholeInitialFeet.hh"
#include "WholeRectLongFeet.hh"
#include "WholeShortFeet.hh"
#include "WholeRectFlexFeet.hh"
#include "WholeRigidFlexFeet.hh"
#include "WholeSpringToeFeet.hh"
#include "WholeSpringToeShortFeet.hh"

#include "FlatGround.hh"
#include "LinearGround.hh"
#include "RndObsGround.hh"
#include "TrainingGround.hh"

#include "user_IO.h"

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] gait_features gait features
 * \param[in] sens_info info from sensors
 */
ComputeGCM::ComputeGCM(MbsData *mbs_data, GaitFeatures *gait_features, SensorsInfo *sens_info)
{
	this->mbs_data = mbs_data;
	this->gait_features = gait_features;
	this->sens_info = sens_info;

	if (mbs_data->user_IO->options->opti)
	{
		// Train using training ground
		ground = new TrainingGround(mbs_data, gait_features, sens_info);
	}
	else 
	{
		ground = new FlatGround(mbs_data, gait_features, sens_info);
	}
	coman_model = mbs_data->user_IO->options->coman_model;

	switch (coman_model)
	{
		case INIT_FEET_COMAN:
			whole_feet = new WholeInitialFeet(mbs_data, gait_features, sens_info, ground);
			break;

		case SHORT_FEET_COMAN:
		case SHORT_FEET_BALL_COMAN:
			whole_feet = new WholeShortFeet(mbs_data, gait_features, sens_info, ground);
			break;

		case LONG_FEET_COMAN:
			whole_feet = new WholeRectLongFeet(mbs_data, gait_features, sens_info, ground);
			break;

		case FLEX_FEET_COMAN:
			whole_feet = new WholeFlexFeet(mbs_data, gait_features, sens_info, ground);
			break;

		case SPRING_TOE_FEET_COMAN:
			whole_feet = new WholeSpringToeFeet(mbs_data, gait_features, sens_info, ground);
			break;

		case SPRING_TOE_SHORT_FEET_COMAN:
			whole_feet = new WholeSpringToeShortFeet(mbs_data, gait_features, sens_info, ground);
			break;
	
		default:
			std::cout << "Error: unknown COMAN model for whole feet !" << std::endl;
			break;
	}
}

/*! \brief destructor
 */
ComputeGCM::~ComputeGCM()
{
	delete whole_feet;

	delete ground;
}

/*! \brief compute the external ground reactions related to one ext force sensor
 * 
 * \param[out] F_tot external force output [N]
 * \param[out] T_tot external torque output [Nm]
 * \param[in] PxF absolute position (provided by Robotran) [m]
 * \param[in] RxF absolute rotation matrix (provided by Robotran) [-]
 * \param[in] VxF absolute position derivative (provided by Robotran) [m/s]
 * \param[in] OMxF absolute rotation derivatives (provided by Robotran) [rad/s]
 * \param[in] index index of the xetrnal sensor
 */
void ComputeGCM::compute_F_T(double F_tot[3], double T_tot[3], double PxF[4], double RxF[4][4], double VxF[4], double OMxF[4], int index)
{	
	whole_feet->compute_F_T(F_tot, T_tot, PxF, RxF, VxF, OMxF, index);
}

/*! \brief computation done once during a simulation time step
 */
void ComputeGCM::state_compute()
{
	whole_feet->state_compute();
}
