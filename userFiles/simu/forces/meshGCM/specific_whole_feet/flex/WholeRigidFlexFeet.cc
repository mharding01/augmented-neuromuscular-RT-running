
#include "WholeRigidFlexFeet.hh"
#include "RigidFlexShape.hh"
#include "SimuIndex.hh"

#define NB_FEET_BODIES 2

/*! \brief contructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] gait_features features of the gait
 * \param[in] sens_info information from sensors
 * \param[in] ground ground model
 */
WholeRigidFlexFeet::WholeRigidFlexFeet(MbsData *mbs_data, GaitFeatures *gait_features, SensorsInfo *sens_info, GroundModel *ground): WholeFeet(mbs_data, gait_features, sens_info, ground)
{
	for(int i=0; i<NB_FEET_BODIES; i++)
	{
		switch (i)
		{
			case RIGHT_ID : bodies.push_back(new RigidFlexShape(mbs_data, gait_features, sens_info, R_FOOT_ID)); break;
			case LEFT_ID  : bodies.push_back(new RigidFlexShape(mbs_data, gait_features, sens_info, L_FOOT_ID )); break;
			
			default: break;
		}
	}
}

/*! \brief destructor
 */
WholeRigidFlexFeet::~WholeRigidFlexFeet()
{
	// delete already done in mother class
}

/*! \brief compute the external ground reactions related to this foot
 * 
 * \param[out] F_tot external force output [N]
 * \param[out] T_tot external torque output [Nm]
 * \param[in] PxF absolute position (provided by Robotran) [m]
 * \param[in] RxF absolute rotation matrix (provided by Robotran) [-]
 * \param[in] VxF absolute position derivative (provided by Robotran) [m/s]
 * \param[in] OMxF absolute rotation derivatives (provided by Robotran) [rad/s]
 * \param[in] index index of the external sensor
 */
void WholeRigidFlexFeet::compute_F_T(double F_tot[3], double T_tot[3], double PxF[4], double RxF[4][4], double VxF[4], double OMxF[4], int index)
{
	ContactFoot *body;

	switch ( index ) 
	{
		case SimuFsensIndex::RightFoot:
			body = bodies[RIGHT_ID];
			break;

		case SimuFsensIndex::LeftFoot:
			body = bodies[LEFT_ID];
			break;
	
		default:
			body = NULL;
			break;
	}

	body->update_pose(PxF, RxF, VxF, OMxF);

	ground->compute_F_T(F_tot, T_tot, body);
}

/*! \brief get feet forces
 * 
 * \param[in] leg_id ID of the leg
 * \param[in] axis axis requested
 * \return force requested [N]
 */
double WholeRigidFlexFeet::get_leg_feet_forces(int leg_id, int axis)
{
	switch (leg_id)
	{
		case RIGHT_ID:
			return bodies[RIGHT_ID]->get_F_tot(axis);
			break;

		case LEFT_ID:
			return bodies[LEFT_ID]->get_F_tot(axis);
			break;
	
		default:
			return 0.0;
			break;
	}
}

/*! \brief get feet torques
 * 
 * \param[in] leg_id ID of the leg
 * \param[in] axis axis requested
 * \return torque requested [Nm]
 */
double WholeRigidFlexFeet::get_leg_feet_torques(int leg_id, int axis)
{
	switch (leg_id)
	{
		case RIGHT_ID:
			return bodies[RIGHT_ID]->get_T_tot(axis);
			break;

		case LEFT_ID:
			return bodies[LEFT_ID]->get_T_tot(axis);
			break;
	
		default:
			return 0.0;
			break;
	}
}
