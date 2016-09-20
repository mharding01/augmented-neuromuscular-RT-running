/*! 
 * \author Nicolas Van der Noot
 * \file WholeRigidFlexFeet.hh
 * \brief WholeRigidFlexFeet class
 */

#ifndef _WHOLE_RIGID_FLEX_FEET_HH_
#define _WHOLE_RIGID_FLEX_FEET_HH_

#include "WholeFeet.hh"

/*! \brief Whole feet: rigid feet with the shape of 'WholeFlexFeet'
 */
class WholeRigidFlexFeet: public WholeFeet
{
	public:
		WholeRigidFlexFeet(MbsData *mbs_data, GaitFeatures *gait_features, SensorsInfo *sens_info, GroundModel *ground);
		virtual ~WholeRigidFlexFeet();

		virtual void compute_F_T(double F_tot[3], double T_tot[3], double PxF[4], double RxF[4][4], double VxF[4], double OMxF[4], int index);

		virtual double get_leg_feet_forces(int leg_id, int axis);
		virtual double get_leg_feet_torques(int leg_id, int axis);
};

#endif
