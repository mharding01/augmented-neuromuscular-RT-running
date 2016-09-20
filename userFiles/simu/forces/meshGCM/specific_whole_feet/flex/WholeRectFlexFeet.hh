/*! 
 * \author Nicolas Van der Noot
 * \file WholeRectFlexFeet.hh
 * \brief WholeRectFlexFeet class
 */

#ifndef _WHOLE_RECT_FLEX_FEET_HH_
#define _WHOLE_RECT_FLEX_FEET_HH_

#include "WholeFeet.hh"

/*! \brief Whole feet: rigid rectangular feet with the length of 'WholeFlexFeet'
 */
class WholeRectFlexFeet: public WholeFeet
{
	public:
		WholeRectFlexFeet(MbsData *mbs_data, GaitFeatures *gait_features, SensorsInfo *sens_info, GroundModel *ground);
		virtual ~WholeRectFlexFeet();

		virtual void compute_F_T(double F_tot[3], double T_tot[3], double PxF[4], double RxF[4][4], double VxF[4], double OMxF[4], int index);

		virtual double get_leg_feet_forces(int leg_id, int axis);
		virtual double get_leg_feet_torques(int leg_id, int axis);
};

#endif
