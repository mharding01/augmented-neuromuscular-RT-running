/*! \brief RigidFlexShape class
 */

#ifndef _RIGID_FLEX_SHAPE_HH_
#define _RIGID_FLEX_SHAPE_HH_

#include "ContactFoot.hh"

/*! \brief Model of a contact foot: rigid foot with the same shape as the global flex foot
 */
class RigidFlexShape: public ContactFoot
{
	public:
		RigidFlexShape(MbsData *mbs_data, GaitFeatures *gait_features, SensorsInfo *sens_info, int foot_id);
		virtual ~RigidFlexShape();

	protected:
		virtual Fsens_info* get_F_Sens_body();
};

#endif
