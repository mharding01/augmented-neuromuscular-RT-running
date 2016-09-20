/*! 
 * \author Nicolas Van der Noot
 * \file FlexDistFoot.hh
 * \brief FlexDistFoot class
 */

#ifndef _FLEX_DIST_FOOT_HH_
#define _FLEX_DIST_FOOT_HH_

#include "ContactFoot.hh"

/*! \brief Model of a contact foot: flexible foot, part 2 (flat on the ground)
 */
class FlexDistFoot: public ContactFoot
{
	public:
		FlexDistFoot(MbsData *mbs_data, GaitFeatures *gait_features, SensorsInfo *sens_info, int foot_id);
		virtual ~FlexDistFoot();

	protected:
		virtual Fsens_info* get_F_Sens_body();
};

#endif
