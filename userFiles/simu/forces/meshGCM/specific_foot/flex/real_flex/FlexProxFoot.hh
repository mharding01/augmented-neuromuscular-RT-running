/*! \brief FlexProxFoot class
 */

#ifndef _FLEX_PROX_FOOT_HH_
#define _FLEX_PROX_FOOT_HH_

#include "ContactFoot.hh"

/*! \brief Model of a contact foot: flexible foot, part 1 (flat on the ground)
 */
class FlexProxFoot: public ContactFoot
{
	public:
		FlexProxFoot(MbsData *mbs_data, GaitFeatures *gait_features, SensorsInfo *sens_info, int foot_id);
		virtual ~FlexProxFoot();

	protected:
		virtual Fsens_info* get_F_Sens_body();
};

#endif
