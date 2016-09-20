/*! 
 * \author Nicolas Van der Noot
 * \file ShortShortFoot.hh
 * \brief ShortShortFoot class
 */

#ifndef _SHORT_SHORT_FOOT_HH_
#define _SHORT_SHORT_FOOT_HH_

#include "ContactFoot.hh"

/*! \brief Model of a contact foot: short foot
 */
class ShortShortFoot: public ContactFoot
{
	public:
		ShortShortFoot(MbsData *mbs_data, GaitFeatures *gait_features, SensorsInfo *sens_info, int foot_id);
		virtual ~ShortShortFoot();

	protected:
		virtual Fsens_info* get_F_Sens_body();
};

#endif
