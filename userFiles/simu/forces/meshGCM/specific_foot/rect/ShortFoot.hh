/*! 
 * \author Nicolas Van der Noot
 * \file ShortFoot.hh
 * \brief ShortFoot class
 */

#ifndef _SHORT_FOOT_HH_
#define _SHORT_FOOT_HH_

#include "ContactFoot.hh"

/*! \brief Model of a contact foot: short foot
 */
class ShortFoot: public ContactFoot
{
	public:
		ShortFoot(MbsData *mbs_data, GaitFeatures *gait_features, SensorsInfo *sens_info, int foot_id);
		virtual ~ShortFoot();

	protected:
		virtual Fsens_info* get_F_Sens_body();
};

#endif
