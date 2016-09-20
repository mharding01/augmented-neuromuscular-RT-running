/*! 
 * \author Nicolas Van der Noot
 * \file RectLongFoot.hh
 * \brief RectLongFoot class
 */

#ifndef _RECT_LONG_FOOT_HH_
#define _RECT_LONG_FOOT_HH_

#include "ContactFoot.hh"

/*! \brief Model of a contact foot: rectangular long foot
 */
class RectLongFoot: public ContactFoot
{
	public:
		RectLongFoot(MbsData *mbs_data, GaitFeatures *gait_features, SensorsInfo *sens_info, int foot_id);
		virtual ~RectLongFoot();

	protected:
		virtual Fsens_info* get_F_Sens_body();
};

#endif
