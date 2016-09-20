/*! 
 * \author Nicolas Van der Noot
 * \file RectToeShortFoot.hh
 * \brief RectToeShortFoot class
 */

#ifndef _RECT_TOE_SHORT_FOOT_HH_
#define _RECT_TOE_SHORT_FOOT_HH_

#include "ContactFoot.hh"

/*! \brief Model of a contact foot: rectangular toe (distal foot)
 */
class RectToeShortFoot: public ContactFoot
{
	public:
		RectToeShortFoot(MbsData *mbs_data, GaitFeatures *gait_features, SensorsInfo *sens_info, int foot_id);
		virtual ~RectToeShortFoot();

	protected:
		virtual Fsens_info* get_F_Sens_body();
};

#endif
