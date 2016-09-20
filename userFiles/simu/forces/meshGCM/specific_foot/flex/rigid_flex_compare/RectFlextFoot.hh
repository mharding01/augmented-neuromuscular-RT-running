/*! 
 * \author Nicolas Van der Noot
 * \file RectFlextFoot.hh
 * \brief RectFlextFoot class
 */

#ifndef _RECT_SHORT_FOOT_HH_
#define _RECT_SHORT_FOOT_HH_

#include "ContactFoot.hh"

/*! \brief Model of a contact foot: short foot with the same size and position as the whole flexfoot
 */
class RectFlextFoot: public ContactFoot
{
	public:
		RectFlextFoot(MbsData *mbs_data, GaitFeatures *gait_features, SensorsInfo *sens_info, int foot_id);
		virtual ~RectFlextFoot();

	protected:
		virtual Fsens_info* get_F_Sens_body();
};

#endif
