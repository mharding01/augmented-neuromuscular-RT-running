/*! 
 * \author Nicolas Van der Noot
 * \file RectToeFoot.hh
 * \brief RectToeFoot class
 */

#ifndef _RECT_TOE_FOOT_HH_
#define _RECT_TOE_FOOT_HH_

#include "ContactFoot.hh"

/*! \brief Model of a contact foot: rectangular toe (distal foot)
 */
class RectToeFoot: public ContactFoot
{
	public:
		RectToeFoot(MbsData *mbs_data, GaitFeatures *gait_features, SensorsInfo *sens_info, int foot_id);
		virtual ~RectToeFoot();

	protected:
		virtual Fsens_info* get_F_Sens_body();
};

#endif
