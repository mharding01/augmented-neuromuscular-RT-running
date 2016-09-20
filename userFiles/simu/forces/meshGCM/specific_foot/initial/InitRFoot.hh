/*! 
 * \author Nicolas Van der Noot
 * \file InitRFoot.hh
 * \brief InitRFoot class
 */

#ifndef _INIT_R_FOOT_HH_
#define _INIT_R_FOOT_HH_

#include "ContactFoot.hh"

/*! \brief Model of a contact foot: initial foot right
 */
class InitRFoot: public ContactFoot
{
	public:
		InitRFoot(MbsData *mbs_data, GaitFeatures *gait_features, SensorsInfo *sens_info, int foot_id);
		virtual ~InitRFoot();

	protected:
		virtual Fsens_info* get_F_Sens_body();
};

#endif
