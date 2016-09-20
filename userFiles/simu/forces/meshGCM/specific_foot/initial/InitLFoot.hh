/*! 
 * \author Nicolas Van der Noot
 * \file InitLFoot.hh
 * \brief InitLFoot class
 */

#ifndef _INIT_L_FOOT_HH_
#define _INIT_L_FOOT_HH_

#include "ContactFoot.hh"

/*! \brief Model of a contact foot: initial foot left
 */
class InitLFoot: public ContactFoot
{
	public:
		InitLFoot(MbsData *mbs_data, GaitFeatures *gait_features, SensorsInfo *sens_info, int foot_id);
		virtual ~InitLFoot();

	protected:
		virtual Fsens_info* get_F_Sens_body();
};

#endif
