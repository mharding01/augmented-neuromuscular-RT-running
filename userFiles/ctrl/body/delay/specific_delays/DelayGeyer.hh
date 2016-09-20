/*! 
 * \author Nicolas Van der Noot
 * \file DelayGeyer.hh
 * \brief DelayGeyer class
 */

#ifndef _DELAY_GEYER_HH_
#define _DELAY_GEYER_HH_

#include "DelayManager.hh"

enum{F_FOOT_R, F_FOOT_L, 
	PHI_K_R, PHI_K_L, PHIP_K_R, PHIP_K_L, PHI_H_R, PHI_H_L, PHIP_H_R, PHIP_H_L, 
	THETA_TORSO, OMEGA_TORSO, 
	LCE_TA_R, LCE_TA_L, LCE_VAS_R, LCE_VAS_L, LCE_HAM_R, LCE_HAM_L, LCE_HFL_R, LCE_HFL_L,
	F_SOL_R, F_SOL_L, F_VAS_R, F_VAS_L, F_GAS_R, F_GAS_L, F_HAM_R, F_HAM_L, F_GLU_R, F_GLU_L, DELAY_GEYER_NB};

/*! \brief Manage delays on signals for Geyer stimulations
 */
class DelayGeyer: public DelayManager
{
	public:
		DelayGeyer();
		virtual ~DelayGeyer();
};

#endif

