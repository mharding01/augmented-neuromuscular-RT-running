/*! 
 * \author Nicolas Van der Noot
 * \file DelayWalk.hh
 * \brief DelayWalk class
 */

#ifndef _DELAY_WALK_HH_
#define _DELAY_WALK_HH

#include "DelayManager.hh"

enum{F_FOOT_R, F_FOOT_L, 
	PHI_K_R, PHI_K_L, PHIP_K_R, PHIP_K_L, 
	THETA_TORSO, OMEGA_TORSO,
	LCE_TA_R, LCE_TA_L, LCE_VAS_R, LCE_VAS_L,
	F_SOL_R, F_SOL_L, F_VAS_R, F_VAS_L, DELAY_WALK_NB};

/*! \brief Manage delays on signals for walking gait stimulations
 */
class DelayWalk: public DelayManager
{
	public:
		DelayWalk();
		virtual ~DelayWalk();
};

#endif

