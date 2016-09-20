/*! 
 * \author Nicolas Van der Noot & Francois Heremans
 * \file Leg.hh
 * \brief Leg class
 */

#ifndef _LEG_HH_
#define _LEG_HH_

#include "BodyPart.hh"

// pitch muscles
#include "SOL_Muscle.hh"
#include "TA_Muscle.hh"
#include "GAS_Muscle.hh"
#include "VAS_Muscle.hh"
#include "HAM_Muscle.hh"
#include "GLU_Muscle.hh"
#include "HFL_Muscle.hh"
#include "RF_Muscle.hh"

// roll muscles
#include "HAB_Muscle.hh"
#include "HAD_Muscle.hh"
#include "EVE_Muscle.hh"
#include "INV_Muscle.hh"

// yaw muscles
#include "HER_Muscle.hh"
#include "HIR_Muscle.hh"

/*! \brief Leg
 */
class Leg: public BodyPart
{
	public:
		Leg(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index, int leg_id);
		virtual ~Leg();

		virtual void update_S_ref();

	private:
		std::vector<double> F_min_pitch;     ///< pitch muscles minimal value [N]
		std::vector<double> l_ce_star_pitch; ///< pitch muscles 'l_ce' value to get the minimal 'F_m' [m]

		// specific muscles : pitch
		SOL_Muscle *sol;
		TA_Muscle *ta;
		GAS_Muscle *gas;
		VAS_Muscle *vas;
		HAM_Muscle *ham;
		GLU_Muscle *glu;
		HFL_Muscle *hfl;
		RF_Muscle *rf;
		
		// specific muscles : roll
		HAB_Muscle *hab;
		HAD_Muscle *had;
		EVE_Muscle *eve;
		INV_Muscle *inv;

		// specific muscles : yaw
		HER_Muscle *her;
		HIR_Muscle *hir;
};

#endif
