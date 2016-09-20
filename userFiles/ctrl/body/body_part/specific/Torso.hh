/*! 
 * \author Nicolas Van der Noot & Bruno Somers
 * \file Torso.hh
 * \brief Torso class
 */

#ifndef _TORSO_HH_
#define _TORSO_HH_

#include "BodyPart.hh"

// pitch muscles
#include "BFL_Muscle.hh"
#include "BET_Muscle.hh"

// roll muscles
#include "BTR_Muscle.hh"
#include "BTL_Muscle.hh"

// yaw muscles
#include "BRR_Muscle.hh"
#include "BRL_Muscle.hh"

/*! \brief torso
 */
class Torso: public BodyPart
{
	public:
		Torso(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index);
		virtual ~Torso();

		virtual void update_S_ref();

	private:
		// specific muscles : pitch
		BFL_Muscle *brf;
		BET_Muscle *brb;
		
		// specific muscles : roll
		BTR_Muscle *btr;
		BTL_Muscle *btl;

		// specific muscles : yaw
		BRR_Muscle *blr;
		BRL_Muscle *bll;
};

#endif
