/*! 
 * \author Nicolas Van der Noot & Bruno Somers
 * \file Arm.hh
 * \brief Arm class
 */

#ifndef _ARM_HH_
#define _ARM_HH_

#include "BodyPart.hh"

// pitch muscles
#include "EET_Muscle.hh"
#include "EFL_Muscle.hh"
#include "SFL_Muscle.hh"
#include "SET_Muscle.hh"

// roll muscles
#include "SAB_Muscle.hh"
#include "SAD_Muscle.hh"

// yaw muscles
#include "SER_Muscle.hh"
#include "SIR_Muscle.hh"

/*! \brief arm
 */
class Arm: public BodyPart
{
	public:
		Arm(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index, int body_part_id);
		virtual ~Arm();

		virtual void update_S_ref();

	private:
		// specific muscles : pitch
		EET_Muscle *esb;
		EFL_Muscle *esf;
		SFL_Muscle *see;
		SET_Muscle *sei;
		
		// specific muscles : roll
		SAB_Muscle *sae;
		SAD_Muscle *sai;

		// specific muscles : yaw
		SER_Muscle *srr;
		SIR_Muscle *srl;
};

#endif
