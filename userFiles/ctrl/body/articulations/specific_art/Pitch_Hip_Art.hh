/*! 
 * \author Nicolas Van der Noot
 * \file Pitch_Hip_Art.hh
 * \brief Pitch_Hip_Art class
 */

#ifndef _PITCH_HIP_ART_HH_
#define _PITCH_HIP_ART_HH_

#include "Articulation.hh"

/*! \brief Pitch hip articulation
 */
class Pitch_Hip_Art : public Articulation
{
	public:
		Pitch_Hip_Art(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index, int leg_id);
		virtual ~Pitch_Hip_Art();

		virtual void add_Qq_soft_lim();
};

#endif
