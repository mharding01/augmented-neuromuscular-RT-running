/*! 
 * \author Nicolas Van der Noot
 * \file Pitch_Knee_Art.hh
 * \brief Pitch_Knee_Art class
 */

#ifndef _PITCH_KNEE_ART_HH_
#define _PITCH_KNEE_ART_HH_

#include "Articulation.hh"

/*! \brief Pitch knee articulation
 */
class Pitch_Knee_Art : public Articulation
{
	public:
		Pitch_Knee_Art(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index, int leg_id);
		virtual ~Pitch_Knee_Art();

		virtual void add_Qq_soft_lim();
};

#endif
