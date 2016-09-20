/*! 
 * \author Nicolas Van der Noot
 * \file Pitch_Torso_Art.hh
 * \brief Pitch_Torso_Art class
 */

#ifndef _PITCH_TORSO_ART_HH_
#define _PITCH_TORSO_ART_HH_

#include "Articulation.hh"

/*! \brief Pitch foot articulation
 */
class Pitch_Torso_Art : public Articulation
{
	public:
		Pitch_Torso_Art(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index);
		virtual ~Pitch_Torso_Art();

		virtual void add_Qq_soft_lim();
};

#endif
