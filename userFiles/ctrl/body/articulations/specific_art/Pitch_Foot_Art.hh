/*! 
 * \author Nicolas Van der Noot
 * \file Pitch_Foot_Art.hh
 * \brief Pitch_Foot_Art class
 */

#ifndef _PITCH_FOOT_ART_HH_
#define _PITCH_FOOT_ART_HH_

#include "Articulation.hh"

/*! \brief Pitch foot articulation
 */
class Pitch_Foot_Art : public Articulation
{
	public:
		Pitch_Foot_Art(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index, int leg_id);
		virtual ~Pitch_Foot_Art();

		virtual void add_Qq_soft_lim();
};

#endif
