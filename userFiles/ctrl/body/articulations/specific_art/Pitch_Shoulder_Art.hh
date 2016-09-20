/*! 
 * \author Nicolas Van der Noot
 * \file Pitch_Shoulder_Art.hh
 * \brief Pitch_Shoulder_Art class
 */

#ifndef _PITCH_SHOULDER_ART_HH_
#define _PITCH_SHOULDER_ART_HH_

#include "Articulation.hh"

/*! \brief Pitch foot articulation
 */
class Pitch_Shoulder_Art : public Articulation
{
	public:
		Pitch_Shoulder_Art(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index, int body_part_id);
		virtual ~Pitch_Shoulder_Art();

		virtual void add_Qq_soft_lim();
};

#endif
