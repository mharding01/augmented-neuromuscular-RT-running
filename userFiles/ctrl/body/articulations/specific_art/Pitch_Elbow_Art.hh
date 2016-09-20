/*! 
 * \author Nicolas Van der Noot
 * \file Pitch_Elbow_Art.hh
 * \brief Pitch_Elbow_Art class
 */

#ifndef _PITCH_ELBOW_ART_HH_
#define _PITCH_ELBOW_ART_HH_

#include "Articulation.hh"

/*! \brief Pitch foot articulation
 */
class Pitch_Elbow_Art : public Articulation
{
	public:
		Pitch_Elbow_Art(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index, int body_part_id);
		virtual ~Pitch_Elbow_Art();

		virtual void add_Qq_soft_lim();
};

#endif
