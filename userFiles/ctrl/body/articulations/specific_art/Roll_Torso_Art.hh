/*! 
 * \author Nicolas Van der Noot
 * \file Roll_Torso_Art.hh
 * \brief Roll_Torso_Art class
 */

#ifndef _ROLL_TORSO_ART_HH_
#define _ROLL_TORSO_ART_HH_

#include "Articulation.hh"

/*! \brief Roll foot articulation
 */
class Roll_Torso_Art : public Articulation
{
	public:
		Roll_Torso_Art(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index);
		virtual ~Roll_Torso_Art();

		virtual void add_Qq_soft_lim();
};

#endif
