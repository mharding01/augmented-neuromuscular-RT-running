/*! 
 * \author Nicolas Van der Noot
 * \file Yaw_Torso_Art.hh
 * \brief Yaw_Torso_Art class
 */

#ifndef _YAW_TORSO_ART_HH_
#define _YAW_TORSO_ART_HH_

#include "Articulation.hh"

/*! \brief Yaw foot articulation
 */
class Yaw_Torso_Art : public Articulation
{
	public:
		Yaw_Torso_Art(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index);
		virtual ~Yaw_Torso_Art();

		virtual void add_Qq_soft_lim();
};

#endif
