/*! 
 * \author Nicolas Van der Noot
 * \file Yaw_Hip_Art.hh
 * \brief Yaw_Hip_Art class
 */

#ifndef _YAW_HIP_ART_HH_
#define _YAW_HIP_ART_HH_

#include "Articulation.hh"

/*! \brief Yaw hip articulation
 */
class Yaw_Hip_Art : public Articulation
{
	public:
		Yaw_Hip_Art(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index, int leg_id);
		virtual ~Yaw_Hip_Art();

		virtual void add_Qq_soft_lim();	
};

#endif
