/*! 
 * \author Nicolas Van der Noot
 * \file Yaw_Shoulder_Art.hh
 * \brief Yaw_Shoulder_Art class
 */

#ifndef _YAW_SHOULDER_ART_HH_
#define _YAW_SHOULDER_ART_HH_

#include "Articulation.hh"

/*! \brief Yaw foot articulation
 */
class Yaw_Shoulder_Art : public Articulation
{
	public:
		Yaw_Shoulder_Art(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index, int body_part_id);
		virtual ~Yaw_Shoulder_Art();

		virtual void add_Qq_soft_lim();
};

#endif
