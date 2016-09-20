/*! 
 * \author Nicolas Van der Noot
 * \file Roll_Shoulder_Art.hh
 * \brief Roll_Shoulder_Art class
 */

#ifndef _ROLL_SHOULDER_ART_HH_
#define _ROLL_SHOULDER_ART_HH_

#include "Articulation.hh"

/*! \brief Roll foot articulation
 */
class Roll_Shoulder_Art : public Articulation
{
	public:
		Roll_Shoulder_Art(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index, int body_part_id);
		virtual ~Roll_Shoulder_Art();

		virtual void add_Qq_soft_lim();
};

#endif
