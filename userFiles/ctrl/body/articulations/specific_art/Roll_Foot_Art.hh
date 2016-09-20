/*! 
 * \author Nicolas Van der Noot
 * \file Roll_Foot_Art.hh
 * \brief Roll_Foot_Art class
 */

#ifndef _ROLL_FOOT_ART_HH_
#define _ROLL_FOOT_ART_HH_

#include "Articulation.hh"

/*! \brief Roll foot articulation
 */
class Roll_Foot_Art : public Articulation
{
	public:
		Roll_Foot_Art(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index, int leg_id);
		virtual ~Roll_Foot_Art();

		virtual void add_Qq_soft_lim();
};

#endif
