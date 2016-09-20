/*! 
 * \author Nicolas Van der Noot
 * \file Roll_Hip_Art.hh
 * \brief Roll_Hip_Art class
 */

#ifndef _ROLL_HIP_ART_HH_
#define _ROLL_HIP_ART_HH_

#include "Articulation.hh"

/*! \brief Roll hip articulation
 */
class Roll_Hip_Art : public Articulation
{
	public:
		Roll_Hip_Art(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index, int leg_id);
		virtual ~Roll_Hip_Art();

		virtual void add_Qq_soft_lim();	
};

#endif
