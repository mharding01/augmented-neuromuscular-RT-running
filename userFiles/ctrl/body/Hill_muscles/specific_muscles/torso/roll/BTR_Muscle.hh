/*! 
 * \author Bruno Somers & Nicolas Van der Noot
 * \file BTR_Muscle.hh
 * \brief BTR_Muscle class
 */
#ifndef _BTR_MUSCLE_HH_
#define _BTR_MUSCLE_HH_

#include "Muscle.hh"

/*! \brief BTR Muscle class
 */
class BTR_Muscle : public Muscle
{
	public:
		BTR_Muscle(CtrlInputs *inputs, Articulation *roll_torso);
		virtual ~BTR_Muscle();
		
		virtual void rm_compute();
		virtual void lmtu_compute();
		virtual void torques_compute();

		double get_rm_rt() const { return rm_rt; }

	private:
		Articulation *roll_torso; ///< roll torso joint

		double rho_rt;     ///< rho for roll torso [-]
		double r0_rt;      ///< r0 for roll torso [m]
		double phi_ref_rt; ///< phi ref for roll torso [rad]
		double phi_max_rt; ///< phi ref for roll torso [rad]

		double rm_rt; ///< rm for torso [m]
};

#endif