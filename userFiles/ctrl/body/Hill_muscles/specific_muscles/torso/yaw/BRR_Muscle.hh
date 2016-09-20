/*! 
 * \author Bruno Somers & Nicolas Van der Noot
 * \file BRR_Muscle.hh
 * \brief BRR_Muscle class
 */
#ifndef _BRR_MUSCLE_HH_
#define _BRR_MUSCLE_HH_

#include "Muscle.hh"

/*! \brief BRR Muscle class
 */
class BRR_Muscle : public Muscle
{
	public:
		BRR_Muscle(CtrlInputs *inputs, Articulation *yaw_torso);
		virtual ~BRR_Muscle();
		
		virtual void rm_compute();
		virtual void lmtu_compute();
		virtual void torques_compute();

		double get_rm_yt() const { return rm_yt; }

	private:
		Articulation *yaw_torso; ///< yaw torso joint

		double rho_yt;     ///< rho for yaw torso [-]
		double r0_yt;      ///< r0 for yaw torso [m]
		double phi_ref_yt; ///< phi ref for yaw torso [rad]
		double phi_max_yt; ///< phi ref for yaw torso [rad]

		double rm_yt; ///< rm for torso [m]
};

#endif
