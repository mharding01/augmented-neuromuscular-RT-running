/*! 
 * \author Bruno Somers & Nicolas Van der Noot
 * \file SIR_Muscle.hh
 * \brief SIR_Muscle class
 */
#ifndef _SIR_MUSCLE_HH_
#define _SIR_MUSCLE_HH_

#include "Muscle.hh"

/*! \brief SIR Muscle class
 */
class SIR_Muscle : public Muscle
{
	public:
		SIR_Muscle(CtrlInputs *inputs, Articulation *yaw_shoulder, int body_part_id);
		virtual ~SIR_Muscle();
		
		virtual void rm_compute();
		virtual void lmtu_compute();
		virtual void torques_compute();

		double get_rm_ys() const { return rm_ys; }

	private:
		Articulation *yaw_shoulder; ///< yaw shoulder joint

		double rho_ys;     ///< rho for yaw shoulder [-]
		double r0_ys;      ///< r0 for yaw shoulder [m]
		double phi_ref_ys; ///< phi ref for yaw shoulder [rad]
		double phi_max_ys; ///< phi ref for yaw shoulder [rad]

		double rm_ys; ///< rm for shoulder [m]
};

#endif