/*! 
 * \author Bruno Somers & Nicolas Van der Noot
 * \file SER_Muscle.hh
 * \brief SER_Muscle class
 */
#ifndef _SER_MUSCLE_HH_
#define _SER_MUSCLE_HH_

#include "Muscle.hh"

/*! \brief SER Muscle class
 */
class SER_Muscle : public Muscle
{
	public:
		SER_Muscle(CtrlInputs *inputs, Articulation *yaw_shoulder, int body_part_id);
		virtual ~SER_Muscle();
		
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