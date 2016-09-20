/*! 
 * \author Bruno Somers & Nicolas Van der Noot
 * \file SAB_Muscle.hh
 * \brief SAB_Muscle class
 */
#ifndef _SAB_MUSCLE_HH_
#define _SAB_MUSCLE_HH_

#include "Muscle.hh"

/*! \brief SAB Muscle class
 */
class SAB_Muscle : public Muscle
{
	public:
		SAB_Muscle(CtrlInputs *inputs, Articulation *roll_shoulder, int body_part_id);
		virtual ~SAB_Muscle();
		
		virtual void rm_compute();
		virtual void lmtu_compute();
		virtual void torques_compute();

		double get_rm_rs() const { return rm_rs; }

	private:
		Articulation *roll_shoulder; ///< roll shoulder joint

		double rho_rs;     ///< rho for roll shoulder [-]
		double r0_rs;      ///< r0 for roll shoulder [m]
		double phi_ref_rs; ///< phi ref for roll shoulder [rad]
		double phi_max_rs; ///< phi ref for roll shoulder [rad]

		double rm_rs; ///< rm for shoulder [m]
};

#endif