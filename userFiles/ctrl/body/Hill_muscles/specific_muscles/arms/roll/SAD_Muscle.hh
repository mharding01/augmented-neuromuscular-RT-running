/*! 
 * \author Bruno Somers & Nicolas Van der Noot
 * \file SAD_Muscle.hh
 * \brief SAD_Muscle class
 */
#ifndef _SAD_MUSCLE_HH_
#define _SAD_MUSCLE_HH_

#include "Muscle.hh"

/*! \brief SAD Muscle class
 */
class SAD_Muscle : public Muscle
{
	public:
		SAD_Muscle(CtrlInputs *inputs, Articulation *roll_shoulder, int body_part_id);
		virtual ~SAD_Muscle();
		
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