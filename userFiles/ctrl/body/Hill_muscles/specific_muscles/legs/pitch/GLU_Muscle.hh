/*! 
 * \author Nicolas Van der Noot
 * \file GLU_Muscle.hh
 * \brief GLU_Muscle class
 */

#ifndef _GLU_MUSCLE_HH_
#define _GLU_MUSCLE_HH_

#include "Muscle.hh"

/*! \brief GLU Muscle class
 */
class GLU_Muscle : public Muscle
{
	public:
		GLU_Muscle(CtrlInputs *inputs, Articulation *pitch_hip, int body_part_id);
		virtual ~GLU_Muscle();
		
		virtual void rm_compute();
		virtual void lmtu_compute();
		virtual void torques_compute();

		double get_rm_ph() const { return rm_ph; }

	private:
		Articulation *pitch_hip; ///< pitch hip joint

		double rho_ph;     ///< rho for pitch hip [-]
		double r0_ph;      ///< r0 for pitch hip [m]
		double phi_ref_ph; ///< phi ref for pitch hip [rad]

		double rm_ph; ///< rm for hip [m]
};

#endif
