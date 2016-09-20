/*! 
 * \author Nicolas Van der Noot
 * \file SOL_Muscle.hh
 * \brief SOL_Muscle class
 */

#ifndef _SOL_MUSCLE_HH_
#define _SOL_MUSCLE_HH_

#include "Muscle.hh"

/*! \brief SOL Muscle class
 */
class SOL_Muscle : public Muscle
{
	public:
		SOL_Muscle(CtrlInputs *inputs, Articulation *pitch_foot, int body_part_id);
		virtual ~SOL_Muscle();

		virtual void rm_compute();
		virtual void lmtu_compute();
		virtual void torques_compute();

		double get_rm_pa() const { return rm_pa; }

	private:
		Articulation *pitch_foot; ///< pitch foot joint
		
		double rho_pa;     ///< rho for pitch foot [-]
		double r0_pa;      ///< r0 for pitch foot [m]
		double phi_max_pa; ///< phi max for pitch foot [rad]
		double phi_ref_pa; ///< phi ref for pitch foot [rad]

		double rm_pa; ///< rm for foot [m]
};

#endif
