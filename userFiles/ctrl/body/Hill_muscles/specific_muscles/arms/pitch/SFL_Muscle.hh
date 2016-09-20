/*! 
 * \author Bruno Somers & Nicolas Van der Noot
 * \file SFL_Muscle.hh
 * \brief SFL_Muscle class
 */
#ifndef _SFL_MUSCLE_HH_
#define _SFL_MUSCLE_HH_

#include "Muscle.hh"

/*! \brief SFL Muscle class
 */
class SFL_Muscle : public Muscle
{
	public:
		SFL_Muscle(CtrlInputs *inputs, Articulation *pitch_shoulder, int body_part_id);
		virtual ~SFL_Muscle();
		
		virtual void rm_compute();
		virtual void lmtu_compute();
		virtual void torques_compute();

		double get_rm_ps() const { return rm_ps; }

	private:
		Articulation *pitch_shoulder; ///< pitch shoulder joint

		double rho_ps;     ///< rho for pitch shoulder [-]
		double r0_ps;      ///< r0 for pitch shoulder [m]
		double phi_ref_ps; ///< phi ref for pitch shoulder [rad]
		double phi_max_ps; ///< phi ref for pitch shoulder [rad]

		double rm_ps; ///< rm for shoulder [m]
};

#endif