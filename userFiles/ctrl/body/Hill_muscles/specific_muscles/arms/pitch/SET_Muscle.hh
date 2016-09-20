/*! 
 * \author Bruno Somers & Nicolas Van der Noot
 * \file SET_Muscle.hh
 * \brief SET_Muscle class
 */
#ifndef _SET_MUSCLE_HH_
#define _SET_MUSCLE_HH_

#include "Muscle.hh"

/*! \brief SET Muscle class
 */
class SET_Muscle : public Muscle
{
	public:
		SET_Muscle(CtrlInputs *inputs, Articulation *pitch_shoulder, int body_part_id);
		virtual ~SET_Muscle();
		
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