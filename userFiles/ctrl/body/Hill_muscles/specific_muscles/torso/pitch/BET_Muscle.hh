/*! 
 * \author Bruno Somers & Nicolas Van der Noot
 * \file BET_Muscle.hh
 * \brief BET_Muscle class
 */
#ifndef _BET_MUSCLE_HH_
#define _BET_MUSCLE_HH_

#include "Muscle.hh"

/*! \brief BET Muscle class
 */
class BET_Muscle : public Muscle
{
	public:
		BET_Muscle(CtrlInputs *inputs, Articulation *pitch_torso);
		virtual ~BET_Muscle();
		
		virtual void rm_compute();
		virtual void lmtu_compute();
		virtual void torques_compute();

		double get_rm_pt() const { return rm_pt; }

	private:
		Articulation *pitch_torso; ///< pitch torso joint

		double rho_pt;     ///< rho for pitch torso [-]
		double r0_pt;      ///< r0 for pitch torso [m]
		double phi_ref_pt; ///< phi ref for pitch torso [rad]
		double phi_max_pt; ///< phi ref for pitch torso [rad]

		double rm_pt; ///< rm for torso [m]
};

#endif