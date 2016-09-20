/*! 
 * \author Nicolas Van der Noot
 * \file HAM_Muscle.hh
 * \brief HAM_Muscle class
 */

#ifndef _HAM_MUSCLE_HH_
#define _HAM_MUSCLE_HH_

#include "Muscle.hh"

/*! \brief HAM Muscle class
 */
class HAM_Muscle : public Muscle
{
	public:
		HAM_Muscle(CtrlInputs *inputs, Articulation *pitch_knee, Articulation *pitch_hip, int body_part_id);
		virtual ~HAM_Muscle();
		
		virtual void rm_compute();
		virtual void lmtu_compute();
		virtual void torques_compute();

		double get_rm_pk() const { return rm_pk; }
		double get_rm_ph() const { return rm_ph; }

	private:
		Articulation *pitch_knee; ///< pitch knee joint
		Articulation *pitch_hip; ///< pitch hip joint

		double rho_pk;     ///< rho for pitch knee [-]
		double r0_pk;      ///< r0 for pitch knee [m]
		double phi_max_pk; ///< phi max for pitch knee [rad]
		double phi_ref_pk; ///< phi ref for pitch knee [rad]

		double rho_ph;     ///< rho for pitch hip [-]
		double r0_ph;      ///< r0 for pitch hip [m]
		double phi_ref_ph; ///< phi ref for pitch hip [rad]

		double rm_pk; ///< rm for knee [m]
		double rm_ph; ///< rm for hip [m]
};

#endif
