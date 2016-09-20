/*! 
 * \author Nicolas Van der Noot
 * \file GAS_Muscle.hh
 * \brief GAS_Muscle class
 * 
 * Details
 */

#ifndef _GAS_MUSCLE_HH_
#define _GAS_MUSCLE_HH_

#include "Muscle.hh"

/*! \brief GAS Muscle class
 */
class GAS_Muscle : public Muscle
{
	public:
		GAS_Muscle(CtrlInputs *inputs, Articulation *pitch_foot, Articulation *pitch_knee, int body_part_id);
		virtual ~GAS_Muscle();

		virtual void rm_compute();
		virtual void lmtu_compute();
		virtual void torques_compute();

		double get_rm_pa() const { return rm_pa; }
		double get_rm_pk() const { return rm_pk; }

	private:
		Articulation *pitch_foot; ///< pitch foot joint
		Articulation *pitch_knee; ///< pitch knee joint

		double rho_pa;     ///< rho for pitch foot [-]
		double r0_pa;      ///< r0 for pitch foot [m]
		double phi_max_pa; ///< phi max for pitch foot [rad]
		double phi_ref_pa; ///< phi ref for pitch foot [rad]

		double rho_pk;     ///< rho for pitch knee [-]
		double r0_pk;      ///< r0 for pitch knee [m]
		double phi_max_pk; ///< phi max for pitch knee [rad]
		double phi_ref_pk; ///< phi ref for pitch knee [rad]

		double rm_pa; ///< rm for foot [m]
		double rm_pk; ///< rm for knee [m]
};

#endif
