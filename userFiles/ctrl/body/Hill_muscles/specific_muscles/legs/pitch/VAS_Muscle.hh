/*! 
 * \author Nicolas Van der Noot
 * \file VAS_Muscle.hh
 * \brief VAS_Muscle class
 */

#ifndef _VAS_MUSCLE_HH_
#define _VAS_MUSCLE_HH_

#include "Muscle.hh"

/*! \brief VAS Muscle class
 */
class VAS_Muscle : public Muscle
{
	public:
		VAS_Muscle(CtrlInputs *inputs, Articulation *pitch_knee, int body_part_id);
		virtual ~VAS_Muscle();
		
		virtual void rm_compute();
		virtual void lmtu_compute();
		virtual void torques_compute();

		double get_rm_pk() const { return rm_pk; }

	private:
		Articulation *pitch_knee; ///< pitch knee joint

		double rho_pk;     ///< rho for pitch knee [-]
		double r0_pk;      ///< r0 for pitch knee [m]
		double phi_max_pk; ///< phi max for pitch knee [rad]
		double phi_ref_pk; ///< phi ref for pitch knee [rad]

		double rm_pk; ///< rm for knee [m]
};

#endif
