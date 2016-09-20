/*! 
 * \author Nicolas Van der Noot
 * \file EVE_Muscle.hh
 * \brief EVE_Muscle class
 */

#ifndef _EVE_MUSCLE_HH_
#define _EVE_MUSCLE_HH_

#include "Muscle.hh"

/*! \brief EVE Muscle class
 */
class EVE_Muscle : public Muscle
{
	public:
		EVE_Muscle(CtrlInputs *inputs, Articulation *roll_foot, int body_part_id);
		virtual ~EVE_Muscle();

		virtual void rm_compute();
		virtual void lmtu_compute();
		virtual void torques_compute();

		double get_rm_ra() const { return rm_ra; }

	private:
		Articulation *roll_foot; ///< pitch foot joint
		
		double rho_ra;     ///< rho for pitch foot [-]
		double r0_ra;      ///< r0 for pitch foot [m]
		double phi_max_ra; ///< phi max for pitch foot [rad]
		double phi_ref_ra; ///< phi ref for pitch foot [rad]

		double rm_ra; ///< rm for foot [m]
};

#endif
