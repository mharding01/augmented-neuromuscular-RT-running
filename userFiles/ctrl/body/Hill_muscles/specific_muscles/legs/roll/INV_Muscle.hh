/*! 
 * \author Nicolas Van der Noot
 * \file INV_Muscle.hh
 * \brief INV_Muscle class
 */

#ifndef _INV_MUSCLE_HH_
#define _INV_MUSCLE_HH_

#include "Muscle.hh"

/*! \brief INV Muscle class
 */
class INV_Muscle : public Muscle
{
	public:
		INV_Muscle(CtrlInputs *inputs, Articulation *roll_foot, int body_part_id);
		virtual ~INV_Muscle();

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
