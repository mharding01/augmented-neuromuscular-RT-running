/*! 
 * \author Nicolas Van der Noot
 * \file HAB_Muscle.hh
 * \brief HAB_Muscle class
 */
#ifndef _HAB_MUSCLE_HH_
#define _HAB_MUSCLE_HH_

#include "Muscle.hh"

/*! \brief HAB Muscle class
 */
class HAB_Muscle : public Muscle
{
	public:
		HAB_Muscle(CtrlInputs *inputs, Articulation *roll_hip, int body_part_id);
		virtual ~HAB_Muscle();
		
		virtual void rm_compute();
		virtual void lmtu_compute();
		virtual void torques_compute();

		double get_rm_rh() const { return rm_rh; }

	private:
		Articulation *roll_hip; ///< pitch hip joint

		double rho_rh;     ///< rho for pitch hip [-]
		double r0_rh;      ///< r0 for pitch hip [m]
		double phi_ref_rh; ///< phi ref for pitch hip [rad]

		double rm_rh; ///< rm for hip [m]
};

#endif
