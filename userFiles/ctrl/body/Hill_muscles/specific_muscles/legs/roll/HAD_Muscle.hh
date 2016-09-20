/*! 
 * \author Nicolas Van der Noot
 * \file HAD_Muscle.hh
 * \brief HAD_Muscle class
 */
#ifndef _HAD_MUSCLE_HH_
#define _HAD_MUSCLE_HH_

#include "Muscle.hh"

/*! \brief HAD Muscle class
 */
class HAD_Muscle : public Muscle
{
	public:
		HAD_Muscle(CtrlInputs *inputs, Articulation *roll_hip, int body_part_id);
		virtual ~HAD_Muscle();
		
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
