/*! 
 * \author Nicolas Van der Noot
 * \file HER_Muscle.hh
 * \brief HER_Muscle class
 */
#ifndef _HER_MUSCLE_HH_
#define _HER_MUSCLE_HH_

#include "Muscle.hh"

/*! \brief HER Muscle class
 */
class HER_Muscle : public Muscle
{
	public:
		HER_Muscle(CtrlInputs *inputs, Articulation *yaw_hip, int body_part_id);
		virtual ~HER_Muscle();
		
		virtual void rm_compute();
		virtual void lmtu_compute();
		virtual void torques_compute();

		double get_rm_yh() const { return rm_yh; }

	private:
		Articulation *yaw_hip; ///< pitch hip joint

		double rho_yh;     ///< rho for pitch hip [-]
		double r0_yh;      ///< r0 for pitch hip [m]
		double phi_ref_yh; ///< phi ref for pitch hip [rad]

		double rm_yh; ///< rm for hip [m]
};

#endif
