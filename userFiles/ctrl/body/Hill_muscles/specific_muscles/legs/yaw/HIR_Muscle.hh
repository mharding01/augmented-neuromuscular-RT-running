/*! 
 * \author Nicolas Van der Noot
 * \file HIR_Muscle.hh
 * \brief HIR_Muscle class
 */
#ifndef _HIR_MUSCLE_HH_
#define _HIR_MUSCLE_HH_

#include "Muscle.hh"

/*! \brief HIR Muscle class
 */
class HIR_Muscle : public Muscle
{
	public:
		HIR_Muscle(CtrlInputs *inputs, Articulation *yaw_hip, int body_part_id);
		virtual ~HIR_Muscle();
		
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
