/*! 
 * \author Nicolas Van der Noot
 * \file HFL_Muscle.hh
 * \brief HFL_Muscle class
 */
#ifndef _HFL_MUSCLE_HH_
#define _HFL_MUSCLE_HH_

#include "Muscle.hh"

/*! \brief HFL Muscle class
 */
class HFL_Muscle : public Muscle
{
	public:
		HFL_Muscle(CtrlInputs *inputs, Articulation *pitch_hip, int body_part_id);
		virtual ~HFL_Muscle();
		
		virtual void rm_compute();
		virtual void lmtu_compute();
		virtual void torques_compute();

		double get_rm_ph() const { return rm_ph; }

	private:
		Articulation *pitch_hip; ///< pitch hip joint

		double rho_ph;     ///< rho for pitch hip [-]
		double r0_ph;      ///< r0 for pitch hip [m]
		double phi_ref_ph; ///< phi ref for pitch hip [rad]

		double rm_ph; ///< rm for hip [m]
};

#endif
