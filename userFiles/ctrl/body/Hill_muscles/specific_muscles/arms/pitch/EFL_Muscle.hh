/*! 
 * \author Bruno Somers & Nicolas Van der Noot
 * \file EFL_Muscle.hh
 * \brief EFL_Muscle class
 */
#ifndef _EFL_MUSCLE_HH_
#define _EFL_MUSCLE_HH_

#include "Muscle.hh"

/*! \brief EFL Muscle class
 */
class EFL_Muscle : public Muscle
{
	public:
		EFL_Muscle(CtrlInputs *inputs, Articulation *pitch_elbow, int body_part_id);
		virtual ~EFL_Muscle();
		
		virtual void rm_compute();
		virtual void lmtu_compute();
		virtual void torques_compute();

		double get_rm_pe() const { return rm_pe; }

	private:
		Articulation *pitch_elbow; ///< pitch elbow joint

		double rho_pe;     ///< rho for pitch elbow [-]
		double r0_pe;      ///< r0 for pitch elbow [m]
		double phi_ref_pe; ///< phi ref for pitch elbow [rad]
		double phi_max_pe; ///< phi ref for pitch elbow [rad]

		double rm_pe; ///< rm for elbow [m]
};

#endif