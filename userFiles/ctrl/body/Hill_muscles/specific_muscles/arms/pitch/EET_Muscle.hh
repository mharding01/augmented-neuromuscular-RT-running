/*! 
 * \author Bruno Somers & Nicolas Van der Noot
 * \file EET_Muscle.hh
 * \brief EET_Muscle class
 */
#ifndef _EET_MUSCLE_HH_
#define _EET_MUSCLE_HH_

#include "Muscle.hh"

/*! \brief EET Muscle class
 */
class EET_Muscle : public Muscle
{
	public:
		EET_Muscle(CtrlInputs *inputs, Articulation *pitch_elbow, int body_part_id);
		virtual ~EET_Muscle();
		
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