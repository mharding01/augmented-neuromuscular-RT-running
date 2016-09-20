/*! 
 * \author Nicolas Van der Noot
 * \file ImpedanceCtrlInit.hh
 * \brief ImpedanceCtrlInit class
 */

#ifndef _IMPEDANCE_CTRL_INIT_HH_
#define _IMPEDANCE_CTRL_INIT_HH_

#include "ImpedanceCtrlStanding.hh"

/*! \brief impedance controller for walk initiation
 */
class ImpedanceCtrlInit: public ImpedanceCtrlStanding
{
	public:
		ImpedanceCtrlInit(CtrlInputs *inputs, CtrlOptions *options, MotorCtrlIndex *ctrl_index, CtrlOutputs *outputs, ForwardKinematics *fwd_kin);
		virtual ~ImpedanceCtrlInit();

		virtual void compute();

		void set_x_ref_end(double value) { x_ref_end = value; }
		void set_y_ref_end(double value) { y_ref_end = value; }

	private:
		double t; ///< time [s]

		int r_first_swing; ///< 1 if right leg first in swing, 0 otherwise
		int flag_3D; ///< 1 for 3D walking, 0 otherwise

		double init_t; ///< initial time

		double x_com_foot_ref; ///< COM x position reference relative to the foot [m]
		double y_com_foot_ref; ///< COM y position reference relative to the foot [m]

		double x_ref_init; ///< initial value of 'x_com_foot_ref' [m]
		double y_ref_init; ///< initial value of 'y_com_foot_ref' [m]

		double x_ref_end; ///< final value of 'x_com_foot_ref' [m]
		double y_ref_end; ///< final value of 'y_com_foot_ref' [m]

		int flag_first; ///< 1 if first call, 0 therwise
};

#endif
