/*! 
 * \author Nicolas Van der Noot
 * \file ImpedanceCtrlJoystick.hh
 * \brief ImpedanceCtrlJoystick class
 */

#ifndef _IMPEDANCE_CTRL_JOYSTICK_HH_
#define _IMPEDANCE_CTRL_JOYSTICK_HH_

#include "ImpedanceCtrlStanding.hh"

/*! \brief impedance controller to be used with the joystick
 */
class ImpedanceCtrlJoystick: public ImpedanceCtrlStanding
{
	public:
		ImpedanceCtrlJoystick(CtrlInputs *inputs, CtrlOptions *options, MotorCtrlIndex *ctrl_index, CtrlOutputs *outputs, ForwardKinematics *fwd_kin);
		virtual ~ImpedanceCtrlJoystick();

		virtual void compute();
};

#endif
