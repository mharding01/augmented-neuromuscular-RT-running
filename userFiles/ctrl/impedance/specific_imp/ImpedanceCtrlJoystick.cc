#include "ImpedanceCtrlJoystick.hh"
#include "UserCtrl.hh"

/*! \brief constructor
 *
 * \param[in] inputs controller inputs
 * \param[in] options controller options
 * \param[in] ctrl_index controller index lists
 * \param[in] outputs controller outputs
 * \param[in] fwd_kin forward kinematics
 */
ImpedanceCtrlJoystick::ImpedanceCtrlJoystick(CtrlInputs *inputs, CtrlOptions *options, MotorCtrlIndex *ctrl_index, CtrlOutputs *outputs, ForwardKinematics *fwd_kin):
	ImpedanceCtrlStanding(inputs, options, ctrl_index, outputs, fwd_kin)
{
	// COM position
	r_COM_ref[0] = 0.0;
	r_COM_ref[1] = 0.0;
	r_COM_ref[2] = 0.56;

	// gains
	kp_x = 250.0;
	kp_y = 250.0;
	kp_z = 2200.0;

	kd_x = 100.0;
	kd_y = 100.0;
	kd_z = 50.0;

	ki_x = 0.0;
	ki_y = 0.0;
	ki_z = 0.0;
}

/*! \brief destructor
 */
ImpedanceCtrlJoystick::~ImpedanceCtrlJoystick()
{

}

/*! \brief main computation
 */
void ImpedanceCtrlJoystick::compute()
{
	UserCtrl *uCtrl;

	uCtrl = inputs->get_user_ctrl();

	// update COM reference position
	r_COM_ref[0] = 0.0  + 0.03*uCtrl->get_joystick_input(0);
	r_COM_ref[1] = 0.0  + 0.05*uCtrl->get_joystick_input(1);
	r_COM_ref[2] = 0.56 + 0.1*uCtrl->get_joystick_input(2);

	// main computation
	ImpedanceCtrl::compute();
}
