#include "BTR_Muscle.hh"

/*! \brief constructor
 * 
 * \param[in] inputs inputs of the controller
 * \param[in] roll_torso joint: roll for torso
 * \param[in] body_part_id body part ID
 */
BTR_Muscle::BTR_Muscle(CtrlInputs *inputs, Articulation *roll_torso):
	Muscle(inputs, BTR_MUSCLE, TORSO_BODY, 0.043, 0.043, 250.0, 18.0, 0.11, 0.5) 
{
	rho_rt = 1.0;
	r0_rt  = 0.049;

	phi_ref_rt =  0.0 * DEG_TO_RAD;
	phi_max_rt = 45.0 * DEG_TO_RAD;

	rm_rt = r0_rt;

	this->roll_torso = roll_torso;
}

/*! \brief destructor
 */
BTR_Muscle::~BTR_Muscle()
{

}

/*! \brief computation of rm
 */
void BTR_Muscle::rm_compute()
{
	rm_rt = r0_rt * cos(roll_torso->get_q() - phi_max_rt);
}

/*! \brief computation of l.mtu
 */
void BTR_Muscle::lmtu_compute()
{
	double delta_lmtu_rt = -rho_rt * r0_rt * (sin(roll_torso->get_q() - phi_max_rt) - sin(phi_ref_rt - phi_max_rt));

	l.mtu = p.l_opt + p.l_slack + delta_lmtu_rt;
}

/*! \brief computation of the torques
 */
void BTR_Muscle::torques_compute()
{
	roll_torso->add_Qq(rm_rt * F.m);
}