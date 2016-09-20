#include "BRR_Muscle.hh"

/*! \brief constructor
 * 
 * \param[in] inputs inputs of the controller
 * \param[in] yaw_torso joint: yaw for torso
 * \param[in] body_part_id body part ID
 */
BRR_Muscle::BRR_Muscle(CtrlInputs *inputs, Articulation *yaw_torso):
	Muscle(inputs, BRR_MUSCLE, TORSO_BODY, 0.047, 0.045, 560.0, 18.0, 0.26, 0.51) 
{
	rho_yt = 1.0;
	r0_yt  = 0.015;

	phi_ref_yt =  20.0 * DEG_TO_RAD;
	phi_max_yt = -35.0 * DEG_TO_RAD;

	rm_yt = r0_yt;

	this->yaw_torso = yaw_torso;
}

/*! \brief destructor
 */
BRR_Muscle::~BRR_Muscle()
{

}

/*! \brief computation of rm
 */
void BRR_Muscle::rm_compute()
{
	rm_yt = r0_yt * cos(yaw_torso->get_q() - phi_max_yt);
}

/*! \brief computation of l.mtu
 */
void BRR_Muscle::lmtu_compute()
{
	double delta_lmtu_yt = rho_yt * r0_yt * (sin(yaw_torso->get_q() - phi_max_yt) - sin(phi_ref_yt - phi_max_yt));

	l.mtu = p.l_opt + p.l_slack + delta_lmtu_yt;
}

/*! \brief computation of the torques
 */
void BRR_Muscle::torques_compute()
{
	yaw_torso->add_Qq(-rm_yt * F.m);
}
