#include "BFL_Muscle.hh"

/*! \brief constructor
 * 
 * \param[in] inputs inputs of the controller
 * \param[in] pitch_torso joint: pitch for torso
 * \param[in] body_part_id body part ID
 */
BFL_Muscle::BFL_Muscle(CtrlInputs *inputs, Articulation *pitch_torso):
	Muscle(inputs, BFL_MUSCLE, TORSO_BODY, 0.048, 0.053, 830.0, 18.0, 0.39, 0.5) 
{
	rho_pt     = 1.0;
	r0_pt      = 0.035; 
	phi_ref_pt = 5.0 * DEG_TO_RAD;
	phi_max_pt = 40.0 * DEG_TO_RAD;

	rm_pt = r0_pt;

	this->pitch_torso = pitch_torso;
}

/*! \brief destructor
 */
BFL_Muscle::~BFL_Muscle()
{

}

/*! \brief computation of rm
 */
void BFL_Muscle::rm_compute()
{
	rm_pt = r0_pt * cos(pitch_torso->get_q() - phi_max_pt);
}

/*! \brief computation of l.mtu
 */
void BFL_Muscle::lmtu_compute()
{
	double delta_lmtu_pt = -rho_pt * r0_pt * (sin(pitch_torso->get_q() - phi_max_pt) - sin(phi_ref_pt - phi_max_pt));

	l.mtu = p.l_opt + p.l_slack + delta_lmtu_pt;
}

/*! \brief computation of the torques
 */
void BFL_Muscle::torques_compute()
{
	pitch_torso->add_Qq(rm_pt * F.m);
}