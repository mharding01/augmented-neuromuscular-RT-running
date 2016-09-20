#include "SET_Muscle.hh"

/*! \brief constructor
 * 
 * \param[in] inputs inputs of the controller
 * \param[in] pitch_shoulder joint: pitch for shoulder
 * \param[in] body_part_id body part ID
 */
SET_Muscle::SET_Muscle(CtrlInputs *inputs, Articulation *pitch_shoulder, int body_part_id):
	Muscle(inputs, SET_MUSCLE, body_part_id, 0.059, 0.038, 180.0, 18.0, 0.11, 0.42) 
{
	rho_ps     = 0.6;
	r0_ps      = 0.018; 
	phi_ref_ps = -120.0 * DEG_TO_RAD;
	phi_max_ps = -70.0 * DEG_TO_RAD;

	rm_ps = r0_ps;

	this->pitch_shoulder = pitch_shoulder;

	// safety
	if ((body_part_id != RIGHT_ARM_BODY) && (body_part_id != LEFT_ARM_BODY))
	{
		std::cout << "Muscle error: body part ID (" << body_part_id << ") is not correct for SEI muscle !" << std::endl;
		exit(EXIT_FAILURE);
	}
}

/*! \brief destructor
 */
SET_Muscle::~SET_Muscle()
{

}

/*! \brief computation of rm
 */
void SET_Muscle::rm_compute()
{
	rm_ps = r0_ps * cos(pitch_shoulder->get_q() - phi_max_ps);
}

/*! \brief computation of l.mtu
 */
void SET_Muscle::lmtu_compute()
{
	double delta_lmtu_ps = -rho_ps * r0_ps * (sin(pitch_shoulder->get_q() - phi_max_ps) - sin(phi_ref_ps - phi_max_ps));

	l.mtu = p.l_opt + p.l_slack + delta_lmtu_ps;
}

/*! \brief computation of the torques
 */
void SET_Muscle::torques_compute()
{
	pitch_shoulder->add_Qq(rm_ps * F.m);
}