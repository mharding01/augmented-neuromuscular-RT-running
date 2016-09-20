#include "EET_Muscle.hh"

/*! \brief constructor
 * 
 * \param[in] inputs inputs of the controller
 * \param[in] pitch_elbow joint: pitch for elbow
 * \param[in] body_part_id body part ID
 */
EET_Muscle::EET_Muscle(CtrlInputs *inputs, Articulation *pitch_elbow, int body_part_id):
	Muscle(inputs, EET_MUSCLE, body_part_id, 0.053, 0.051, 460.0, 18.0, 0.24, 0.32) 
{
	rho_pe     = 0.8;
	r0_pe      = 0.01; 
	phi_ref_pe = -60.0 * DEG_TO_RAD;
	phi_max_pe = -25.0 * DEG_TO_RAD;

	rm_pe = r0_pe;

	this->pitch_elbow = pitch_elbow;

	// safety
	if ((body_part_id != RIGHT_ARM_BODY) && (body_part_id != LEFT_ARM_BODY))
	{
		std::cout << "Muscle error: body part ID (" << body_part_id << ") is not correct for ESB muscle !" << std::endl;
		exit(EXIT_FAILURE);
	}
}

/*! \brief destructor
 */
EET_Muscle::~EET_Muscle()
{

}

/*! \brief computation of rm
 */
void EET_Muscle::rm_compute()
{
	rm_pe = r0_pe * cos(pitch_elbow->get_q() - phi_max_pe);
}

/*! \brief computation of l.mtu
 */
void EET_Muscle::lmtu_compute()
{
	double delta_lmtu_pe = -rho_pe * r0_pe * (sin(pitch_elbow->get_q() - phi_max_pe) - sin(phi_ref_pe - phi_max_pe));

	l.mtu = p.l_opt + p.l_slack + delta_lmtu_pe;
}

/*! \brief computation of the torques
 */
void EET_Muscle::torques_compute()
{
	pitch_elbow->add_Qq(rm_pe * F.m);
}