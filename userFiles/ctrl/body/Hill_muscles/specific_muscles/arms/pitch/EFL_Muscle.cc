#include "EFL_Muscle.hh"

/*! \brief constructor
 * 
 * \param[in] inputs inputs of the controller
 * \param[in] pitch_elbow joint: pitch for elbow
 * \param[in] body_part_id body part ID
 */
EFL_Muscle::EFL_Muscle(CtrlInputs *inputs, Articulation *pitch_elbow, int body_part_id):
	Muscle(inputs, EFL_MUSCLE, body_part_id, 0.05, 0.072, 390.0, 18.0, 0.19, 0.46) 
{
	rho_pe     = 1.0;
	r0_pe      = 0.016; 
	phi_ref_pe = -60.0 * DEG_TO_RAD;
	phi_max_pe = -70.0 * DEG_TO_RAD;

	rm_pe = r0_pe;

	this->pitch_elbow = pitch_elbow;

	// safety
	if ((body_part_id != RIGHT_ARM_BODY) && (body_part_id != LEFT_ARM_BODY))
	{
		std::cout << "Muscle error: body part ID (" << body_part_id << ") is not correct for ESF muscle !" << std::endl;
		exit(EXIT_FAILURE);
	}
}

/*! \brief destructor
 */
EFL_Muscle::~EFL_Muscle()
{

}

/*! \brief computation of rm
 */
void EFL_Muscle::rm_compute()
{
	rm_pe = r0_pe * cos(pitch_elbow->get_q() - phi_max_pe);
}

/*! \brief computation of l.mtu
 */
void EFL_Muscle::lmtu_compute()
{
	double delta_lmtu_pe = rho_pe * r0_pe * (sin(pitch_elbow->get_q() - phi_max_pe) - sin(phi_ref_pe - phi_max_pe));

	l.mtu = p.l_opt + p.l_slack + delta_lmtu_pe;
}

/*! \brief computation of the torques
 */
void EFL_Muscle::torques_compute()
{
	pitch_elbow->add_Qq(-rm_pe * F.m);
}