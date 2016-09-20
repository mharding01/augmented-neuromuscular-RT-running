
#include "TA_Muscle.hh"

/*! \brief constructor
 * 
 * \param[in] inputs inputs of the controller
 * \param[in] pitch_foot joint: pitch for foot
 * \param[in] body_part_id body part ID
 */
TA_Muscle::TA_Muscle(CtrlInputs *inputs, Articulation *pitch_foot, int body_part_id):
	Muscle(inputs, TA_MUSCLE, body_part_id, 0.026, 0.100, 285.0, 18.0, 0.07, 0.7)
{
	rho_pa     = 0.7;
	r0_pa      = 0.017; 
	phi_max_pa = -10.0 * DEG_TO_RAD;
	phi_ref_pa =  20.0 * DEG_TO_RAD;

	rm_pa = r0_pa;

	this->pitch_foot = pitch_foot;

	// safety
	if ((body_part_id != RIGHT_LEG_BODY) && (body_part_id != LEFT_LEG_BODY))
	{
		std::cout << "Muscle error: body part ID (" << body_part_id << ") is not correct for TA muscle !" << std::endl;
		exit(EXIT_FAILURE);
	}
}

/*! \brief destructor
 */
TA_Muscle::~TA_Muscle()
{

}

/*! \brief computation of rm
 */
void TA_Muscle::rm_compute()
{
	rm_pa = r0_pa * cos(pitch_foot->get_q() - phi_max_pa);
}

/*! \brief computation of l.mtu
 */
void TA_Muscle::lmtu_compute()
{
	double delta_lmtu_pa = rho_pa * r0_pa * (sin(pitch_foot->get_q() - phi_max_pa) - sin(phi_ref_pa - phi_max_pa));

	l.mtu = p.l_opt + p.l_slack + delta_lmtu_pa;
}

/*! \brief computation of the torques
 */
void TA_Muscle::torques_compute()
{
	pitch_foot->add_Qq(-rm_pa * F.m);
}
