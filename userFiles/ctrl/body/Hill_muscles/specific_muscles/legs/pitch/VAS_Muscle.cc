
#include "VAS_Muscle.hh"

/*! \brief constructor
 * 
 * \param[in] inputs inputs of the controller
 * \param[in] pitch_knee joint: pitch for knee
 * \param[in] body_part_id body part ID
 */
VAS_Muscle::VAS_Muscle(CtrlInputs *inputs, Articulation *pitch_knee, int body_part_id):
	Muscle(inputs, VAS_MUSCLE, body_part_id, 0.034, 0.098, 2125.0, 18.0, 0.72, 0.5)
{
	rho_pk     = 0.7;
	r0_pk      = 0.026; 
	phi_max_pk = 15.0 * DEG_TO_RAD;
	phi_ref_pk = 55.0 * DEG_TO_RAD;

	rm_pk = r0_pk;

	this->pitch_knee = pitch_knee;

	// safety
	if ((body_part_id != RIGHT_LEG_BODY) && (body_part_id != LEFT_LEG_BODY))
	{
		std::cout << "Muscle error: body part ID (" << body_part_id << ") is not correct for VAS muscle !" << std::endl;
		exit(EXIT_FAILURE);
	}
}

/*! \brief destructor
 */
VAS_Muscle::~VAS_Muscle()
{

}

/*! \brief computation of rm
 */
void VAS_Muscle::rm_compute()
{
	rm_pk = r0_pk * cos(pitch_knee->get_q() - phi_max_pk);    
}

/*! \brief computation of l.mtu
 */
void VAS_Muscle::lmtu_compute()
{
	double delta_lmtu_pk = rho_pk * r0_pk * (sin(pitch_knee->get_q() - phi_max_pk) - sin(phi_ref_pk - phi_max_pk));

	l.mtu = p.l_opt + p.l_slack + delta_lmtu_pk;
}

/*! \brief computation of the torques
 */
void VAS_Muscle::torques_compute()
{
	pitch_knee->add_Qq(-rm_pk * F.m);
}
