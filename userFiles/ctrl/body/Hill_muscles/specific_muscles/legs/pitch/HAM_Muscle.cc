
#include "HAM_Muscle.hh"

/*! \brief constructor
 * 
 * \param[in] inputs inputs of the controller
 * \param[in] pitch_knee joint: pitch for knee
 * \param[in] pitch_hip joint: pitch for hip
 * \param[in] body_part_id body part ID
 */
HAM_Muscle::HAM_Muscle(CtrlInputs *inputs, Articulation *pitch_knee, Articulation *pitch_hip, int body_part_id):
	Muscle(inputs, HAM_MUSCLE, body_part_id, 0.043, 0.132, 1060.0, 18.0, 0.45, 0.44)
{
	rho_pk     = 0.7;
	r0_pk      = 0.021; 
	phi_max_pk = 0.0 * DEG_TO_RAD;
	phi_ref_pk = 0.0 * DEG_TO_RAD;

	rho_ph     = 0.7;
	r0_ph      = 0.034; 
	phi_ref_ph = -25.0 * DEG_TO_RAD;

	rm_pk = r0_pk;
	rm_ph = r0_ph;

	this->pitch_knee = pitch_knee;
	this->pitch_hip  = pitch_hip;

	// safety
	if ((body_part_id != RIGHT_LEG_BODY) && (body_part_id != LEFT_LEG_BODY))
	{
		std::cout << "Muscle error: body part ID (" << body_part_id << ") is not correct for HAM muscle !" << std::endl;
		exit(EXIT_FAILURE);
	}
}

/*! \brief destructor
 */
HAM_Muscle::~HAM_Muscle()
{
	
}

/*! \brief computation of rm
 */
void HAM_Muscle::rm_compute()
{
	rm_pk = r0_pk * cos(pitch_knee->get_q() - phi_max_pk);

	//rm_ph = r0_ph; // constant
}

/*! \brief computation of l.mtu
 */
void HAM_Muscle::lmtu_compute()
{
	double delta_lmtu_pk = -rho_pk * r0_pk * (sin(pitch_knee->get_q() - phi_max_pk) - sin(phi_ref_pk - phi_max_pk));
	double delta_lmtu_ph = -rho_ph * r0_ph * (pitch_hip->get_q() - phi_ref_ph);

	l.mtu = p.l_opt + p.l_slack + delta_lmtu_pk + delta_lmtu_ph;
}

/*! \brief computation of the torques
 */
void HAM_Muscle::torques_compute()
{
	pitch_knee->add_Qq(rm_pk * F.m);
	pitch_hip->add_Qq(rm_ph * F.m);
}
