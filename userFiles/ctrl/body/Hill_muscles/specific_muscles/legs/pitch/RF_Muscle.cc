
#include "RF_Muscle.hh"

/*! \brief constructor
 * 
 * \param[in] inputs inputs of the controller
 * \param[in] pitch_knee joint: pitch for knee
 * \param[in] pitch_hip joint: pitch for hip
 * \param[in] body_part_id body part ID
 */
RF_Muscle::RF_Muscle(CtrlInputs *inputs, Articulation *pitch_knee, Articulation *pitch_hip, int body_part_id):
	Muscle(inputs, RF_MUSCLE, body_part_id, 0.034, 0.149, 425.0, 18.0, 0.14, 0.45)
{
	rho_pk     = 0.5;
	r_min_pk   = 0.017; 
	r_max_pk   = 0.026; 
	phi_min_pk = 15 * DEG_TO_RAD;
	phi_max_pk = 135 * DEG_TO_RAD;
	phi_ref_pk = 55 * DEG_TO_RAD;
	k_pk 	   = acos((r_min_pk/r_max_pk)/(phi_min_pk-phi_max_pk));

	rho_ph     = 0.3;
	r0_ph      = 0.034;
	phi_ref_ph = 10 * DEG_TO_RAD;

	rm_ph = r0_ph;
	rm_pk = r_max_pk;

	this->pitch_knee = pitch_knee;
	this->pitch_hip  = pitch_hip;

	// safety
	if ((body_part_id != RIGHT_LEG_BODY) && (body_part_id != LEFT_LEG_BODY))
	{
		std::cout << "Muscle error: body part ID (" << body_part_id << ") is not correct for RF muscle !" << std::endl;
		exit(EXIT_FAILURE);
	}
}

/*! \brief destructor
 */
RF_Muscle::~RF_Muscle()
{
	
}

/*! \brief computation of rm
 */
void RF_Muscle::rm_compute()
{
	rm_pk = r_max_pk * cos(k_pk*(pitch_knee->get_q() - phi_max_pk));

	//rm_ph = r0_ph; // constant
}

/*! \brief computation of l.mtu
 */
void RF_Muscle::lmtu_compute()
{
	double delta_lmtu_pk = rho_pk * r_max_pk * (sin(k_pk*(pitch_knee->get_q() - phi_max_pk)) - sin(k_pk*(phi_ref_pk - phi_max_pk))) / k_pk;
	//double delta_lmtu_pk = rho_pk * r_max_pk * (sin(pitch_knee->get_q() - phi_max_pk) - sin(phi_ref_pk - phi_max_pk));

	double delta_lmtu_ph = rho_ph * r0_ph * (pitch_hip->get_q() - phi_ref_ph);

	l.mtu = p.l_opt + p.l_slack + delta_lmtu_pk + delta_lmtu_ph;
}

/*! \brief computation of the torques
 */
void RF_Muscle::torques_compute()
{
	pitch_knee->add_Qq(-rm_pk * F.m);
	pitch_hip->add_Qq(-rm_ph * F.m);
}
