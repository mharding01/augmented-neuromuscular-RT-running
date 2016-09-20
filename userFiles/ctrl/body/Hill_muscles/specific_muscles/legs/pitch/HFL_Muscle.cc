
#include "HFL_Muscle.hh"

/*! \brief constructor
 * 
 * \param[in] inputs inputs of the controller
 * \param[in] pitch_hip joint: pitch for hip
 * \param[in] body_part_id body part ID
 */
HFL_Muscle::HFL_Muscle(CtrlInputs *inputs, Articulation *pitch_hip, int body_part_id):
	Muscle(inputs, HFL_MUSCLE, body_part_id, 0.047, 0.043, 710.0, 18.0, 0.33, 0.5)
{
	rho_ph     = 0.5;
	r0_ph      = 0.043; 
	phi_ref_ph = 0.0 * DEG_TO_RAD;

	rm_ph = r0_ph;

	this->pitch_hip = pitch_hip;

	// safety
	if ((body_part_id != RIGHT_LEG_BODY) && (body_part_id != LEFT_LEG_BODY))
	{
		std::cout << "Muscle error: body part ID (" << body_part_id << ") is not correct for HFL muscle !" << std::endl;
		exit(EXIT_FAILURE);
	}
}

/*! \brief destructor
 */
HFL_Muscle::~HFL_Muscle()
{

}

/*! \brief computation of rm
 */
void HFL_Muscle::rm_compute()
{
	//rm_ph = r0_ph; // constant  
}

/*! \brief computation of l.mtu
 */
void HFL_Muscle::lmtu_compute()
{
	double delta_lmtu_ph = rho_ph * r0_ph * (pitch_hip->get_q() - phi_ref_ph);

	l.mtu = p.l_opt + p.l_slack + delta_lmtu_ph;
}

/*! \brief computation of the torques
 */
void HFL_Muscle::torques_compute()
{
	pitch_hip->add_Qq(-rm_ph * F.m);
}
