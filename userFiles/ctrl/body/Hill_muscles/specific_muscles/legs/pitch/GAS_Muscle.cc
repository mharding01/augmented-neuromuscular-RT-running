
#include "GAS_Muscle.hh"

/*! \brief constructor
 * 
 * \param[in] inputs inputs of the controller
 * \param[in] pitch_foot joint: pitch for foot
 * \param[in] pitch_knee joint: pitch for knee
 * \param[in] body_part_id body part ID
 */
GAS_Muscle::GAS_Muscle(CtrlInputs *inputs, Articulation *pitch_foot, Articulation *pitch_knee, int body_part_id):
	Muscle(inputs, GAS_MUSCLE, body_part_id, 0.021, 0.17, 530.0, 18.0, 0.11, 0.54)
{
	rho_pa     = 0.7;
	r0_pa      = 0.021; 
	phi_max_pa =  20.0 * DEG_TO_RAD;
	phi_ref_pa = -10.0 * DEG_TO_RAD; 

	rho_pk     = 0.7;
	r0_pk      = 0.021; 
	phi_max_pk = 40.0 * DEG_TO_RAD;
	phi_ref_pk = 15.0 * DEG_TO_RAD;

	rm_pa = r0_pa;
	rm_pk = r0_pk;

	this->pitch_foot  = pitch_foot;
	this->pitch_knee = pitch_knee;

	// safety
	if ((body_part_id != RIGHT_LEG_BODY) && (body_part_id != LEFT_LEG_BODY))
	{
		std::cout << "Muscle error: body part ID (" << body_part_id << ") is not correct for GAS muscle !" << std::endl;
		exit(EXIT_FAILURE);
	}
}

/*! \brief destructor
 */
GAS_Muscle::~GAS_Muscle()
{

}

/*! \brief computation of rm
 */
void GAS_Muscle::rm_compute()
{
	rm_pa = r0_pa * cos(pitch_foot->get_q()  - phi_max_pa);
	rm_pk = r0_pk * cos(pitch_knee->get_q() - phi_max_pk);
}

/*! \brief computation of l.mtu
 */
void GAS_Muscle::lmtu_compute()
{
	double delta_lmtu_pa = -rho_pa * r0_pa * (sin(pitch_foot->get_q()  - phi_max_pa) - sin(phi_ref_pa - phi_max_pa));
	double delta_lmtu_pk = -rho_pk * r0_pk * (sin(pitch_knee->get_q() - phi_max_pk) - sin(phi_ref_pk - phi_max_pk));

	l.mtu = p.l_opt + p.l_slack + delta_lmtu_pa + delta_lmtu_pk;
}

/*! \brief computation of the torques
 */
void GAS_Muscle::torques_compute()
{
	pitch_foot->add_Qq(rm_pa * F.m);
	pitch_knee->add_Qq(rm_pk * F.m);
}
