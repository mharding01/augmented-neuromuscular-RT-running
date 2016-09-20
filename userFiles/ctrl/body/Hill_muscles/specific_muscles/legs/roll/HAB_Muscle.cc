
#include "HAB_Muscle.hh"

/*! \brief constructor
 * 
 * \param[in] inputs inputs of the controller
 * \param[in] roll_hip joint: roll for hip
 * \param[in] body_part_id body part ID
 */
HAB_Muscle::HAB_Muscle(CtrlInputs *inputs, Articulation *roll_hip, int body_part_id):
	Muscle(inputs, HAB_MUSCLE, body_part_id, 0.038, 0.03, 1060.0, 18.0, 0.404, 0.5)
{
	rho_rh =  0.7;
	r0_rh  =  0.026;

	switch (body_part_id)
	{
		case RIGHT_LEG_BODY:
			phi_ref_rh = -10.0 * DEG_TO_RAD;
			break;

		case LEFT_LEG_BODY:
			phi_ref_rh = 10.0 * DEG_TO_RAD;
			break;

		default:
			std::cout << "Muscle error: body part ID (" << body_part_id << ") is not correct for HAB muscle !" << std::endl;
			exit(EXIT_FAILURE);
	}

	rm_rh = r0_rh;

	this->roll_hip = roll_hip;
}

/*! \brief destructor
 */
HAB_Muscle::~HAB_Muscle()
{

}

/*! \brief computation of rm
 */
void HAB_Muscle::rm_compute()
{
	//rm_rh = r0_rh; // constant  
}

/*! \brief computation of l.mtu
 */
void HAB_Muscle::lmtu_compute()
{
	double delta_lmtu_rh;

	if (body_part_id == RIGHT_LEG_BODY)
	{
		delta_lmtu_rh = rho_rh * r0_rh * (roll_hip->get_q() - phi_ref_rh);
	}
	else
	{
		delta_lmtu_rh = -rho_rh * r0_rh * (roll_hip->get_q() - phi_ref_rh);
	}
	
	l.mtu = p.l_opt + p.l_slack + delta_lmtu_rh;
}

/*! \brief computation of the torques
 */
void HAB_Muscle::torques_compute()
{
	if (body_part_id == RIGHT_LEG_BODY)
	{
		roll_hip->add_Qq(-rm_rh * F.m);
	}
	else
	{
		roll_hip->add_Qq(rm_rh * F.m);
	}
}
