#include "SAD_Muscle.hh"

/*! \brief constructor
 * 
 * \param[in] inputs inputs of the controller
 * \param[in] roll_shoulder joint: roll for shoulder
 * \param[in] body_part_id body part ID
 */
SAD_Muscle::SAD_Muscle(CtrlInputs *inputs, Articulation *roll_shoulder, int body_part_id):
	Muscle(inputs, SAD_MUSCLE, body_part_id, 0.059, 0.056, 140.0, 18.0, 0.08, 0.42) 
{
	rho_rs = 0.6;
	r0_rs  = 0.021;

	switch (body_part_id)
	{
		case RIGHT_ARM_BODY:
			phi_ref_rs = -155.0 * DEG_TO_RAD;
			phi_max_rs = -20.0 * DEG_TO_RAD;
			break;

		case LEFT_ARM_BODY:
			phi_ref_rs = 155.0 * DEG_TO_RAD;
			phi_max_rs = 20.0 * DEG_TO_RAD;
			break;

		default:
			std::cout << "Muscle error: body part ID (" << body_part_id << ") is not correct for SAI muscle !" << std::endl;
			exit(EXIT_FAILURE);
	}

	rm_rs = r0_rs;

	this->roll_shoulder = roll_shoulder;
}

/*! \brief destructor
 */
SAD_Muscle::~SAD_Muscle()
{

}

/*! \brief computation of rm
 */
void SAD_Muscle::rm_compute()
{
	rm_rs = r0_rs * cos(roll_shoulder->get_q() - phi_max_rs);
}

/*! \brief computation of l.mtu
 */
void SAD_Muscle::lmtu_compute()
{
	double delta_lmtu_rs;

	if (body_part_id == RIGHT_ARM_BODY)
	{
		delta_lmtu_rs = -rho_rs * r0_rs * (sin(roll_shoulder->get_q() - phi_max_rs) - sin(phi_ref_rs - phi_max_rs));
	}
	else
	{
		delta_lmtu_rs = rho_rs * r0_rs * (sin(roll_shoulder->get_q() - phi_max_rs) - sin(phi_ref_rs - phi_max_rs));
	}

	l.mtu = p.l_opt + p.l_slack + delta_lmtu_rs;
}

/*! \brief computation of the torques
 */
void SAD_Muscle::torques_compute()
{
	if (body_part_id == RIGHT_ARM_BODY)
	{
		roll_shoulder->add_Qq(rm_rs * F.m);
	}
	else
	{
		roll_shoulder->add_Qq(-rm_rs * F.m);
	}
}