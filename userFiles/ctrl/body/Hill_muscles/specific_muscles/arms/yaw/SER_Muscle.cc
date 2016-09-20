#include "SER_Muscle.hh"

/*! \brief constructor
 * 
 * \param[in] inputs inputs of the controller
 * \param[in] yaw_shoulder joint: yaw for shoulder
 * \param[in] body_part_id body part ID
 */
SER_Muscle::SER_Muscle(CtrlInputs *inputs, Articulation *yaw_shoulder, int body_part_id):
	Muscle(inputs, SER_MUSCLE, body_part_id, 0.032, 0.013, 430.0, 18.0, 0.14, 0.45) 
{
	rho_ys = 0.7;
	r0_ys  = 0.012;

	switch (body_part_id)
	{
		case RIGHT_ARM_BODY:
			phi_ref_ys =   0.0 * DEG_TO_RAD;
			phi_max_ys = -35.0 * DEG_TO_RAD;
			break;

		case LEFT_ARM_BODY:
			phi_ref_ys =  0.0 * DEG_TO_RAD;
			phi_max_ys = 35.0 * DEG_TO_RAD;
			break;

		default:
			std::cout << "Muscle error: body part ID (" << body_part_id << ") is not correct for SRR muscle !" << std::endl;
			exit(EXIT_FAILURE);
	}

	rm_ys = r0_ys;

	this->yaw_shoulder = yaw_shoulder;
}

/*! \brief destructor
 */
SER_Muscle::~SER_Muscle()
{

}

/*! \brief computation of rm
 */
void SER_Muscle::rm_compute()
{
	rm_ys = r0_ys * cos(yaw_shoulder->get_q() - phi_max_ys);
}

/*! \brief computation of l.mtu
 */
void SER_Muscle::lmtu_compute()
{
	double delta_lmtu_ys;

	if (body_part_id == RIGHT_ARM_BODY)
	{
		delta_lmtu_ys = rho_ys * r0_ys * (sin(yaw_shoulder->get_q() - phi_max_ys) - sin(phi_ref_ys - phi_max_ys));
	}
	else
	{
		delta_lmtu_ys = -rho_ys * r0_ys * (sin(yaw_shoulder->get_q() - phi_max_ys) - sin(phi_ref_ys - phi_max_ys));
	}

	l.mtu = p.l_opt + p.l_slack + delta_lmtu_ys;
}

/*! \brief computation of the torques
 */
void SER_Muscle::torques_compute()
{
	if (body_part_id == RIGHT_ARM_BODY)
	{
		yaw_shoulder->add_Qq(-rm_ys * F.m);
	}
	else
	{
		yaw_shoulder->add_Qq(rm_ys * F.m);
	}
}