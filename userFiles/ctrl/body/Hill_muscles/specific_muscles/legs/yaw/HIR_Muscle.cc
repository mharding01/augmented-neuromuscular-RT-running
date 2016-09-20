
#include "HIR_Muscle.hh"

/*! \brief constructor
 * 
 * \param[in] inputs inputs of the controller
 * \param[in] yaw_hip joint: yaw for hip
 * \param[in] body_part_id body part ID
 */
HIR_Muscle::HIR_Muscle(CtrlInputs *inputs, Articulation *yaw_hip, int body_part_id):
	Muscle(inputs, HIR_MUSCLE, body_part_id, 0.034, 0.03, 430.0, 18.0, 0.144, 0.5)
{
	rho_yh =  0.7;
	r0_yh  =  0.013;

	switch (body_part_id)
	{
		case RIGHT_LEG_BODY:
			phi_ref_yh = -20.0 * DEG_TO_RAD;
			break;

		case LEFT_LEG_BODY:
			phi_ref_yh = 20.0 * DEG_TO_RAD;
			break;

		default:
			std::cout << "Muscle error: body part ID (" << body_part_id << ") is not correct for HIR muscle !" << std::endl;
			exit(EXIT_FAILURE);
	}

	rm_yh = r0_yh;

	this->yaw_hip = yaw_hip;
}

/*! \brief destructor
 */
HIR_Muscle::~HIR_Muscle()
{

}

/*! \brief computation of rm
 */
void HIR_Muscle::rm_compute()
{
	//rm_yh = r0_yh; // constant  
}

/*! \brief computation of l.mtu
 */
void HIR_Muscle::lmtu_compute()
{
	double delta_lmtu_yh;

	if (body_part_id == RIGHT_LEG_BODY)
	{
		delta_lmtu_yh = -rho_yh * r0_yh * (yaw_hip->get_q() - phi_ref_yh);
	}
	else
	{
		delta_lmtu_yh = rho_yh * r0_yh * (yaw_hip->get_q() - phi_ref_yh);
	}

	l.mtu = p.l_opt + p.l_slack + delta_lmtu_yh;
}

/*! \brief computation of the torques
 */
void HIR_Muscle::torques_compute()
{
	if (body_part_id == RIGHT_LEG_BODY)
	{
		yaw_hip->add_Qq(rm_yh * F.m);
	}
	else
	{
		yaw_hip->add_Qq(-rm_yh * F.m);
	}
}
