
#include "EVE_Muscle.hh"

/*! \brief constructor
 * 
 * \param[in] inputs inputs of the controller
 * \param[in] roll_foot joint: roll for foot
 * \param[in] body_part_id body part ID
 */
EVE_Muscle::EVE_Muscle(CtrlInputs *inputs, Articulation *roll_foot, int body_part_id):
	Muscle(inputs, EVE_MUSCLE, body_part_id, 0.021, 0.107, 250.0, 18.0, 0.053, 0.57)
{
	rho_ra =  0.7;
	r0_ra  =  0.013;

	switch (body_part_id)
	{
		case RIGHT_LEG_BODY:
			phi_max_ra = -10.0 * DEG_TO_RAD;
			phi_ref_ra =  -5.0 * DEG_TO_RAD;
			break;

		case LEFT_LEG_BODY:
			phi_max_ra = 10.0 * DEG_TO_RAD;
			phi_ref_ra =  5.0 * DEG_TO_RAD;
			break;

		default:
			std::cout << "Muscle error: body part ID (" << body_part_id << ") is not correct for EVE muscle !" << std::endl;
			exit(EXIT_FAILURE);
	}

	rm_ra = r0_ra;

	this->roll_foot = roll_foot;
}

/*! \brief destructor
 */
EVE_Muscle::~EVE_Muscle()
{

}

/*! \brief computation of rm
 */
void EVE_Muscle::rm_compute()
{
	rm_ra = r0_ra * cos(roll_foot->get_q() - phi_max_ra);
}

/*! \brief computation of l.mtu
 */
void EVE_Muscle::lmtu_compute()
{
	double delta_lmtu_ra;

	if (body_part_id == RIGHT_LEG_BODY)
	{
		delta_lmtu_ra = rho_ra * r0_ra * (sin(roll_foot->get_q() - phi_max_ra) - sin(phi_ref_ra - phi_max_ra));
	}
	else
	{
		delta_lmtu_ra = -rho_ra * r0_ra * (sin(roll_foot->get_q() - phi_max_ra) - sin(phi_ref_ra - phi_max_ra));
	}

	l.mtu = p.l_opt + p.l_slack + delta_lmtu_ra;
}

/*! \brief computation of the torques
 */
void EVE_Muscle::torques_compute()
{
	if (body_part_id == RIGHT_LEG_BODY)
	{
		roll_foot->add_Qq(-rm_ra * F.m);
	}
	else
	{
		roll_foot->add_Qq(rm_ra * F.m);
	}
}
