
#include "INV_Muscle.hh"

/*! \brief constructor
 * 
 * \param[in] inputs inputs of the controller
 * \param[in] roll_foot joint: roll for foot
 * \param[in] body_part_id body part ID
 */
INV_Muscle::INV_Muscle(CtrlInputs *inputs, Articulation *roll_foot, int body_part_id):
	Muscle(inputs, INV_MUSCLE, body_part_id, 0.021, 0.128, 320.0, 18.0, 0.067, 0.55)
{
	rho_ra =  0.7;
	r0_ra  =  0.009;

	switch (body_part_id)
	{
		case RIGHT_LEG_BODY:
			phi_max_ra =   5.0 * DEG_TO_RAD;
			phi_ref_ra = -10.0 * DEG_TO_RAD;
			break;

		case LEFT_LEG_BODY:
			phi_max_ra = -5.0 * DEG_TO_RAD;
			phi_ref_ra = 10.0 * DEG_TO_RAD;
			break;

		default:
			std::cout << "Muscle error: body part ID (" << body_part_id << ") is not correct for INV muscle !" << std::endl;
			exit(EXIT_FAILURE);
	}

	rm_ra = r0_ra;

	this->roll_foot = roll_foot;
}

/*! \brief destructor
 */
INV_Muscle::~INV_Muscle()
{

}

/*! \brief computation of rm
 */
void INV_Muscle::rm_compute()
{
	rm_ra = r0_ra * cos(roll_foot->get_q() - phi_max_ra);
}

/*! \brief computation of l.mtu
 */
void INV_Muscle::lmtu_compute()
{
	double delta_lmtu_ra;

	if (body_part_id == RIGHT_LEG_BODY)
	{
		delta_lmtu_ra = -rho_ra * r0_ra * (sin(roll_foot->get_q() - phi_max_ra) - sin(phi_ref_ra - phi_max_ra));
	}
	else
	{
		delta_lmtu_ra = rho_ra * r0_ra * (sin(roll_foot->get_q() - phi_max_ra) - sin(phi_ref_ra - phi_max_ra));
	}

	l.mtu = p.l_opt + p.l_slack + delta_lmtu_ra;
}

/*! \brief computation of the torques
 */
void INV_Muscle::torques_compute()
{
	if (body_part_id == RIGHT_LEG_BODY)
	{
		roll_foot->add_Qq(rm_ra * F.m);
	}
	else
	{
		roll_foot->add_Qq(-rm_ra * F.m);
	}
}
