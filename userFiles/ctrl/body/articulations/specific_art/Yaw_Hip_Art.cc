
#include "Yaw_Hip_Art.hh"

/*! \brief constructor
 * 
 * \param[in] inputs controller inputs
 * \param[in] ctrl_index indexes of the controller
 * \param[in] body_part_id ID of the arm
 */
Yaw_Hip_Art::Yaw_Hip_Art(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index, int body_part_id): Articulation(inputs, ctrl_index, YAW_HIP_ART, body_part_id)
{
	switch (body_part_id)
	{
		case RIGHT_LEG_BODY:
			joint_id = ctrl_index->get_inv_index(CtrlIndex::RightHipYaw);
			break;

		case LEFT_LEG_BODY:
			joint_id = ctrl_index->get_inv_index(CtrlIndex::LeftHipYaw);
			break;
	
		default:
			std::cout << "Error: not correct body part ID (" << body_part_id << ") for yaw hip art !" << std::endl;
			exit(EXIT_FAILURE);
	}

	q_min = -40.0 * DEG_TO_RAD; // [rad]
	q_max =  40.0 * DEG_TO_RAD; // [rad]
}

/*! \brief destructor
 */
Yaw_Hip_Art::~Yaw_Hip_Art()
{

}

/*! \brief add soft limit contribution
 */
void Yaw_Hip_Art::add_Qq_soft_lim()
{
	apply_Qq_soft(1000.0);
}
