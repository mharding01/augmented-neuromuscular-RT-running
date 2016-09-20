
#include "Yaw_Shoulder_Art.hh"

/*! \brief constructor
 * 
 * \param[in] inputs controller inputs
 * \param[in] ctrl_index indexes of the controller
 * \param[in] body_part_id ID of the arm
 */
Yaw_Shoulder_Art::Yaw_Shoulder_Art(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index, int body_part_id): Articulation(inputs, ctrl_index, YAW_SHOULDER_ART, body_part_id)
{
	switch (body_part_id)
	{
		case RIGHT_ARM_BODY:
			joint_id = ctrl_index->get_inv_index(CtrlIndex::RightShYaw);
			break;

		case LEFT_ARM_BODY:
			joint_id = ctrl_index->get_inv_index(CtrlIndex::LeftShYaw);
			break;
	
		default:
			std::cout << "Error: not correct body part ID (" << body_part_id << ") for yaw shoulder art !" << std::endl;
			exit(EXIT_FAILURE);
	}

	q_min = -80.0 * DEG_TO_RAD;
	q_max =  80.0 * DEG_TO_RAD;
}

/*! \brief destructor
 */
Yaw_Shoulder_Art::~Yaw_Shoulder_Art()
{

}

/*! \brief add soft limit contribution
 */
void Yaw_Shoulder_Art::add_Qq_soft_lim()
{
	apply_Qq_soft(250.0);
}

