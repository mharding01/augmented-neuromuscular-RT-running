
#include "Roll_Shoulder_Art.hh"

/*! \brief constructor
 * 
 * \param[in] inputs controller inputs
 * \param[in] ctrl_index indexes of the controller
 * \param[in] body_part_id ID of the arm
 */
Roll_Shoulder_Art::Roll_Shoulder_Art(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index, int body_part_id): Articulation(inputs, ctrl_index, ROLL_SHOULDER_ART, body_part_id)
{
	switch (body_part_id)
	{
		case RIGHT_ARM_BODY:
			joint_id = ctrl_index->get_inv_index(CtrlIndex::RightShRoll);
			q_min = -90.0 * DEG_TO_RAD;
			q_max = 10.0 * DEG_TO_RAD;
			break;

		case LEFT_ARM_BODY:
			joint_id = ctrl_index->get_inv_index(CtrlIndex::LeftShRoll);
			q_min = -10.0 * DEG_TO_RAD;
			q_max = 90.0 * DEG_TO_RAD;
			break;
	
		default:
			std::cout << "Error: not correct body part ID (" << body_part_id << ") for roll shoulder art !" << std::endl;
			exit(EXIT_FAILURE);
	}
}

/*! \brief destructor
 */
Roll_Shoulder_Art::~Roll_Shoulder_Art()
{

}

/*! \brief add soft limit contribution
 */
void Roll_Shoulder_Art::add_Qq_soft_lim()
{
	apply_Qq_soft(150.0);
}

