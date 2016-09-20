
#include "Roll_Foot_Art.hh"

/*! \brief constructor
 * 
 * \param[in] inputs controller inputs
 * \param[in] ctrl_index indexes of the controller
 * \param[in] body_part_id ID of the arm
 */
Roll_Foot_Art::Roll_Foot_Art(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index, int body_part_id): Articulation(inputs, ctrl_index, ROLL_FOOT_ART, body_part_id)
{
	switch (body_part_id)
	{
		case RIGHT_LEG_BODY:
			joint_id = ctrl_index->get_inv_index(CtrlIndex::RightFootRoll);
			break;

		case LEFT_LEG_BODY:
			joint_id = ctrl_index->get_inv_index(CtrlIndex::LeftFootRoll);
			break;
	
		default:
			std::cout << "Error: not correct body part ID (" << body_part_id << ") for roll foot art !" << std::endl;
			exit(EXIT_FAILURE);
	}

	q_min = -25 * DEG_TO_RAD; // [rad]
	q_max =  25 * DEG_TO_RAD; // [rad]
}

/*! \brief destructor
 */
Roll_Foot_Art::~Roll_Foot_Art()
{

}

/*! \brief add soft limit contribution
 */
void Roll_Foot_Art::add_Qq_soft_lim()
{
	apply_Qq_soft(1000.0);
}
