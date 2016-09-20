
#include "Roll_Hip_Art.hh"

/*! \brief constructor
 * 
 * \param[in] inputs controller inputs
 * \param[in] ctrl_index indexes of the controller
 * \param[in] body_part_id ID of the arm
 */
Roll_Hip_Art::Roll_Hip_Art(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index, int body_part_id): Articulation(inputs, ctrl_index, ROLL_HIP_ART, body_part_id)
{
	switch (body_part_id)
	{
		case RIGHT_LEG_BODY:
			joint_id = ctrl_index->get_inv_index(CtrlIndex::RightHipRoll);

			q_min = -40.0 * DEG_TO_RAD; // [rad]
			q_max =  10.0 * DEG_TO_RAD; // [rad]
			break;

		case LEFT_LEG_BODY:
			joint_id = ctrl_index->get_inv_index(CtrlIndex::LeftHipRoll);

			q_min = -10.0 * DEG_TO_RAD; // [rad]
			q_max =  40.0 * DEG_TO_RAD; // [rad]
			break;
	
		default:
			std::cout << "Error: not correct body part ID (" << body_part_id << ") for roll hip art !" << std::endl;
			exit(EXIT_FAILURE);
	}
}

/*! \brief destructor
 */
Roll_Hip_Art::~Roll_Hip_Art()
{

}

/*! \brief add soft limit contribution
 */
void Roll_Hip_Art::add_Qq_soft_lim()
{
	apply_Qq_soft(1000.0);
}
