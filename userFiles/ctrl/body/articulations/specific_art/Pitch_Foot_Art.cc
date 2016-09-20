
#include "Pitch_Foot_Art.hh"

/*! \brief constructor
 * 
 * \param[in] inputs controller inputs
 * \param[in] ctrl_index indexes of the controller
 * \param[in] body_part_id ID of the arm
 */
Pitch_Foot_Art::Pitch_Foot_Art(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index, int body_part_id): Articulation(inputs, ctrl_index, PITCH_FOOT_ART, body_part_id)
{
	switch (body_part_id)
	{
		case RIGHT_LEG_BODY:
			joint_id = ctrl_index->get_inv_index(CtrlIndex::RightFootPitch);
			break;

		case LEFT_LEG_BODY:
			joint_id = ctrl_index->get_inv_index(CtrlIndex::LeftFootPitch);
			break;
	
		default:
			std::cout << "Error: not correct body part ID (" << body_part_id << ") for pitch foot art !" << std::endl;
			exit(EXIT_FAILURE);
	}

	q_min = -20.0 * DEG_TO_RAD;
	q_max =  40.0 * DEG_TO_RAD;
}

/*! \brief destructor
 */
Pitch_Foot_Art::~Pitch_Foot_Art()
{

}

/*! \brief add soft limit contribution
 */
void Pitch_Foot_Art::add_Qq_soft_lim()
{
	apply_Qq_soft(135.514);
}

