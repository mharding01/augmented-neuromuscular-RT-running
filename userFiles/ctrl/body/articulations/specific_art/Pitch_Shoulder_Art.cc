
#include "Pitch_Shoulder_Art.hh"

/*! \brief constructor
 * 
 * \param[in] inputs controller inputs
 * \param[in] ctrl_index indexes of the controller
 * \param[in] body_part_id ID of the arm
 */
Pitch_Shoulder_Art::Pitch_Shoulder_Art(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index, int body_part_id): Articulation(inputs, ctrl_index, PITCH_SHOULDER_ART, body_part_id)
{
	switch (body_part_id)
	{
		case RIGHT_ARM_BODY:
			joint_id = ctrl_index->get_inv_index(CtrlIndex::RightShPitch);
			break;

		case LEFT_ARM_BODY:
			joint_id = ctrl_index->get_inv_index(CtrlIndex::LeftShPitch);
			break;
	
		default:
			std::cout << "Error: not correct body part ID (" << body_part_id << ") for pitch shoulder art !" << std::endl;
			exit(EXIT_FAILURE);
	}

	q_min = -180.0 * DEG_TO_RAD;
	q_max = 20.0 * DEG_TO_RAD;
}

/*! \brief destructor
 */
Pitch_Shoulder_Art::~Pitch_Shoulder_Art()
{

}

/*! \brief add soft limit contribution
 */
void Pitch_Shoulder_Art::add_Qq_soft_lim()
{
	apply_Qq_soft(0.0);
}

