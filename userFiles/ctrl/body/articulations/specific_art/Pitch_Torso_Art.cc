
#include "Pitch_Torso_Art.hh"

/*! \brief constructor
 * 
 * \param[in] inputs controller inputs
 * \param[in] ctrl_index indexes of the controller
 */
Pitch_Torso_Art::Pitch_Torso_Art(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index): Articulation(inputs, ctrl_index, PITCH_TORSO_ART, TORSO_BODY)
{
	joint_id = ctrl_index->get_inv_index(CtrlIndex::TorsoPitch);

	q_min = -10.0 * DEG_TO_RAD;
	q_max =  45.0 * DEG_TO_RAD;
}

/*! \brief destructor
 */
Pitch_Torso_Art::~Pitch_Torso_Art()
{

}

/*! \brief add soft limit contribution
 */
void Pitch_Torso_Art::add_Qq_soft_lim()
{
	apply_Qq_soft(250.0);
}

