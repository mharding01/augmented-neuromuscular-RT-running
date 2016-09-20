
#include "Roll_Torso_Art.hh"

/*! \brief constructor
 * 
 * \param[in] inputs controller inputs
 * \param[in] ctrl_index indexes of the controller
 */
Roll_Torso_Art::Roll_Torso_Art(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index): Articulation(inputs, ctrl_index, ROLL_TORSO_ART, TORSO_BODY)
{
	joint_id = ctrl_index->get_inv_index(CtrlIndex::TorsoRoll);

	q_min = -40.0 * DEG_TO_RAD;
	q_max =  40.0 * DEG_TO_RAD;
}

/*! \brief destructor
 */
Roll_Torso_Art::~Roll_Torso_Art()
{

}

/*! \brief add soft limit contribution
 */
void Roll_Torso_Art::add_Qq_soft_lim()
{
	apply_Qq_soft(300.0);
}

