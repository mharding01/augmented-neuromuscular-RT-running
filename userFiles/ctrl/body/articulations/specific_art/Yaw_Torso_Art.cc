
#include "Yaw_Torso_Art.hh"

/*! \brief constructor
 * 
 * \param[in] inputs controller inputs
 * \param[in] ctrl_index indexes of the controller
 */
Yaw_Torso_Art::Yaw_Torso_Art(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index): Articulation(inputs, ctrl_index, YAW_TORSO_ART, TORSO_BODY)
{
	joint_id = ctrl_index->get_inv_index(CtrlIndex::TorsoYaw);

	q_min = -50.0 * DEG_TO_RAD;
	q_max =  50.0 * DEG_TO_RAD;
}

/*! \brief destructor
 */
Yaw_Torso_Art::~Yaw_Torso_Art()
{

}

/*! \brief add soft limit contribution
 */
void Yaw_Torso_Art::add_Qq_soft_lim()
{
	apply_Qq_soft(150.0);
}

