/*! 
 * \author Nicolas Van der Noot
 * \file controller_outputs.cc
 * \brief Outputs of the controller
 */

#include "controller_io.hh"
#include "user_simu_functions.h"
#include "MotorCoManIndex.hh"
#include "LowLevelGains.hh"
#include "RobotActuators.hh"
#include "get_simu_id_interface.hh"
#include "Actuator.hh"
#include "SimuIndex.hh"

/*! \brief interface of the controller with the robot for the outputs
 * 
 * \param[in] coman_index indexes of the COMAN robot (interface)
 * \param[in] actuators actuators of the robot
 * \param[in] gains gains of the low-level controller
 * \param[in] ovs outputs structure
 */
void controller_outputs(MotorCoManIndex *coman_index, RobotActuators *actuators, LowLevelGains *gains, Outputs_ctrl *ovs)
{
	int cur_coman_id, cur_simu_id;

	Actuator *cur_actuator;

	for(int i=0; i<coman_index->get_nb_mot(); i++)
	{
		cur_coman_id = coman_index->get_index(i);
		cur_simu_id = get_simu_id_interface(cur_coman_id);

		cur_actuator = actuators->get_actuator_from_index(cur_simu_id);

		// send references
		switch (gains->motor_config(cur_simu_id))
		{
			case POSITION_TRACKING:
				cur_actuator->set_q_ref(ovs->q_ref[cur_coman_id]);
				break;

			case TORQUE_TRACKING:
				cur_actuator->set_Qq_ref(ovs->Qq_ref[cur_coman_id]);
			default:
				break;
		}

		// low-level controller
		cur_actuator->low_level_controller();
	}
}

/*! \brief set the gains of the robot according to the tracking mode
 *
 * \param[in] coman_index indexes of the COMAN robot (interface)
 * \param[in] gains gains of the low-level controller
 */
void controller_tracking_mode(MotorCoManIndex *coman_index, LowLevelGains *gains)
{
	int cur_coman_id, cur_simu_id;

	for(int i=0; i<coman_index->get_nb_mot(); i++)
	{
		cur_coman_id = coman_index->get_index(i);
		cur_simu_id = get_simu_id_interface(cur_coman_id);

		// send references
		switch (gains->motor_config(cur_simu_id))
		{
			case POSITION_TRACKING:
				gains->set_gains_position(cur_simu_id);
				break;

			case TORQUE_TRACKING:
				gains->set_gains_torque(cur_simu_id);
			default:
				break;
		}
	}
}
