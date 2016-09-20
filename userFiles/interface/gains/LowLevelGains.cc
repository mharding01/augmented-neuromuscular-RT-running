#include "LowLevelGains.hh"
#include "SimuIndex.hh"
#include "Actuator.hh"

#include <iostream>

LowLevelGains::LowLevelGains(RobotActuators *actuators)
{
	this->actuators = actuators;
}

/*! \brief destructor
 */
LowLevelGains::~LowLevelGains()
{

}

/*! \brief choose the type of motor control between position and torque tracking
 * 
 * \param[in] joint_id ID of the joint in simulation
 *
 * Choose between 'POSITION_TRACKING' and 'TORQUE_TRACKING'
 */
int LowLevelGains::motor_config(int joint_id)
{
	switch (joint_id)
	{
		// right leg
		case SimuJointIndex::RightHipPitch  : return TORQUE_TRACKING;
		case SimuJointIndex::RightHipRoll   : return POSITION_TRACKING;
		case SimuJointIndex::RightHipYaw    : return POSITION_TRACKING;
		case SimuJointIndex::RightKneePitch : return TORQUE_TRACKING;
		case SimuJointIndex::RightFootRoll  : return POSITION_TRACKING;
		case SimuJointIndex::RightFootPitch : return TORQUE_TRACKING;

		// left leg
		case SimuJointIndex::LeftHipPitch  : return TORQUE_TRACKING;
		case SimuJointIndex::LeftHipRoll   : return POSITION_TRACKING;
		case SimuJointIndex::LeftHipYaw    : return POSITION_TRACKING;
		case SimuJointIndex::LeftKneePitch : return TORQUE_TRACKING;
		case SimuJointIndex::LeftFootRoll  : return POSITION_TRACKING;
		case SimuJointIndex::LeftFootPitch : return TORQUE_TRACKING;

		// torso
		case SimuJointIndex::TorsoRoll  : return POSITION_TRACKING;
		case SimuJointIndex::TorsoPitch : return POSITION_TRACKING;
		case SimuJointIndex::TorsoYaw   : return POSITION_TRACKING;

		// right arm
		case SimuJointIndex::RightShPitch  : return TORQUE_TRACKING;
		case SimuJointIndex::RightShRoll   : return POSITION_TRACKING;
		case SimuJointIndex::RightShYaw    : return POSITION_TRACKING;
		case SimuJointIndex::RightElbPitch : return TORQUE_TRACKING;

		// left arm
		case SimuJointIndex::LeftShPitch  : return TORQUE_TRACKING;
		case SimuJointIndex::LeftShRoll   : return POSITION_TRACKING;
		case SimuJointIndex::LeftShYaw    : return POSITION_TRACKING;
		case SimuJointIndex::LeftElbPitch : return TORQUE_TRACKING;

		default:
			std::cout << "Error: unknown motor " << joint_id << " to choose tracking mode !" << std::endl;
			return -1;
	}
}

/*! \brief set the low-level impedance controller gains to achieve position tracking
 * 
 * \param[in] joint_id ID of the joint in simulation
 *
 * P_stiffness: position proportional gain = stiffness [Nm/rad]
 * D_damping  : position derivative gain = damping [(Nm s)/rad]
 *
 * P: torque proportional gain [V/Nm]
 * I: torque integral gain [V/(Nm s)]
 */
void LowLevelGains::set_gains_position(int joint_id)
{
	Actuator *cur_actuator;
	double P, I, P_stiffness, D_damping;

	cur_actuator = actuators->get_actuator_from_index(joint_id);

	switch (joint_id)
	{
		// right leg
		case SimuJointIndex::RightHipPitch:
			P_stiffness = 1000.0;
			D_damping   = 50.0;
			P = 0.445;
			I = 0.022;
			break;

		case SimuJointIndex::RightHipRoll:
			P_stiffness = 1000.0;
			D_damping   = 50.0;
			P = 0.445;
			I = 0.022;
			break;

		case SimuJointIndex::RightHipYaw:
			P_stiffness = 1000.0;
			D_damping   = 50.0;
			P = 0.445;
			I = 0.022;
			break;

		case SimuJointIndex::RightKneePitch:
			P_stiffness = 1000.0;
			D_damping   = 50.0;
			P = 0.445;
			I = 0.022;
			break;

		case SimuJointIndex::RightFootRoll:
			P_stiffness = 1000.0;
			D_damping   = 50.0;
			P = 0.445;
			I = 0.022;
			break;

		case SimuJointIndex::RightFootPitch:
			P_stiffness = 1000.0;
			D_damping   = 50.0;
			P = 0.445;
			I = 0.022;
			break;


		// left leg
		case SimuJointIndex::LeftHipPitch:
			P_stiffness = 1000.0;
			D_damping   = 50.0;
			P = 0.445;
			I = 0.022;
			break;

		case SimuJointIndex::LeftHipRoll:
			P_stiffness = 1000.0;
			D_damping   = 50.0;
			P = 0.445;
			I = 0.022;
			break;		

		case SimuJointIndex::LeftHipYaw:
			P_stiffness = 1000.0;
			D_damping   = 50.0;
			P = 0.445;
			I = 0.022;
			break;

		case SimuJointIndex::LeftKneePitch:
			P_stiffness = 1000.0;
			D_damping   = 50.0;
			P = 0.445;
			I = 0.022;
			break;

		case SimuJointIndex::LeftFootRoll:
			P_stiffness = 1000.0;
			D_damping   = 50.0;
			P = 0.445;
			I = 0.022;
			break;

		case SimuJointIndex::LeftFootPitch:
			P_stiffness = 1000.0;
			D_damping   = 50.0;
			P = 0.445;
			I = 0.022;
			break;


		// torso
		case SimuJointIndex::TorsoRoll:
			P_stiffness = 1000.0;
			D_damping   = 50.0;
			P = 0.445;
			I = 0.022;
			break;

		case SimuJointIndex::TorsoPitch:
			P_stiffness = 1000.0;
			D_damping   = 50.0;
			P = 0.445;
			I = 0.022;
			break;

		case SimuJointIndex::TorsoYaw:
			P_stiffness = 1000.0;
			D_damping   = 50.0;
			P = 0.445;
			I = 0.022;
			break;
		

		// right arm
		case SimuJointIndex::RightShPitch:
			P_stiffness = 500.0;
			D_damping   = 50.0;
			P = 0.445;
			I = 0.022;
			break;

		case SimuJointIndex::RightShRoll:
			P_stiffness = 500.0;
			D_damping   = 50.0;
			P = 0.445;
			I = 0.022;
			break;

		case SimuJointIndex::RightShYaw:
			P_stiffness = 500.0;
			D_damping   = 50.0;
			P = 0.445;
			I = 0.022;
			break;

		case SimuJointIndex::RightElbPitch:
			P_stiffness = 500.0;
			D_damping   = 50.0;
			P = 0.445;
			I = 0.022;
			break;


		// left arm
		case SimuJointIndex::LeftShPitch:
			P_stiffness = 500.0;
			D_damping   = 50.0;
			P = 0.445;
			I = 0.022;
			break;

		case SimuJointIndex::LeftShRoll:
			P_stiffness = 500.0;
			D_damping   = 50.0;
			P = 0.445;
			I = 0.022;
			break;

		case SimuJointIndex::LeftShYaw:
			P_stiffness = 500.0;
			D_damping   = 50.0;
			P = 0.445;
			I = 0.022;
			break;

		case SimuJointIndex::LeftElbPitch:
			P_stiffness = 500.0;
			D_damping   = 50.0;
			P = 0.445;
			I = 0.022;
			break;

		default:
			P_stiffness = 0.0;
			D_damping   = 0.0;
			P = 0.0;
			I = 0.0;
			std::cout << "Low-level position gains error, unknown motor: " << joint_id << " !" << std::endl;
			break;
	}

	cur_actuator->set_stiffness_damping(P_stiffness, D_damping);
	cur_actuator->set_PI_gains(P, I);
}

/*! \brief set the low-level impedance controller gains to achieve torque tracking
 * 
 * \param[in] joint_id ID of the joint in simulation
 *
 * P_stiffness: position proportional gain = stiffness [Nm/rad]
 * D_damping  : position derivative gain = damping [(Nm s)/rad]
 *
 * P: torque proportional gain [V/Nm]
 * I: torque integral gain [V/(Nm s)]
 */
void LowLevelGains::set_gains_torque(int joint_id)
{
	Actuator *cur_actuator;
	double P, I, P_stiffness, D_damping;

	cur_actuator = actuators->get_actuator_from_index(joint_id);

	switch (joint_id)
	{
		// right leg
		case SimuJointIndex::RightHipPitch:
			P_stiffness = 0.0;
			D_damping   = 0.1;
			P = 10.0;
			I = 0.4;
			break;

		case SimuJointIndex::RightHipRoll:
			P_stiffness = 3.0;
			D_damping   = 0.3;
			P = 4.0;
			I = 0.2;
			break;

		case SimuJointIndex::RightHipYaw:
			P_stiffness = 0.0;
			D_damping   = 0.1;
			P = 4.0;
			I = 0.2;
			break;

		case SimuJointIndex::RightKneePitch:
			P_stiffness = 0.0;
			D_damping   = 0.1;
			P = 10.0;
			I = 0.4;
			break;

		case SimuJointIndex::RightFootRoll:
			P_stiffness = 3.0;
			D_damping   = 0.3;
			P = 4.0;
			I = 0.2;
			break;

		case SimuJointIndex::RightFootPitch:
			P_stiffness = 0.0;
			D_damping   = 0.1;
			P = 10.0;
			I = 0.4;
			break;


		// left leg
		case SimuJointIndex::LeftHipPitch:
			P_stiffness = 0.0;
			D_damping   = 0.1;
			P = 10.0;
			I = 0.4;
			break;

		case SimuJointIndex::LeftHipRoll:
			P_stiffness = 3.0;
			D_damping   = 0.3;
			P = 4.0;
			I = 0.2;
			break;		

		case SimuJointIndex::LeftHipYaw:
			P_stiffness = 0.0;
			D_damping   = 0.1;
			P = 4.0;
			I = 0.2;
			break;

		case SimuJointIndex::LeftKneePitch:
			P_stiffness = 0.0;
			D_damping   = 0.1;
			P = 10.0;
			I = 0.4;
			break;

		case SimuJointIndex::LeftFootRoll:
			P_stiffness = 3.0;
			D_damping   = 0.3;
			P = 4.0;
			I = 0.2;
			break;

		case SimuJointIndex::LeftFootPitch:
			P_stiffness = 0.0;
			D_damping   = 0.1;
			P = 10.0;
			I = 0.4;
			break;


		// torso
		case SimuJointIndex::TorsoRoll:
			P_stiffness = 10.0;
			D_damping   = 2.0;
			P = 6.0;
			I = 0.3;
			break;

		case SimuJointIndex::TorsoPitch:
			P_stiffness = 5.0;
			D_damping   = 0.5;
			P = 10.0;
			I = 0.4;
			break;

		case SimuJointIndex::TorsoYaw:
			P_stiffness = 3.0;
			D_damping   = 0.3;
			P = 10.0;
			I = 0.4;
			break;
		

		// right arm
		case SimuJointIndex::RightShPitch:
			P_stiffness = 3.0;
			D_damping   = 0.3;
			P = 10.0;
			I = 0.4;
			break;

		case SimuJointIndex::RightShRoll:
			P_stiffness = 10.0;
			D_damping   = 1.0;
			P = 3.0;
			I = 1.0;
			break;

		case SimuJointIndex::RightShYaw:
			P_stiffness = 3.0;
			D_damping   = 0.3;
			P = 10.0;
			I = 0.4;
			break;

		case SimuJointIndex::RightElbPitch:
			P_stiffness = 0.0;
			D_damping   = 0.1;
			P = 10.0;
			I = 0.4;
			break;


		// left arm
		case SimuJointIndex::LeftShPitch:
			P_stiffness = 3.0;
			D_damping   = 0.3;
			P = 10.0;
			I = 0.4;
			break;

		case SimuJointIndex::LeftShRoll:
			P_stiffness = 10.0;
			D_damping   = 1.0;
			P = 3.0;
			I = 1.0;
			break;

		case SimuJointIndex::LeftShYaw:
			P_stiffness = 3.0;
			D_damping   = 0.3;
			P = 10.0;
			I = 0.4;
			break;

		case SimuJointIndex::LeftElbPitch:
			P_stiffness = 0.0;
			D_damping   = 0.1;
			P = 10.0;
			I = 0.4;
			break;

		default:
			P_stiffness = 0.0;
			D_damping   = 0.0;
			P = 0.0;
			I = 0.0;
			std::cout << "Low-level torque gains error, unknown motor: " << joint_id << " !" << std::endl;
			break;
	}

	cur_actuator->set_stiffness_damping(P_stiffness, D_damping);
	cur_actuator->set_PI_gains(P, I);
}
