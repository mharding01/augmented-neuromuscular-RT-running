#include "get_simu_id_interface.hh"
#include "CoManIndex.hh"
#include "SimuIndex.hh"

#include <iostream>

/*! \brief get the COMAN ID correponding to simulation interface
 * 
 * \param[in] motor_id ID of the motor (as in the interface)
 * \return requested MBS index
 */
int get_simu_id_interface(int motor_id)
{
	switch (motor_id)
	{
		// right leg
		case CoManIndex::RightHipPitch  : return SimuJointIndex::RightHipPitch;
		case CoManIndex::RightHipRoll   : return SimuJointIndex::RightHipRoll;
		case CoManIndex::RightHipYaw    : return SimuJointIndex::RightHipYaw;
		case CoManIndex::RightKneePitch : return SimuJointIndex::RightKneePitch;
		case CoManIndex::RightFootRoll  : return SimuJointIndex::RightFootRoll;
		case CoManIndex::RightFootPitch : return SimuJointIndex::RightFootPitch;

		// left leg
		case CoManIndex::LeftHipPitch  : return SimuJointIndex::LeftHipPitch;
		case CoManIndex::LeftHipRoll   : return SimuJointIndex::LeftHipRoll;
		case CoManIndex::LeftHipYaw    : return SimuJointIndex::LeftHipYaw;
		case CoManIndex::LeftKneePitch : return SimuJointIndex::LeftKneePitch;
		case CoManIndex::LeftFootRoll  : return SimuJointIndex::LeftFootRoll;
		case CoManIndex::LeftFootPitch : return SimuJointIndex::LeftFootPitch;

		// torso
		case CoManIndex::TorsoRoll  : return SimuJointIndex::TorsoRoll;
		case CoManIndex::TorsoPitch : return SimuJointIndex::TorsoPitch;
		case CoManIndex::TorsoYaw   : return SimuJointIndex::TorsoYaw;

		// right arm
		case CoManIndex::RightShPitch  : return SimuJointIndex::RightShPitch;
		case CoManIndex::RightShRoll   : return SimuJointIndex::RightShRoll;
		case CoManIndex::RightShYaw    : return SimuJointIndex::RightShYaw;
		case CoManIndex::RightElbPitch : return SimuJointIndex::RightElbPitch;

		// left arm
		case CoManIndex::LeftShPitch  : return SimuJointIndex::LeftShPitch;
		case CoManIndex::LeftShRoll   : return SimuJointIndex::LeftShRoll;
		case CoManIndex::LeftShYaw    : return SimuJointIndex::LeftShYaw;
		case CoManIndex::LeftElbPitch : return SimuJointIndex::LeftElbPitch;

		default:
			std::cout << "Interface error unknown joint " << motor_id << " !!!" << std::endl;
			return -1;
	}
}
