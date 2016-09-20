/*! 
 * \author Nicolas Van der Noot
 * \file CtrlIndex.hh
 * \brief controller indexes
 */

#ifndef _CTRL_INDEX_HH_
#define _CTRL_INDEX_HH_

namespace CtrlIndex
{
	enum JointIndexes
	{
		RightHipPitch, RightHipRoll, RightHipYaw, RightKneePitch, RightFootRoll, RightFootPitch, // right leg
		LeftHipPitch, LeftHipRoll, LeftHipYaw, LeftKneePitch, LeftFootRoll, LeftFootPitch, // left leg
		TorsoRoll, TorsoPitch, TorsoYaw, // torso
		RightShPitch, RightShRoll, RightShYaw, RightElbPitch, // right arm
		LeftShPitch, LeftShRoll, LeftShYaw, LeftElbPitch // left arm
	};

	const char* get_index_name(int index);
}

#endif
