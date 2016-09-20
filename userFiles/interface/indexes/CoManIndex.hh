/*! 
 * \author Nicolas Van der Noot
 * \file CoManIndex.hh
 * \brief controller indexes
 *
 * Important: contrary to "CtrlIndex.hh" and "SimuIndex.hh", the indexes of this file cannot be modified !
 */

#ifndef _COMAN_INDEX_HH_
#define _COMAN_INDEX_HH_

namespace CoManIndex
{
	enum JointIndexes
	{
		// right leg
		RightHipPitch  = 3,
		RightHipRoll   = 5,
		RightHipYaw    = 6,
		RightKneePitch = 7,
		RightFootRoll  = 9,
		RightFootPitch = 8,

		// left leg
		LeftHipPitch  = 4,
		LeftHipRoll   = 10,
		LeftHipYaw    = 11,
		LeftKneePitch = 12,
		LeftFootRoll  = 14,
		LeftFootPitch = 13,
		
		// torso
		TorsoRoll  = 2,
		TorsoPitch = 1,
		TorsoYaw   = 0,

		// right arm
		RightShPitch  = 15,
		RightShRoll   = 16,
		RightShYaw    = 17,
		RightElbPitch = 18,

		// left arm
		LeftShPitch  = 19,
		LeftShRoll   = 20,
		LeftShYaw    = 21,
		LeftElbPitch = 22
	};

	const char* get_index_name(int index);
}

#endif
