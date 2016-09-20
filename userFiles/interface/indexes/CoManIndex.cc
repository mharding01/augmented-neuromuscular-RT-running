#include "CoManIndex.hh"

namespace CoManIndex
{
	const char* get_index_name(int index)
	{
		switch (index)
		{
			// right leg
			case RightHipPitch  : return "RightHipPitch";
			case RightHipRoll   : return "RightHipRoll";
			case RightHipYaw    : return "RightHipYaw";
			case RightKneePitch : return "RightKneePitch";
			case RightFootRoll  : return "RightFootRoll";
			case RightFootPitch : return "RightFootPitch";

			// left leg
			case LeftHipPitch  : return "LeftHipPitch";
			case LeftHipRoll   : return "LeftHipRoll";
			case LeftHipYaw    : return "LeftHipYaw";
			case LeftKneePitch : return "LeftKneePitch";
			case LeftFootRoll  : return "LeftFootRoll";
			case LeftFootPitch : return "LeftFootPitch";

			// torso
			case TorsoRoll  : return "TorsoRoll";
			case TorsoPitch : return "TorsoPitch";
			case TorsoYaw   : return "TorsoYaw";

			// right arm
			case RightShPitch  : return "RightShPitch";
			case RightShRoll   : return "RightShRoll";
			case RightShYaw    : return "RightShYaw";
			case RightElbPitch : return "RightElbPitch";

			// left arm
			case LeftShPitch  : return "LeftShPitch";
			case LeftShRoll   : return "LeftShRoll";
			case LeftShYaw    : return "LeftShYaw";
			case LeftElbPitch : return "LeftElbPitch";
		
			default : return "Unknown joint !";
		}
	}
};
