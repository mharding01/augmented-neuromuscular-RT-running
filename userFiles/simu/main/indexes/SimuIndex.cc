#include "SimuIndex.hh"

// joints
namespace SimuJointIndex
{
	/*! \brief get the name of a joint index
	 * 
	 * \param[in] index index of the joint
	 * \return requested name
	 */
	const char* get_index_name(int index)
	{
		switch (index)
		{
			// floating base
			case FloatingT1 : return "FloatingT1";
			case FloatingT2 : return "FloatingT2";
			case FloatingT3 : return "FloatingT3";
			case FloatingR1 : return "FloatingR1";
			case FloatingR2 : return "FloatingR2";
			case FloatingR3 : return "FloatingR3";

			// right leg
			case RightHipPitch  : return "RightHipPitch";
			case RightHipRoll   : return "RightHipRoll";
			case RightHipYaw    : return "RightHipYaw";
			case RightKneePitch : return "RightKneePitch";
			case RightFootRoll  : return "RightFootRoll";
			case RightFootPitch : return "RightFootPitch";
			case RightFlexProx  : return "RightFlexProx";
			case RightFlexDist  : return "RightFlexDist";
			case RightToePitch  : return "RightToePitch";

			// left leg
			case LeftHipPitch  : return "LeftHipPitch";
			case LeftHipRoll   : return "LeftHipRoll";
			case LeftHipYaw    : return "LeftHipYaw";
			case LeftKneePitch : return "LeftKneePitch";
			case LeftFootRoll  : return "LeftFootRoll";
			case LeftFootPitch : return "LeftFootPitch";
			case LeftFlexProx  : return "LeftFlexProx";
			case LeftFlexDist  : return "LeftFlexDist";
			case LeftToePitch  : return "LeftToePitch";

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

			// ball A
			case Ball_A_T1 : return "Ball_A_T1";
			case Ball_A_T2 : return "Ball_A_T2";
			case Ball_A_T3 : return "Ball_A_T3";
			case Ball_A_R1 : return "Ball_A_R1";
			case Ball_A_R2 : return "Ball_A_R2";
			case Ball_A_R3 : return "Ball_A_R3";
		
			default: return "Unknown joint !";
		}
	}
};

// bodies
namespace SimuBodyIndex
{
	/*! \brief get the name of a body index
	 * 
	 * \param[in] index index of the body
	 * \return requested name
	 */
	const char* get_index_name(int index)
	{
		switch (index)
		{

			// waist
			case Waist : return "Waist";

			// right leg
			case RHipmot : return "RHipmot";
			case RThighUpLeg  : return "RThighUpLeg";
			case RThighLowLeg : return "RThighLowLeg";
			case RLowLeg  : return "RLowLeg";
			case RFootmot : return "RFootmot";
			case RFoot    : return "RFoot";
			case RFlexPr  : return "RFlexPr";
			case RFlexDs  : return "RFlexDs";
			case RFootDistal : return "RFootDistal";

			// left leg
			case LHipmot : return "LHipmot";
			case LThighUpLeg  : return "LThighUpLeg";
			case LThighLowLeg : return "LThighLowLeg";
			case LLowLeg  : return "LLowLeg";
			case LFootmot : return "LFootmot";
			case LFoot    : return "LFoot";
			case LFlexPr  : return "LFlexPr";
			case LFlexDs  : return "LFlexDs";
			case LFootDistal : return "LFootDistal";

			// torso
			case DWL : return "DWL";
			case DWS : return "DWS";
			case DWYTorso : return "DWYTorso";

			// right arm
			case RShp : return "RShp";
			case RShr : return "RShr";
			case RShy : return "RShy";
			case RElb : return "RElb";

			// left arm
			case LShp : return "LShp";
			case LShr : return "LShr";
			case LShy : return "LShy";
			case LElb : return "LElb";

			// balls
			case Ball_A : return "Ball_A";
		
			default: return "Unknown link !";
		}
	}
};

// S sensors
namespace SimuSsensIndex
{
	/*! \brief get the name of a S sensor index
	 * 
	 * \param[in] index index of the S sensor
	 * \return requested name
	 */
	const char* get_index_name(int index)
	{
		switch (index)
		{
			// mid-waist
			case MidWaist : return "MidWaist";

			// legs
			case RThigh  : return "RThigh";
			case LThigh  : return "LThigh";
			case RLowLeg : return "RLowLeg";
			case LLowLeg : return "LLowLeg";

			// one part feet
			case RightFoot : return "RightFoot";
			case LeftFoot  : return "LeftFoot";

			// spring toe feet
			case RToe    : return "RToe";
			case LToe    : return "LToe";

			// flexible feet
			case RightFlexProx : return "RightFlexProx";
			case RightFlexDist : return "RightFlexDist";
			case LeftFlexProx  : return "LeftFlexProx";
			case LeftFlexDist  : return "LeftFlexDist";

			// torso
			case Torso : return "Torso";

			// arms
			case RShoulder : return "RShoulder";
			case LShoulder : return "LShoulder";
			case RElbow    : return "RElbow";
			case LElbow    : return "LElbow";

			// balls
			case Ball_A : return "Ball_A";

			// com
			case WaistCg : return "WaistCg";

			case RHipmotCg      : return "RHipmotCg";
			case RThighUpLegCg  : return "RThighUpLegCg";
			case RThighLowLegCg : return "RThighLowLegCg";
			case RLowLegCg   : return "RLowLegCg";
			case RFootmotCg  : return "RFootmotCg";
			case RFootCg     : return "RFootCg";

			case LHipmotCg      : return "LHipmotCg";
			case LThighUpLegCg  : return "LThighUpLegCg";
			case LThighLowLegCg : return "LThighLowLegCg";
			case LLowLegCg   : return "LLowLegCg";
			case LFootmotCg  : return "LFootmotCg";

			case DWLCg : return "DWLCg";
			case DWSCg : return "DWSCg";
			case DWYCg : return "DWYCg";

			case RShpCg : return "RShpCg";
			case RShrCg : return "RShrCg";
			case RShyCg : return "RShyCg";
			case RElbCg : return "RElbCg";

			case LShpCg : return "LShpCg";
			case LShrCg : return "LShrCg";
			case LShyCg : return "LShyCg";
			case LElbCg : return "LElbCg";

			default: return "Unknown S sensor !";
		}
	}
};

// F sensors
namespace SimuFsensIndex
{
	/*! \brief get the name of a F sensor index
	 * 
	 * \param[in] index index of the F sensor
	 * \return requested name
	 */
	const char* get_index_name(int index)
	{
		switch (index)
		{
			// mid-waist
			case MidWaist : return "MidWaist";

			// legs
			case RThigh  : return "RThigh";
			case LThigh  : return "LThigh";
			case RLowLeg : return "RLowLeg";
			case LLowLeg : return "LLowLeg";

			// one part feet
			case RightFoot : return "RightFoot";
			case LeftFoot  : return "LeftFoot";

			// spring toe feet
			case RightToe  : return "RightToe";
			case LeftToe   : return "LeftToe";

			// flexible feet
			case RightFlexProx : return "RightFlexProx";
			case RightFlexDist : return "RightFlexDist";
			case LeftFlexProx  : return "LeftFlexProx";
			case LeftFlexDist  : return "LeftFlexDist";
			
			// torso
			case Torso : return "Torso";

			// arms
			case RShoulder : return "RShoulder";
			case LShoulder : return "LShoulder";
			case RElbow    : return "RElbow";
			case LElbow    : return "LElbow";

			// balls
			case Ball_A : return "Ball_A";

			default: return "Unknown F sensor !";
		}
	}
};

// links
namespace SimuLinkIndex
{
	/*! \brief get the name of a link index
	 * 
	 * \param[in] index index of the link
	 * \return requested name
	 */
	const char* get_index_name(int index)
	{
		switch (index)
		{
			// flexible feet
			case RightFlexProx : return "RightFlexProx";
			case RightFlexDist : return "RightFlexDist";
			case LeftFlexProx  : return "LeftFlexProx";
			case LeftFlexDist  : return "LeftFlexDist";
		
			default: return "Unknown link !";
		}
	}
};
