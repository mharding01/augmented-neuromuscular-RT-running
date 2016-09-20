/*! 
 * \author Nicolas Van der Noot
 * \file SimuIndex.hh
 * \brief controller indexes
 */

#ifndef _SIMU_INDEX_HH_
#define _SIMU_INDEX_HH_

// joints
namespace SimuJointIndex
{
	enum JointIndexes
	{
		FloatingT1, FloatingT2, FloatingT3, FloatingR1, FloatingR2, FloatingR3, // floating base	
		RightHipPitch, RightHipRoll, RightHipYaw, RightKneePitch, RightFootRoll, RightFootPitch, RightFlexProx, RightFlexDist, RightToePitch, // right leg		
		LeftHipPitch, LeftHipRoll, LeftHipYaw, LeftKneePitch, LeftFootRoll, LeftFootPitch, LeftFlexProx, LeftFlexDist, LeftToePitch, // left leg
		TorsoRoll, TorsoPitch, TorsoYaw, // torso
		RightShPitch, RightShRoll, RightShYaw, RightElbPitch, // right arm
		LeftShPitch, LeftShRoll, LeftShYaw, LeftElbPitch, // left arm
		Ball_A_T1, Ball_A_T2, Ball_A_T3, Ball_A_R1, Ball_A_R2, Ball_A_R3 // ball A
	};

	const char* get_index_name(int index);
}

// bodies
namespace SimuBodyIndex
{
	enum BodyIndexes
	{
		Waist, // waist
		RHipmot, RThighUpLeg, RThighLowLeg, RLowLeg, RFootmot, RFoot, RFlexPr, RFlexDs, RFootDistal, // right leg
		LHipmot, LThighUpLeg, LThighLowLeg, LLowLeg, LFootmot, LFoot, LFlexPr, LFlexDs, LFootDistal, // left leg
		DWL, DWS, DWYTorso, // torso
		RShp, RShr, RShy, RElb, // right arm
		LShp, LShr, LShy, LElb, // left arm
		Ball_A // balls
	};

	const char* get_index_name(int index);
}

// S sensors
namespace SimuSsensIndex
{
	enum SsensIndexes
	{
		MidWaist, // mid-waist
		RThigh, LThigh, RLowLeg, LLowLeg, // legs
		RightFoot, LeftFoot, // one part feet
		RToe, LToe, // spring toe feet
		RightFlexProx, RightFlexDist, LeftFlexProx, LeftFlexDist, // flexible feet
		Torso, // torso
		RShoulder, LShoulder, RElbow, LElbow, // arms
		Ball_A, // balls

		// com
		WaistCg,
		RHipmotCg, RThighUpLegCg, RThighLowLegCg, RLowLegCg, RFootmotCg, RFootCg,
		LHipmotCg, LThighUpLegCg, LThighLowLegCg, LLowLegCg, LFootmotCg, LFootCg,
		DWLCg, DWSCg, DWYCg,
		RShpCg, RShrCg, RShyCg, RElbCg,
		LShpCg, LShrCg, LShyCg, LElbCg
	};

	const char* get_index_name(int index);
}

// F sensors
namespace SimuFsensIndex
{
	enum FsensIndexes
	{
		MidWaist, // mid-waist
		RThigh, LThigh, RLowLeg, LLowLeg, // legs
		RightFoot, LeftFoot, // one part feet
		RightToe, LeftToe, // spring toe feet
		RightFlexProx, RightFlexDist, LeftFlexProx, LeftFlexDist, // flexible feet
		Torso, // torso
		RShoulder, LShoulder, RElbow, LElbow, // arms
		Ball_A // balls
	};

	const char* get_index_name(int index);
}

// links
namespace SimuLinkIndex
{
	enum LinkIndexes
	{
		RightFlexProx, RightFlexDist, LeftFlexProx, LeftFlexDist // flexible feet
	};

	const char* get_index_name(int index);
}

#endif
