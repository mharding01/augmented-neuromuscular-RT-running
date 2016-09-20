#include "ModelSimuIndex.hh"
#include "SimuIndex.hh"
#include "user_all_id.h"

#include <iostream>
#include <stdlib.h>

/*! \brief get the indexes related to the MBS file for the joints
 * 
 * \param[in] joint_id ID of the requested joint
 * \return requested MBS index
 */
int ModelSimuIndex::get_mbs_joint_index(int joint_id)
{
	switch (joint_id)
	{
		// floating base
		case SimuJointIndex::FloatingT1 : return FJ_T1_Coman_id;
		case SimuJointIndex::FloatingT2 : return FJ_T2_Coman_id;
		case SimuJointIndex::FloatingT3 : return FJ_T3_Coman_id;
		case SimuJointIndex::FloatingR1 : return FJ_R1_Coman_id;
		case SimuJointIndex::FloatingR2 : return FJ_R2_Coman_id;
		case SimuJointIndex::FloatingR3 : return FJ_R3_Coman_id;

		// right leg
		case SimuJointIndex::RightHipPitch  : return RHipSag_id;
		case SimuJointIndex::RightHipRoll   : return RHipLat_id;
		case SimuJointIndex::RightHipYaw    : return RHipYaw_id;
		case SimuJointIndex::RightKneePitch : return RKneeSag_id;
		case SimuJointIndex::RightFootRoll  : return RAnkLat_id;
		case SimuJointIndex::RightFootPitch : return RAnkSag_id;

		// left leg
		case SimuJointIndex::LeftHipPitch  : return LHipSag_id;
		case SimuJointIndex::LeftHipRoll   : return LHipLat_id;
		case SimuJointIndex::LeftHipYaw    : return LHipYaw_id;
		case SimuJointIndex::LeftKneePitch : return LKneeSag_id;
		case SimuJointIndex::LeftFootRoll  : return LAnkLat_id;
		case SimuJointIndex::LeftFootPitch : return LAnkSag_id;

		// torso
		case SimuJointIndex::TorsoRoll  : return WaistLat_id;
		case SimuJointIndex::TorsoPitch : return WaistSag_id;
		case SimuJointIndex::TorsoYaw   : return WaistYaw_id;

		// right arm
		case SimuJointIndex::RightShPitch  : return RShSag_id;
		case SimuJointIndex::RightShRoll   : return RShLat_id;
		case SimuJointIndex::RightShYaw    : return RShYaw_id;
		case SimuJointIndex::RightElbPitch : return RElb_id;

		// left arm
		case SimuJointIndex::LeftShPitch  : return LShSag_id;
		case SimuJointIndex::LeftShRoll   : return LShLat_id;
		case SimuJointIndex::LeftShYaw    : return LShYaw_id;
		case SimuJointIndex::LeftElbPitch : return LElb_id;
	
		default:
			std::cout << "Error: unknown joint index: " << joint_id << std::endl;
			exit(EXIT_FAILURE);
	}

	return -1;
}

/*! \brief get the indexes related to the MBS file for the bodies
 * 
 * \param[in] body_id ID of the requested body
 * \return requested MBS index
 */
int ModelSimuIndex::get_mbs_body_index(int body_id)
{
	switch (body_id)
	{
		case SimuBodyIndex::Waist : return Waist_id;

		case SimuBodyIndex::RHipmot : return RHipmot_id;
		case SimuBodyIndex::RThighUpLeg  : return RThighUpLeg_id;
		case SimuBodyIndex::RThighLowLeg : return RThighLowLeg_id;
		case SimuBodyIndex::RLowLeg  : return RLowLeg_id;
		case SimuBodyIndex::RFootmot : return RFootmot_id;
		case SimuBodyIndex::RFoot    : return RFoot_id;

		case SimuBodyIndex::LHipmot : return LHipmot_id;
		case SimuBodyIndex::LThighUpLeg  : return LThighUpLeg_id;
		case SimuBodyIndex::LThighLowLeg : return LThighLowLeg_id;
		case SimuBodyIndex::LLowLeg  : return LLowLeg_id;
		case SimuBodyIndex::LFootmot : return LFootmot_id;
		case SimuBodyIndex::LFoot    : return LFoot_id;

		case SimuBodyIndex::DWL : return DWL_id;
		case SimuBodyIndex::DWS : return DWS_id;
		case SimuBodyIndex::DWYTorso : return DWYTorso_id;

		case SimuBodyIndex::RShp : return RShp_id;
		case SimuBodyIndex::RShr : return RShr_id;
		case SimuBodyIndex::RShy : return RShy_id;
		case SimuBodyIndex::RElb : return RElb_id;
		case SimuBodyIndex::LShp : return LShp_id;
		case SimuBodyIndex::LShr : return LShr_id;
		case SimuBodyIndex::LShy : return LShy_id;
		case SimuBodyIndex::LElb : return LElb_id;
	
		default:
				std::cout << "Error: unknown body index: " << body_id << std::endl;
				exit(EXIT_FAILURE);
	}

	return -1;
}

/*! \brief get the indexes related to the MBS file for the S sensors
 * 
 * \param[in] Ssens_id ID of the requested S sensor
 * \return requested MBS index
 */
int ModelSimuIndex::get_mbs_Ssens_index(int Ssens_id)
{
	switch (Ssens_id)
	{
		// mid-waist
		case SimuSsensIndex::MidWaist : return mbs_data->Nsensor + Waist_contact_force_id;

		// one part feet
		case SimuSsensIndex::RightFoot : return mbs_data->Nsensor + RFoot_Force_id;
		case SimuSsensIndex::LeftFoot  : return mbs_data->Nsensor + LFoot_Force_id;

		// legs
		case SimuSsensIndex::RThigh  : return mbs_data->Nsensor + RThigh_contact_force_id;
		case SimuSsensIndex::LThigh  : return mbs_data->Nsensor + LThigh_contact_force_id;
		case SimuSsensIndex::RLowLeg : return mbs_data->Nsensor + RLowLeg_contact_force_id;
		case SimuSsensIndex::LLowLeg : return mbs_data->Nsensor + LLowLeg_contact_visu_id;

		// torso
		case SimuSsensIndex::Torso : return mbs_data->Nsensor + Torso_contact_force_id;

		// arms
		case SimuSsensIndex::RShoulder : return mbs_data->Nsensor + RShoulder_contact_force_id;
		case SimuSsensIndex::LShoulder : return mbs_data->Nsensor + LShoulder_contact_force_id;
		case SimuSsensIndex::RElbow    : return mbs_data->Nsensor + RElbow_contact_force_id;
		case SimuSsensIndex::LElbow    : return mbs_data->Nsensor + LElbow_contact_force_id;

		// com
		case SimuSsensIndex::WaistCg : return WaistCg_id;

		case SimuSsensIndex::RHipmotCg      : return RHipmotCg_id;
		case SimuSsensIndex::RThighUpLegCg  : return RThighUpLegCg_id;
		case SimuSsensIndex::RThighLowLegCg : return RThighLowLegCg_id;

		case SimuSsensIndex::RLowLegCg  : return RLowLegCg_id;
		case SimuSsensIndex::RFootmotCg : return RFootmotCg_id;
		case SimuSsensIndex::RFootCg    : return RFootCg_id;

		case SimuSsensIndex::LHipmotCg      : return LHipmotCg_id;
		case SimuSsensIndex::LThighUpLegCg  : return LThighUpLegCg_id;
		case SimuSsensIndex::LThighLowLegCg : return LThighLowLegCg_id;

		case SimuSsensIndex::LLowLegCg  : return LLowLegCg_id;
		case SimuSsensIndex::LFootmotCg : return LFootmotCg_id;
		case SimuSsensIndex::LFootCg    : return LFootCg_id;

		case SimuSsensIndex::DWLCg : return DWLCg_id;
		case SimuSsensIndex::DWSCg : return DWSCg_id;
		case SimuSsensIndex::DWYCg : return DWYCg_id;

		case SimuSsensIndex::RShpCg : return RShpCg_id;
		case SimuSsensIndex::RShrCg : return RShrCg_id;
		case SimuSsensIndex::RShyCg : return RShyCg_id;
		case SimuSsensIndex::RElbCg : return RElbCg_id;

		case SimuSsensIndex::LShpCg : return LShpCg_id;
		case SimuSsensIndex::LShrCg : return LShrCg_id;
		case SimuSsensIndex::LShyCg : return LShyCg_id;
		case SimuSsensIndex::LElbCg : return LElbCg_id;
	
		default:
			std::cout << "Error: unknown S sensor index: " << Ssens_id << std::endl;
			exit(EXIT_FAILURE);
	}

	return -1;
}

/*! \brief get the indexes related to the MBS file for the F sensors
 * 
 * \param[in] Fsens_id ID of the requested F sensor
 * \return requested MBS index
 */
int ModelSimuIndex::get_mbs_Fsens_index(int Fsens_id)
{
	switch (Fsens_id)
	{
		// mid-waist
		case SimuFsensIndex::MidWaist : return Waist_contact_force_id;

		// legs
		case SimuFsensIndex::RThigh  : return RThigh_contact_force_id;
		case SimuFsensIndex::LThigh  : return LThigh_contact_force_id;
		case SimuFsensIndex::RLowLeg : return RLowLeg_contact_force_id;
		case SimuFsensIndex::LLowLeg : return LLowLeg_contact_visu_id;

		// one part feet
		case SimuFsensIndex::RightFoot : return RFoot_Force_id;
		case SimuFsensIndex::LeftFoot  : return LFoot_Force_id;

		// torso
		case SimuFsensIndex::Torso : return Torso_contact_force_id;

		// arms
		case SimuFsensIndex::RShoulder : return RShoulder_contact_force_id;
		case SimuFsensIndex::LShoulder : return LShoulder_contact_force_id;
		case SimuFsensIndex::RElbow    : return RElbow_contact_force_id;
		case SimuFsensIndex::LElbow    : return LElbow_contact_force_id;
	
		default:
			std::cout << "Error: unknown F sensor index: " << Fsens_id << std::endl;
			exit(EXIT_FAILURE);
	}

	return -1;
}

/*! \brief get the indexes related to the MBS file for the links
 * 
 * \param[in] link_id ID of the requested link
 * \return requested MBS index
 */
int ModelSimuIndex::get_mbs_link_index(int link_id)
{
	std::cout << "Error: this COMAN model does not feature any link !" << std::endl;
	exit(EXIT_FAILURE);

	return -1;
}
