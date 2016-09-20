#include "ModelSimuIndex.hh"
#include <iostream>

/*! \brief constructor
 * 
 * \param[in] options simulation options
 */
ModelSimuIndex::ModelSimuIndex(MbsData *mbs_data, SimuOptions *options)
{
	this->mbs_data    = mbs_data;
	this->options     = options;
	this->coman_model = options->coman_model;


	// -- Joints -- //

	// floating base
	joint_add(SimuJointIndex::FloatingT1);
	joint_add(SimuJointIndex::FloatingT2);
	joint_add(SimuJointIndex::FloatingT3);
	joint_add(SimuJointIndex::FloatingR1);
	joint_add(SimuJointIndex::FloatingR2);
	joint_add(SimuJointIndex::FloatingR3);

	// right leg
	joint_add(SimuJointIndex::RightHipPitch);
	joint_add(SimuJointIndex::RightHipRoll);
	joint_add(SimuJointIndex::RightHipYaw);
	joint_add(SimuJointIndex::RightKneePitch);
	joint_add(SimuJointIndex::RightFootRoll);
	joint_add(SimuJointIndex::RightFootPitch);

	if (coman_model == FLEX_FEET_COMAN)
	{
		joint_add(SimuJointIndex::RightFlexProx);
		joint_add(SimuJointIndex::RightFlexDist);
	}

	if((coman_model == SPRING_TOE_SHORT_FEET_COMAN) || (coman_model == SPRING_TOE_FEET_COMAN))
	{
		joint_add(SimuJointIndex::RightToePitch);
	}

	// left leg
	joint_add(SimuJointIndex::LeftHipPitch);
	joint_add(SimuJointIndex::LeftHipRoll);
	joint_add(SimuJointIndex::LeftHipYaw);
	joint_add(SimuJointIndex::LeftKneePitch);
	joint_add(SimuJointIndex::LeftFootRoll);
	joint_add(SimuJointIndex::LeftFootPitch);

	if (coman_model == FLEX_FEET_COMAN)
	{
		joint_add(SimuJointIndex::LeftFlexProx);
		joint_add(SimuJointIndex::LeftFlexDist);
	}

	if((coman_model == SPRING_TOE_SHORT_FEET_COMAN) || (coman_model == SPRING_TOE_FEET_COMAN))
	{
		joint_add(SimuJointIndex::LeftToePitch);
	}

	// torso
	joint_add(SimuJointIndex::TorsoRoll);
	joint_add(SimuJointIndex::TorsoPitch);
	joint_add(SimuJointIndex::TorsoYaw);

	// right arm
	joint_add(SimuJointIndex::RightShPitch);
	joint_add(SimuJointIndex::RightShRoll);
	joint_add(SimuJointIndex::RightShYaw);
	joint_add(SimuJointIndex::RightElbPitch);

	// left arm
	joint_add(SimuJointIndex::LeftShPitch);
	joint_add(SimuJointIndex::LeftShRoll);
	joint_add(SimuJointIndex::LeftShYaw);
	joint_add(SimuJointIndex::LeftElbPitch);

	// ball A
	if (coman_model == SHORT_FEET_BALL_COMAN)
	{
		joint_add(SimuJointIndex::Ball_A_T1);
		joint_add(SimuJointIndex::Ball_A_T2);
		joint_add(SimuJointIndex::Ball_A_T3);
		joint_add(SimuJointIndex::Ball_A_R1);
		joint_add(SimuJointIndex::Ball_A_R2);
		joint_add(SimuJointIndex::Ball_A_R3);
	}



	// -- Bodies -- //	

	// right leg
	body_add(SimuBodyIndex::RHipmot);
	body_add(SimuBodyIndex::RThighUpLeg);
	body_add(SimuBodyIndex::RThighLowLeg);
	body_add(SimuBodyIndex::RLowLeg);
	body_add(SimuBodyIndex::RFootmot);
	body_add(SimuBodyIndex::RFoot);

	if (coman_model == FLEX_FEET_COMAN)
	{
		body_add(SimuBodyIndex::RFlexPr);
		body_add(SimuBodyIndex::RFlexDs);
	}

	if((coman_model == SPRING_TOE_SHORT_FEET_COMAN) || (coman_model == SPRING_TOE_FEET_COMAN))
	{
		body_add(SimuBodyIndex::RFootDistal);
	}

	// left leg
	body_add(SimuBodyIndex::LHipmot);
	body_add(SimuBodyIndex::LThighUpLeg);
	body_add(SimuBodyIndex::LThighLowLeg);
	body_add(SimuBodyIndex::LLowLeg);
	body_add(SimuBodyIndex::LFootmot);
	body_add(SimuBodyIndex::LFoot);

	if (coman_model == FLEX_FEET_COMAN)
	{
		body_add(SimuBodyIndex::LFlexPr);
		body_add(SimuBodyIndex::LFlexDs);
	}

	if((coman_model == SPRING_TOE_SHORT_FEET_COMAN) || (coman_model == SPRING_TOE_FEET_COMAN))
	{
		body_add(SimuBodyIndex::LFootDistal);
	}

	// waist - torso
	body_add(SimuBodyIndex::Waist);
	body_add(SimuBodyIndex::DWL);
	body_add(SimuBodyIndex::DWS);
	body_add(SimuBodyIndex::DWYTorso);

	// right arm
	body_add(SimuBodyIndex::RShp);
	body_add(SimuBodyIndex::RShr);
	body_add(SimuBodyIndex::RShy);
	body_add(SimuBodyIndex::RElb);

	// left arm
	body_add(SimuBodyIndex::LShp);
	body_add(SimuBodyIndex::LShr);
	body_add(SimuBodyIndex::LShy);
	body_add(SimuBodyIndex::LElb);

	// balls
	if (coman_model == SHORT_FEET_BALL_COMAN)
	{
		body_add(SimuBodyIndex::Ball_A);
	}



	// -- S sensors -- //

	// mid-waist
	Ssens_add(SimuSsensIndex::MidWaist);

	// main part of the feet
	Ssens_add(SimuSsensIndex::RightFoot);
	Ssens_add(SimuSsensIndex::LeftFoot);

	// additional parts for the flexible feet
	if(coman_model == FLEX_FEET_COMAN)
	{
		Ssens_add(SimuSsensIndex::RightFlexProx);
		Ssens_add(SimuSsensIndex::RightFlexDist);
		Ssens_add(SimuSsensIndex::LeftFlexProx);
		Ssens_add(SimuSsensIndex::LeftFlexDist);
	}

	// legs
	Ssens_add(SimuSsensIndex::RThigh);
	Ssens_add(SimuSsensIndex::LThigh);
	Ssens_add(SimuSsensIndex::RLowLeg);
	Ssens_add(SimuSsensIndex::LLowLeg);

	if((coman_model == SPRING_TOE_SHORT_FEET_COMAN) || (coman_model == SPRING_TOE_FEET_COMAN))
	{
		Ssens_add(SimuSsensIndex::RToe);
		Ssens_add(SimuSsensIndex::LToe);
	}

	// torso
	Ssens_add(SimuSsensIndex::Torso);

	// arms
	Ssens_add(SimuSsensIndex::RShoulder);
	Ssens_add(SimuSsensIndex::LShoulder);
	Ssens_add(SimuSsensIndex::RElbow);
	Ssens_add(SimuSsensIndex::LElbow);

	// balls
	if (coman_model == SHORT_FEET_BALL_COMAN)
	{
		Ssens_add(SimuSsensIndex::Ball_A);
	}

	// com
	Ssens_add(SimuSsensIndex::WaistCg);
	Ssens_add(SimuSsensIndex::RHipmotCg);
	Ssens_add(SimuSsensIndex::RThighUpLegCg);
	Ssens_add(SimuSsensIndex::RThighLowLegCg);
	Ssens_add(SimuSsensIndex::RLowLegCg);
	Ssens_add(SimuSsensIndex::RFootmotCg);
	Ssens_add(SimuSsensIndex::RFootCg);
	Ssens_add(SimuSsensIndex::LHipmotCg);
	Ssens_add(SimuSsensIndex::LThighUpLegCg);
	Ssens_add(SimuSsensIndex::LThighLowLegCg);
	Ssens_add(SimuSsensIndex::LLowLegCg);
	Ssens_add(SimuSsensIndex::LFootmotCg);
	Ssens_add(SimuSsensIndex::LFootCg);
	Ssens_add(SimuSsensIndex::DWLCg);
	Ssens_add(SimuSsensIndex::DWSCg);
	Ssens_add(SimuSsensIndex::DWYCg);
	Ssens_add(SimuSsensIndex::RShpCg);
	Ssens_add(SimuSsensIndex::RShrCg);
	Ssens_add(SimuSsensIndex::RShyCg);
	Ssens_add(SimuSsensIndex::RElbCg);
	Ssens_add(SimuSsensIndex::LShpCg);
	Ssens_add(SimuSsensIndex::LShrCg);
	Ssens_add(SimuSsensIndex::LShyCg);
	Ssens_add(SimuSsensIndex::LElbCg);


	// -- F sensors -- //

	if (coman_model == FLEX_FEET_COMAN)
	{
		// flexible feet
		Fsens_add(SimuFsensIndex::RightFlexProx);
		Fsens_add(SimuFsensIndex::RightFlexDist);
		Fsens_add(SimuFsensIndex::LeftFlexProx);
		Fsens_add(SimuFsensIndex::LeftFlexDist);
	}
	else
	{
		// one part feet
		Fsens_add(SimuFsensIndex::RightFoot);
		Fsens_add(SimuFsensIndex::LeftFoot);
	}

	if ((coman_model == SPRING_TOE_SHORT_FEET_COMAN) || (coman_model == SPRING_TOE_FEET_COMAN))
	{
		Fsens_add(SimuFsensIndex::RightToe);
		Fsens_add(SimuFsensIndex::LeftToe);
	}

	// mid-waist
	Fsens_add(SimuFsensIndex::MidWaist);

	// legs
	Fsens_add(SimuFsensIndex::RThigh);
	Fsens_add(SimuFsensIndex::LThigh);
	Fsens_add(SimuFsensIndex::RLowLeg);
	Fsens_add(SimuFsensIndex::LLowLeg);

	// torso
	Fsens_add(SimuFsensIndex::Torso);

	// arms
	Fsens_add(SimuFsensIndex::RShoulder);
	Fsens_add(SimuFsensIndex::LShoulder);
	Fsens_add(SimuFsensIndex::RElbow);
	Fsens_add(SimuFsensIndex::LElbow);

	// balls
	if (coman_model == SHORT_FEET_BALL_COMAN)
	{
		Fsens_add(SimuFsensIndex::Ball_A);
	}



	// -- links -- //

	if (coman_model == FLEX_FEET_COMAN)
	{
		// flexible feet
		link_add(SimuLinkIndex::RightFlexProx);
		link_add(SimuLinkIndex::RightFlexDist);
		link_add(SimuLinkIndex::LeftFlexProx);
		link_add(SimuLinkIndex::LeftFlexDist);
	}


	// -- additional functions -- //

	check_sizes();
	mbs_indexes();
	inverse_create();
}

/*! \brief check the vector sizes
 */
void ModelSimuIndex::check_sizes()
{
	// joint_indexes
	if ((int) joint_indexes.size() != mbs_data->njoint)
	{
		std::cout << "Error: size of 'joint_indexes' (" << (int) joint_indexes.size()
			<< ") is different from 'mbs_data->njoint' (" << mbs_data->njoint << ") !" << std::endl;
		exit(EXIT_FAILURE);
	}

	// body_indexes
	if ((int) body_indexes.size() != mbs_data->nbody)
	{
		std::cout << "Error: size of 'body_indexes' (" << (int) body_indexes.size()
			<< ") is different from 'mbs_data->nbody' (" << mbs_data->nbody << ") !" << std::endl;
		exit(EXIT_FAILURE);
	}

	// Ssens_indexes
	if ((int) Ssens_indexes.size() != mbs_data->Nsensor + mbs_data->Nxfrc)
	{
		std::cout << "Error: size of 'Ssens_indexes' (" << (int) Ssens_indexes.size()
			<< ") is different from 'mbs_data->Nsensor (" << mbs_data->Nsensor << ") + mbs_data->Nxfrc' ("
			<< mbs_data->Nxfrc << ") !" << std::endl;
		exit(EXIT_FAILURE);
	}

	// Fsens_indexes
	if ((int) Fsens_indexes.size() != mbs_data->Nxfrc)
	{
		std::cout << "Error: size of 'Fsens_indexes' (" << (int) Fsens_indexes.size()
			<< ") is different from 'mbs_data->Nxfrc' (" << mbs_data->Nxfrc << ") !" << std::endl;
		exit(EXIT_FAILURE);
	}

	// link_indexes
	if ((int) link_indexes.size() != mbs_data->Nlink)
	{
		std::cout << "Error: size of 'link_indexes' (" << (int) link_indexes.size()
			<< ") is different from 'mbs_data->Nlink' (" << mbs_data->Nlink << ") !" << std::endl;
		exit(EXIT_FAILURE);
	}
}

/*! \brief get mbs indexes
 */
void ModelSimuIndex::mbs_indexes()
{
	// joint_indexes
	for(unsigned int i=0; i<joint_indexes.size(); i++)
	{
		mbs_joint_indexes.push_back(get_mbs_joint_index(joint_indexes[i]));
	}

	// body_indexes
	for(unsigned int i=0; i<body_indexes.size(); i++)
	{
		mbs_body_indexes.push_back(get_mbs_body_index(body_indexes[i]));
	}

	// Ssens_indexes
	for(unsigned int i=0; i<Ssens_indexes.size(); i++)
	{
		mbs_Ssens_indexes.push_back(get_mbs_Ssens_index(Ssens_indexes[i]));
	}

	// Fsens_indexes
	for(unsigned int i=0; i<Fsens_indexes.size(); i++)
	{
		mbs_Fsens_indexes.push_back(get_mbs_Fsens_index(Fsens_indexes[i]));
	}

	// link_indexes
	for(unsigned int i=0; i<link_indexes.size(); i++)
	{
		mbs_link_indexes.push_back(get_mbs_link_index(link_indexes[i]));
	}
}

/*! \brief create the inverse of the vectors
 */
void ModelSimuIndex::inverse_create()
{
	// indexes
	generate_inverse(joint_indexes, inv_joint_indexes);
	generate_inverse(body_indexes, inv_body_indexes);
	generate_inverse(Ssens_indexes, inv_Ssens_indexes);
	generate_inverse(Fsens_indexes, inv_Fsens_indexes);
	generate_inverse(link_indexes,  inv_link_indexes);

	// mbs indexes
	generate_inverse(mbs_joint_indexes, inv_mbs_joint_indexes);
	generate_inverse(mbs_body_indexes, inv_mbs_body_indexes);
	generate_inverse(mbs_Ssens_indexes, inv_mbs_Ssens_indexes);
	generate_inverse(mbs_Fsens_indexes, inv_mbs_Fsens_indexes);
	generate_inverse(mbs_link_indexes,  inv_mbs_link_indexes);
}

/*! \brief generate the inverse of a vector of indexes
 * 
 * \param[in] index_vec initial vector of indexes
 * \param[out] inv_index_vec vector inverted
 *
 * This inverse vector is a vector whose values are the indexes of the initial vector
 * where the index of this inverse vector is stored.
 */
void ModelSimuIndex::generate_inverse(std::vector<int>& index_vec, std::vector<int>& inv_index_vec)
{
	int index_max;
	int cur_index;

	index_max = -1;

	for(unsigned int i=0; i<index_vec.size(); i++)
	{
		cur_index = index_vec[i];

		if (cur_index > index_max)
		{
			index_max = cur_index;
		}
	}

	for(int i=0; i<=index_max; i++)
	{
		inv_index_vec.push_back(-1);
	}

	for(unsigned int i=0; i<index_vec.size(); i++)
	{
		cur_index = index_vec[i];
		inv_index_vec[cur_index] = i;
	}
}

/*! \brief destructor
 */
ModelSimuIndex::~ModelSimuIndex()
{

}
