#include "JointsLimitRobot.hh"
#include "SimuIndex.hh"
#include "user_IO.h"
#include <cmath>
#include <iostream>

#define DEG_TO_RAD (M_PI/180.0)

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] simu_index simulation indexes 
 */
JointsLimitRobot::JointsLimitRobot(MbsData *mbs_data, ModelSimuIndex *simu_index)
{
	int cur_id;

	this->mbs_data   = mbs_data;
	this->simu_index = simu_index;

	coman_model = mbs_data->user_IO->options->coman_model;

	for(int i=0; i<simu_index->get_nb_joints(); i++)
	{
		cur_id = simu_index->get_joint_indexes(i);

		push_new_joint(cur_id);
	}
}

/*! \brief destructor
 */
JointsLimitRobot::~JointsLimitRobot()
{
	for(unsigned int i=0; i<joints.size(); i++)
	{
		delete joints[i];
	}
}

/*! \brief check all the joints limit ranges
 */
void JointsLimitRobot::check()
{
	for(unsigned int i = 0; i<joints.size(); i++)
	{
		joints[i]->check();
	}
}

/*! \brief push a new joint to check its limits
 * 
 * \param[in] ID of the joint (simulation ID)
 */
void JointsLimitRobot::push_new_joint(int joint_id)
{
	switch (joint_id)
	{
		// floating base
		case SimuJointIndex::FloatingT1 : break;
		case SimuJointIndex::FloatingT2 : break;
		case SimuJointIndex::FloatingT3 : break;
		case SimuJointIndex::FloatingR1 : break;
		case SimuJointIndex::FloatingR2 : break;
		case SimuJointIndex::FloatingR3 : break;
	
		// right leg
		case SimuJointIndex::RightHipPitch  : joints.push_back(new JointLimit(mbs_data, simu_index, joint_id, -120.0*DEG_TO_RAD,  70.0*DEG_TO_RAD)); break;
		case SimuJointIndex::RightHipRoll   : joints.push_back(new JointLimit(mbs_data, simu_index, joint_id,  -80.0*DEG_TO_RAD,  35.0*DEG_TO_RAD)); break;
		case SimuJointIndex::RightHipYaw    : joints.push_back(new JointLimit(mbs_data, simu_index, joint_id,  -50.0*DEG_TO_RAD,  50.0*DEG_TO_RAD)); break;
		case SimuJointIndex::RightKneePitch : joints.push_back(new JointLimit(mbs_data, simu_index, joint_id,  -15.0*DEG_TO_RAD, 120.0*DEG_TO_RAD)); break;
		case SimuJointIndex::RightFootRoll  : joints.push_back(new JointLimit(mbs_data, simu_index, joint_id,  -35.0*DEG_TO_RAD,  35.0*DEG_TO_RAD)); break;
		case SimuJointIndex::RightFootPitch : joints.push_back(new JointLimit(mbs_data, simu_index, joint_id,  -90.0*DEG_TO_RAD,  90.0*DEG_TO_RAD)); break;
		case SimuJointIndex::RightFlexProx  : joints.push_back(new JointLimit(mbs_data, simu_index, joint_id, -0.1, 0.0)); break;
		case SimuJointIndex::RightFlexDist  : joints.push_back(new JointLimit(mbs_data, simu_index, joint_id, -0.1, 0.0)); break;
		case SimuJointIndex::RightToePitch  : joints.push_back(new JointLimit(mbs_data, simu_index, joint_id, -45.0*DEG_TO_RAD,  45.0*DEG_TO_RAD)); break;

		// left leg
		case SimuJointIndex::LeftHipPitch  : joints.push_back(new JointLimit(mbs_data, simu_index, joint_id, -120.0*DEG_TO_RAD,  70.0*DEG_TO_RAD)); break;
		case SimuJointIndex::LeftHipRoll   : joints.push_back(new JointLimit(mbs_data, simu_index, joint_id,  -35.0*DEG_TO_RAD,  80.0*DEG_TO_RAD)); break;
		case SimuJointIndex::LeftHipYaw    : joints.push_back(new JointLimit(mbs_data, simu_index, joint_id,  -50.0*DEG_TO_RAD,  50.0*DEG_TO_RAD)); break;
		case SimuJointIndex::LeftKneePitch : joints.push_back(new JointLimit(mbs_data, simu_index, joint_id,  -15.0*DEG_TO_RAD, 120.0*DEG_TO_RAD)); break;
		case SimuJointIndex::LeftFootRoll  : joints.push_back(new JointLimit(mbs_data, simu_index, joint_id,  -35.0*DEG_TO_RAD,  35.0*DEG_TO_RAD)); break;
		case SimuJointIndex::LeftFootPitch : joints.push_back(new JointLimit(mbs_data, simu_index, joint_id,  -90.0*DEG_TO_RAD,  90.0*DEG_TO_RAD)); break;
		case SimuJointIndex::LeftFlexProx  : joints.push_back(new JointLimit(mbs_data, simu_index, joint_id, -0.1, 0.0)); break;
		case SimuJointIndex::LeftFlexDist  : joints.push_back(new JointLimit(mbs_data, simu_index, joint_id, -0.1, 0.0)); break;
		case SimuJointIndex::LeftToePitch  : joints.push_back(new JointLimit(mbs_data, simu_index, joint_id, -45.0*DEG_TO_RAD,  45.0*DEG_TO_RAD)); break;

		// torso
		case SimuJointIndex::TorsoRoll  : joints.push_back(new JointLimit(mbs_data, simu_index, joint_id, -50.0*DEG_TO_RAD, 50.0*DEG_TO_RAD)); break;
		case SimuJointIndex::TorsoPitch : joints.push_back(new JointLimit(mbs_data, simu_index, joint_id, -60.0*DEG_TO_RAD, 90.0*DEG_TO_RAD)); break;
		case SimuJointIndex::TorsoYaw   : joints.push_back(new JointLimit(mbs_data, simu_index, joint_id, -90.0*DEG_TO_RAD, 90.0*DEG_TO_RAD)); break;

		// right arm
		case SimuJointIndex::RightShPitch  : joints.push_back(new JointLimit(mbs_data, simu_index, joint_id,  -90.0*DEG_TO_RAD, 90.0*DEG_TO_RAD)); break;
		case SimuJointIndex::RightShYaw    : joints.push_back(new JointLimit(mbs_data, simu_index, joint_id,  -90.0*DEG_TO_RAD, 90.0*DEG_TO_RAD)); break;
		case SimuJointIndex::RightElbPitch : joints.push_back(new JointLimit(mbs_data, simu_index, joint_id, -120.0*DEG_TO_RAD, 45.0*DEG_TO_RAD)); break;
		case SimuJointIndex::RightShRoll   : joints.push_back(new JointLimit(mbs_data, simu_index, joint_id, -120.0*DEG_TO_RAD, 35.0*DEG_TO_RAD)); break;

		// left arm
		case SimuJointIndex::LeftShPitch  : joints.push_back(new JointLimit(mbs_data, simu_index, joint_id,  -90.0*DEG_TO_RAD,  90.0*DEG_TO_RAD)); break;
		case SimuJointIndex::LeftShYaw    : joints.push_back(new JointLimit(mbs_data, simu_index, joint_id,  -90.0*DEG_TO_RAD,  90.0*DEG_TO_RAD)); break;
		case SimuJointIndex::LeftElbPitch : joints.push_back(new JointLimit(mbs_data, simu_index, joint_id, -120.0*DEG_TO_RAD,  45.0*DEG_TO_RAD)); break;
		case SimuJointIndex::LeftShRoll   : joints.push_back(new JointLimit(mbs_data, simu_index, joint_id,  -35.0*DEG_TO_RAD, 120.0*DEG_TO_RAD));	break;

		// ball
		case SimuJointIndex::Ball_A_T1 : break;
		case SimuJointIndex::Ball_A_T2 : break;
		case SimuJointIndex::Ball_A_T3 : break;
		case SimuJointIndex::Ball_A_R1 : break;
		case SimuJointIndex::Ball_A_R2 : break;
		case SimuJointIndex::Ball_A_R3 : break;

		default:
			std::cout << "Error: unknow limit for joint " << joint_id << " !" << std::endl;
			exit(EXIT_FAILURE);
			break;
	}
}

