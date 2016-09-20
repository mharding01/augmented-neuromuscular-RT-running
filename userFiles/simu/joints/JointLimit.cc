#include "JointLimit.hh"
#include "SimuIndex.hh"
#include "user_IO.h"
#include <iostream>
#include <cmath>

#define RAD_TO_DEG (180.0/M_PI)

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] simu_index simulation indexes
 * \param[in] joint_id simulation joint ID
 * \param[in] q_min minimal joint range value [rad]
 * \param[in] q_max maximal joint range value [rad]
 */
JointLimit::JointLimit(MbsData *mbs_data, ModelSimuIndex *simu_index, int joint_id, double q_min, double q_max)
{
	this->mbs_data   = mbs_data;
	this->simu_index = simu_index;

	this->q_min = q_min;
	this->q_max = q_max;

	this->joint_id = joint_id;

	mbs_id = simu_index->get_mbs_jt(joint_id);

	q = mbs_data->q[mbs_id];

	options = mbs_data->user_IO->options;
}

/*! \brief destructor
 */
JointLimit::~JointLimit()
{

}

/*! \brief check if the joint is out of range
 * 
 * stop the simulation if the joint is out of range
 */
void JointLimit::check()
{
	q = mbs_data->q[mbs_id];

	// joints outside range
	if ( (q < q_min) || (q_max < q) )
	{
		if (options->print)
		{
			std::cout << "Stopping simulation: joint " << SimuJointIndex::get_index_name(joint_id) << " (" << q * RAD_TO_DEG 
				<< ") is out of range [" << q_min * RAD_TO_DEG << " ; " << q_max * RAD_TO_DEG << "] !" << std::endl;
		}

		mbs_data->flag_stop = 1; // stop the simulation
	}
}
