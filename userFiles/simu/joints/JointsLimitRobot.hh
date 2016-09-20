/*! 
 * \author Nicolas Van der Noot
 * \file JointsLimitRobot.hh
 * \brief JointsLimitRobot class
 */

#ifndef _JOINTS_LIMIT_ROBOT_HH_
#define _JOINTS_LIMIT_ROBOT_HH_

#include "JointLimit.hh"
#include <vector>

/*! \brief all the robot joints limit
 */
class JointsLimitRobot
{
	public:
		JointsLimitRobot(MbsData *mbs_data, ModelSimuIndex *simu_index);
		~JointsLimitRobot();

		void check();

	private:
		std::vector<JointLimit*> joints;

		MbsData *mbs_data; ///< Robotran structure

		ModelSimuIndex *simu_index; ///< simulation indexes list

		int coman_model; ///< COMAN model

		void push_new_joint(int joint_id);
};

#endif
