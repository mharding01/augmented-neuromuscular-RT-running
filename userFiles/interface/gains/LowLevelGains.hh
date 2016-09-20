/*! 
 * \author Nicolas Van der Noot
 * \file low_level_gains.hh
 * \brief LowLevelGains class
 */

#ifndef _LOW_LEVEL_GAINS_HH_
#define _LOW_LEVEL_GAINS_HH_

enum {POSITION_TRACKING, TORQUE_TRACKING};

#include "RobotActuators.hh"

/*! \brief low-level impedance controller gains
 */
class LowLevelGains
{
	public:
		LowLevelGains(RobotActuators *actuators);
		~LowLevelGains();

		void set_gains_position(int joint_id);
		void set_gains_torque(int joint_id);
		int motor_config(int joint_id);

	private:
		RobotActuators *actuators; ///< robot actuators
};

#endif
