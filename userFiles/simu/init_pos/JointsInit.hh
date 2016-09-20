/*! 
 * \author Bruno Somers
 * \file JointsInit.hh
 * \brief JointsInit class
 */

#ifndef _JOINTS_INIT_HH_
#define _JOINTS_INIT_HH_

extern "C" {
	#include "mbs_data.h"
}

#include "CtrlInputs.hh"

/*! \brief initials joints positions
 */
class JointsInit
{
	public:
		JointsInit(MbsData *mbs_data, CtrlInputs *inputs);
		virtual ~JointsInit();

		void set_joints_init();

		void set_T1_p(double value)      { T1_p = value; }
		void set_T3(double value)        { T3 = value; }
		void set_T3_p(double value)      { T3_p = value; }
		void set_R2(double value)        { R2 = value; }
		void set_R2_p(double value)      { R2_p = value; }
		void set_r_sh(double value)      { r_sh = value; }
		void set_r_sh_p(double value)    { r_sh_p = value; }
		void set_r_hip(double value)     { r_hip = value; }
		void set_r_hip_p(double value)   { r_hip_p = value; }
		void set_r_knee(double value)    { r_knee = value; }
		void set_r_knee_p(double value)  { r_knee_p = value; }
		void set_r_ankle(double value)   { r_ankle = value; }
		void set_r_ankle_p(double value) { r_ankle_p = value; }
		void set_l_hip(double value)     { l_hip = value; }
		void set_l_hip_p(double value)   { l_hip_p = value; }
		void set_l_knee(double value)    { l_knee = value; }
		void set_l_knee_p(double value)  { l_knee_p = value; }
		void set_l_ankle(double value)   { l_ankle = value; }
		void set_l_ankle_p(double value) { l_ankle_p = value; }

	private:
		MbsData *mbs_data; ///< Robotran structure
		CtrlInputs *inputs; ///< inputs Ctrl (used to set opti)

		int nb_mot;

		double T1_p; //initial forward velocity [m/s]
		double T3; //initial up position of base [m]
		double T3_p; //initial up velocity of base [m/s]
		double R2; //initial sagital angle of base [rad]
		double R2_p; //initial sagital angular velocity of base [rad/s]

		double r_sh; // initial sagital angle of shoulder [rad]
		double r_sh_p; // initial sagital angular velocity of shoulder [rad/s]

		double r_hip; //initial sagital angle of right hip [rad]
		double r_hip_p; //initial sagital angular velocity of righy hip [rad/s]
		double r_knee; //initial sagital angle of right knee [rad]
		double r_knee_p; //initial sagital angular velocity of right knee [rad/s]
		double r_ankle; //initial sagital angle of right ankle [rad]
		double r_ankle_p; //initial sagital angular velocity of right ankle [rad/s]

		double l_hip; //initial sagital angle of left hip [rad]
		double l_hip_p; //initial sagital angular velocity of left hip [rad/s]
		double l_knee; //initial sagital angle of left knee [rad]
		double l_knee_p; //initial sagital angular velocity of left knee [rad/s]
		double l_ankle; //initial sagital angle of left ankle [rad]
		double l_ankle_p; //initial sagital angular velocity of left ankle [rad/s]
};

#endif
