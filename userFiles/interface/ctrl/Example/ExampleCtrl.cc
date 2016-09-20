/*! 
 * \author Nicolas Van der Noot
 * \file ExampleCtrl.cc
 * \brief ExampleCtrl class
 */

#include "ExampleCtrl.hh"

#include <iostream>
#include "user_realtime.h"

// -- Maximal user inputs -- //

#define MIN_INPUT -16
#define MAX_INPUT  36

inline double limit(double x, double min, double max){ return (x < min) ? min : (x > max) ? max : x; }

// -- Motor indexes -- //

namespace MotIndex
{
	enum JointIndexes
	{
		// right leg
		RightHipPitch  = 3,
		RightHipRoll   = 5,
		RightHipYaw    = 6,
		RightKneePitch = 7,
		RightFootRoll  = 9,
		RightFootPitch = 8,

		// left leg
		LeftHipPitch  = 4,
		LeftHipRoll   = 10,
		LeftHipYaw    = 11,
		LeftKneePitch = 12,
		LeftFootRoll  = 14,
		LeftFootPitch = 13,
		
		// torso
		TorsoRoll  = 2,
		TorsoPitch = 1,
		TorsoYaw   = 0,

		// right arm
		RightShPitch  = 15,
		RightShRoll   = 16,
		RightShYaw    = 17,
		RightElbPitch = 18,

		// left arm
		LeftShPitch  = 19,
		LeftShRoll   = 20,
		LeftShYaw    = 21,
		LeftElbPitch = 22,
	};
};

/*! \brief constructor
 */
ExampleCtrl::ExampleCtrl(): Ctrl(EXAMPLE_CTRL)
{
	// number of motors
	nb_mot = 23;

	/*
	 * The following function must be coherent with the
	 * 'userFiles/simu/actuators/LowLevelGains.cc' file
	 * (modify its 'motor_config' function).
	 * In this example, all the joints must be in 'POSITION_TRACKING',
	 * except the arm joints which can be in 'POSITION_TRACKING' or in
	 * 'TORQUE_TRACKING', according to the following 'torque_ctrl' flag
	 * (set 'torque_ctrl' to 1 to track torque references for the arms,
	 * 0 to track position references).
	 */
	torque_ctrl = 0;

	// proportional gain
	kp_tor = 10.0;

	// time input
	t = 0.0;

	// joint inputs
	q.resize(nb_mot);
	qd.resize(nb_mot);
	Qq.resize(nb_mot);
	q_mot.resize(nb_mot);
	qd_mot.resize(nb_mot);

	// ground contact forces inputs
	F_Rfoot.resize(3);
	F_Lfoot.resize(3);
	T_Rfoot.resize(3);
	T_Lfoot.resize(3);

	// IMU inputs
	IMU_Angular_Rate.resize(3);
	IMU_Acceleration.resize(3);
	IMU_Orientation.resize(9);

	// joints outputs
	q_ref.resize(nb_mot);
	Qq_ref.resize(nb_mot);
}

/*! \brief desctructor
 */
ExampleCtrl::~ExampleCtrl()
{

}

/*! \brief controller initialization
 */
void ExampleCtrl::init_ctrl()
{

}

/*! \brief controller main loop
 */
void ExampleCtrl::loop_ctrl()
{
	// variables declaration
	int r_keyboard, l_keyboard;

	double r_sh_pitch_ref, l_sh_pitch_ref;
	double r_sh_yaw_ref, l_sh_yaw_ref;
	double r_elb_ref, l_elb_ref;

	// get keyboard inputs
	r_keyboard = user_ctrl->get_keyboard_input(0, 0); ///< right keyboard input [-]
	l_keyboard = user_ctrl->get_keyboard_input(0, 1); ///< left keyboard input [-]

	// limiting inputs
	r_keyboard = limit(r_keyboard, MIN_INPUT, MAX_INPUT);
	l_keyboard = limit(l_keyboard, MIN_INPUT, MAX_INPUT);

	// save limited keyboard input
	user_ctrl->set_keyboard_input(r_keyboard, 0, 0);
	user_ctrl->set_keyboard_input(l_keyboard, 0, 1);

	// shoulders pitch
	r_sh_pitch_ref = -0.018 * r_keyboard; ///< right sh pitch position reference [rad]
	l_sh_pitch_ref = -0.018 * l_keyboard; ///< left sh pitch position reference [rad]

	// shoulders yaw
	r_sh_yaw_ref =  0.021 * r_keyboard; ///< right sh yaw position reference [rad]
	l_sh_yaw_ref = -0.021 * l_keyboard; ///< left sh yaw position reference [rad]

	// elbows
	r_elb_ref = -0.036 * r_keyboard; ///< right elbow position reference [rad]
	l_elb_ref = -0.036 * l_keyboard; ///< left elbow position reference [rad]

	if (torque_ctrl) // arms in torque control
	{
		Qq_ref[MotIndex::RightShPitch]  = kp_tor * ( r_sh_pitch_ref - q[MotIndex::RightShPitch]  );
		Qq_ref[MotIndex::LeftShPitch]   = kp_tor * ( l_sh_pitch_ref - q[MotIndex::LeftShPitch]   );
		Qq_ref[MotIndex::RightShYaw]    = kp_tor * ( r_sh_yaw_ref   - q[MotIndex::RightShYaw]    );
		Qq_ref[MotIndex::LeftShYaw]     = kp_tor * ( l_sh_yaw_ref   - q[MotIndex::LeftShYaw]     );
		Qq_ref[MotIndex::RightElbPitch] = kp_tor * ( r_elb_ref      - q[MotIndex::RightElbPitch] );
		Qq_ref[MotIndex::LeftElbPitch]  = kp_tor * ( l_elb_ref      - q[MotIndex::LeftElbPitch]  );

		// graphs
		set_plot(Qq_ref[MotIndex::RightShPitch], "Qq right ref [Nm]");
		set_plot(Qq[MotIndex::RightShPitch], "Qq right real [Nm]");

		set_plot(Qq_ref[MotIndex::LeftShPitch], "Qq left ref [Nm]");
		set_plot(Qq[MotIndex::LeftShPitch], "Qq left real [Nm]");
	}
	else // arms in position control
	{
		q_ref[MotIndex::RightShPitch]  = r_sh_pitch_ref;
		q_ref[MotIndex::LeftShPitch]   = l_sh_pitch_ref;
		q_ref[MotIndex::RightShYaw]    = r_sh_yaw_ref;
		q_ref[MotIndex::LeftShYaw]     = l_sh_yaw_ref;
		q_ref[MotIndex::RightElbPitch] = r_elb_ref;
		q_ref[MotIndex::LeftElbPitch]  = l_elb_ref;

		// graphs
		set_plot(q_ref[MotIndex::RightShPitch], "q right ref [rad]");
		set_plot(q[MotIndex::RightShPitch], "q right real [rad]");

		set_plot(q_ref[MotIndex::LeftShPitch], "q left ref [rad]");
		set_plot(q[MotIndex::LeftShPitch], "q left real [rad]");
	}
}

/*! \brief controller finalization
 */
void ExampleCtrl::finish_ctrl()
{

}

/*! \brief controller inputs filling
 * 
 * \param[in] inputsCtrl controller inputs structure
 *
 * For more information about the controller inputs, src/project/interface_controller/controller_io.hh.
 * Fill your controller inputs, reading the ones from 'inputsCtrl'.
 */
void ExampleCtrl::set_inputs(Inputs_ctrl *inputsCtrl)
{
	// time
	t = inputsCtrl->t;

	// positions - velocities - torques
	for (int i=0; i<nb_mot; i++)
	{
		q[i]  = inputsCtrl->q[i];
		qd[i] = inputsCtrl->qd[i];
		Qq[i] = inputsCtrl->Qq[i];

		q_mot[i]  = inputsCtrl->q_mot[i];
		qd_mot[i] = inputsCtrl->qd_mot[i];
	}

	// feet (forces - torques) and IMU
	for (int i=0; i<3; i++)
	{
		F_Rfoot[i] = inputsCtrl->F_Rfoot[i];
		F_Lfoot[i] = inputsCtrl->F_Lfoot[i];
		T_Rfoot[i] = inputsCtrl->T_Rfoot[i];
		T_Lfoot[i] = inputsCtrl->T_Lfoot[i];

		IMU_Angular_Rate[i] = inputsCtrl->IMU_Angular_Rate[i];
		IMU_Acceleration[i] = inputsCtrl->IMU_Acceleration[i];
	}

	// IMU orientation
	for(int i=0; i<9; i++)
	{
		IMU_Orientation[i] = inputsCtrl->IMU_Orientation[i];
	}
}

/*! \brief controller outputs filling
 * 
 * \param[out] outputsCtrl controller outputs structure
 *
 * For more information about the controller outputs, src/project/interface_controller/controller_io.hh.
 * Fill 'outputsCtrl' with your controller outputs.
 */
void ExampleCtrl::set_outputs(Outputs_ctrl *outputsCtrl)
{
	// tracking (position - velocity - torque)
	for (int i=0; i<nb_mot; i++)
	{
		outputsCtrl->q_ref[i]   = q_ref[i];
		outputsCtrl->Qq_ref[i]  = Qq_ref[i];
	}
}
