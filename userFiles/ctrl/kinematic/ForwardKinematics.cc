
#include "ForwardKinematics.hh"
#include "CtrlIndex.hh"
#include "hardcoded_model.h"
#include "InverseKinLeg.hh"
#include <cmath>

// joints enumeration
enum {RIGHT_HIP_PITCH, RIGHT_HIP_ROLL, RIGHT_HIP_YAW, RIGHT_KNEE_PITCH, RIGHT_FOOT_ROLL,
	  RIGHT_FOOT_PITCH, LEFT_HIP_PITCH, LEFT_HIP_ROLL, LEFT_HIP_YAW, LEFT_KNEE_PITCH, LEFT_FOOT_ROLL,
	  LEFT_FOOT_PITCH, TORSO_ROLL, TORSO_PITCH, TORSO_YAW, RIGHT_SH_PITCH, RIGHT_SH_ROLL,
	  RIGHT_SH_YAW, RIGHT_ELB_PITCH, LEFT_SH_PITCH, LEFT_SH_ROLL, LEFT_SH_YAW, LEFT_ELB_PITCH};

/*! \brief constructor
 */
KinematicsInOut::KinematicsInOut(int nb_mot)
{
	// -- inputs -- //

	this->nb_mot = nb_mot;

	for(int i=0; i<nb_mot; i++)
	{
		q_mot.push_back(0.0);
		qd_mot.push_back(0.0);
	}

	for(int i=0; i<9; i++)
	{
		IMU_Orientation[i] = 0.0;
	}

	for(int i=0; i<3; i++)
	{
		IMU_Angular_Rate[i] = 0.0;
	}


	// -- outputs -- //

	for(int i=0; i<3; i++)
	{
		// center of mass
		r_COM[i] = 0.0;
		rp_COM[i] = 0.0;

		// feet position
		r_Rfoot[i] = 0.0;
		r_Lfoot[i] = 0.0;

		// feet velocity
		rp_Rfoot[i] = 0.0;
		rp_Lfoot[i] = 0.0;

		// wrists position
		r_Rwrist[i] = 0.0;
		r_Lwrist[i] = 0.0;

		// wrists velocity
		rp_Rwrist[i] = 0.0;
		rp_Lwrist[i] = 0.0;

		// rotation matrices as angles
		theta_waist[i] = 0.0;
		theta_torso[i] = 0.0;
		theta_Rfoot[i] = 0.0;
		theta_Lfoot[i] = 0.0;

		omega_waist[i] = 0.0;
		omega_torso[i] = 0.0;
		omega_Rfoot[i] = 0.0;
		omega_Lfoot[i] = 0.0;
	}

	for(int i=0; i<9; i++)
	{
		// feet absolute orientation
		Rfoot_or[i] = 0.0;
		Lfoot_or[i] = 0.0;

		// wrists absolute orientation
		Rwrist_or[i] = 0.0;
		Lwrist_or[i] = 0.0;
	}

	Rfoot_or[0] = 1.0;
	Rfoot_or[4] = 1.0;
	Rfoot_or[8] = 1.0;

	Lfoot_or[0] = 1.0;
	Lfoot_or[4] = 1.0;
	Lfoot_or[8] = 1.0;

	Rwrist_or[0] = 1.0;
	Rwrist_or[4] = 1.0;
	Rwrist_or[8] = 1.0;

	Lwrist_or[0] = 1.0;
	Lwrist_or[4] = 1.0;
	Lwrist_or[8] = 1.0;

	// jacobian related
	for(int i=0; i<NB_MOTORS; i++)
	{
		for(int j=0; j<3; j++)
		{
			r_COM_der[i][j] = 0.0;

			r_Rfoot_der[i][j] = 0.0;
			r_Lfoot_der[i][j] = 0.0;

			r_Rwrist_der[i][j] = 0.0;
			r_Lwrist_der[i][j] = 0.0;
		}

		for(int j=0; j<9; j++)
		{
			Rfoot_or_der[i][j] = 0.0;
			Lfoot_or_der[i][j] = 0.0;

			Rwrist_or_der[i][j] = 0.0;
			Lwrist_or_der[i][j] = 0.0;
		}
	}

	// feet contact realted
	for(int i=0; i<4; i++)
	{
		for(int j=0; j<3; j++)
		{
			r_Rfoot_cont[i][j] = 0.0;
			r_Lfoot_cont[i][j] = 0.0;
		}

		for(int j=0; j<NB_MOTORS; j++)
		{
			for(int k=0; k<3; k++)
			{
				r_Rfoot_cont_der[i][j][k] = 0.0;
				r_Lfoot_cont_der[i][j][k] = 0.0;
			}
		}
	}
}

/*! \brief destructor
 */
KinematicsInOut::~KinematicsInOut()
{

}

/*! \brief constructor
 * 
 * \param[in] inputs controller inputs
 * \param[in] options controller options
 * \param[in] ctrl_index controller index lists
 */
ForwardKinematics::ForwardKinematics(CtrlInputs *inputs, CtrlOptions *options, MotorCtrlIndex *ctrl_index, WalkStates *ws):
	gen_in_out(inputs->get_nb_mot()), Computation(inputs, options, ctrl_index)
{
	this->ws = ws;
	ms = static_cast<MainState*>(ws->get_state(MAIN_STATE));

	// jacobian computation
	flag_jacob = 0;

	// inverse kinematics default
	r_leg_inverse = NULL;
	l_leg_inverse = NULL;

	// i/o vector IDs
	RightHipPitch_id  = ctrl_index->get_inv_index(CtrlIndex::RightHipPitch);
	RightHipRoll_id   = ctrl_index->get_inv_index(CtrlIndex::RightHipRoll);
	RightHipYaw_id    = ctrl_index->get_inv_index(CtrlIndex::RightHipYaw);
	RightKneePitch_id = ctrl_index->get_inv_index(CtrlIndex::RightKneePitch);
	RightFootRoll_id  = ctrl_index->get_inv_index(CtrlIndex::RightFootRoll);
	RightFootPitch_id = ctrl_index->get_inv_index(CtrlIndex::RightFootPitch);

	LeftHipPitch_id  = ctrl_index->get_inv_index(CtrlIndex::LeftHipPitch);
	LeftHipRoll_id   = ctrl_index->get_inv_index(CtrlIndex::LeftHipRoll);
	LeftHipYaw_id    = ctrl_index->get_inv_index(CtrlIndex::LeftHipYaw);
	LeftKneePitch_id = ctrl_index->get_inv_index(CtrlIndex::LeftKneePitch);
	LeftFootRoll_id  = ctrl_index->get_inv_index(CtrlIndex::LeftFootRoll);
	LeftFootPitch_id = ctrl_index->get_inv_index(CtrlIndex::LeftFootPitch);

	TorsoRoll_id  = ctrl_index->get_inv_index(CtrlIndex::TorsoRoll);
	TorsoPitch_id = ctrl_index->get_inv_index(CtrlIndex::TorsoPitch);
	TorsoYaw_id   = ctrl_index->get_inv_index(CtrlIndex::TorsoYaw);

	RightShPitch_id  = ctrl_index->get_inv_index(CtrlIndex::RightShPitch);
	RightShRoll_id   = ctrl_index->get_inv_index(CtrlIndex::RightShRoll);
	RightShYaw_id    = ctrl_index->get_inv_index(CtrlIndex::RightShYaw);
	RightElbPitch_id = ctrl_index->get_inv_index(CtrlIndex::RightElbPitch);

	LeftShPitch_id  = ctrl_index->get_inv_index(CtrlIndex::LeftShPitch);
	LeftShRoll_id   = ctrl_index->get_inv_index(CtrlIndex::LeftShRoll);
	LeftShYaw_id    = ctrl_index->get_inv_index(CtrlIndex::LeftShYaw);
	LeftElbPitch_id = ctrl_index->get_inv_index(CtrlIndex::LeftElbPitch);

	for(int i=0; i<3; i++)
	{
		r_COM_Rfoot[i] = 0.0;
		r_COM_Lfoot[i] = 0.0;
		r_COM_feet[i]  = 0.0;

		rp_COM_Rfoot[i] = 0.0;
		rp_COM_Lfoot[i] = 0.0;
		rp_COM_feet[i]  = 0.0;

		r_COM_Rfoot_yaw[i] = 0.0;
		r_COM_Lfoot_yaw[i] = 0.0;
		r_COM_feet_yaw[i]  = 0.0;

		rp_COM_Rfoot_yaw[i] = 0.0;
		rp_COM_Lfoot_yaw[i] = 0.0;
		rp_COM_feet_yaw[i]  = 0.0;

		rp_COM[i] = 0.0;
	}

	r_leg_inverse = new InverseKinLeg(this, ctrl_index, inputs, 6, R_ID);
	l_leg_inverse = new InverseKinLeg(this, ctrl_index, inputs, 6, L_ID);

	inverse_kin.push_back(r_leg_inverse);
	inverse_kin.push_back(l_leg_inverse);
}

/*! \brief destructor
 */
ForwardKinematics::~ForwardKinematics()
{
	for(unsigned int i=0; i<inverse_kin.size(); i++)
	{
		delete inverse_kin[i];
	}
}

/*! \brief forward kinematics computation
 */
void ForwardKinematics::compute()
{
	state_check();
	fill_gen_inputs(gen_in_out);
	main_kinematics(gen_in_out);
	foot_abs_or();
	force_torque_inertial_frame();
	torso_orientation();
	com_feet();

	for (int i=0; i<3; i++)
	{
		rp_COM[i] = gen_in_out.rp_COM[i];
	}
}

/*! \brief checks related to the robot state<s
 */
void ForwardKinematics::state_check()
{
	switch (ms->get_coman_state())
	{
		case INIT_UPRIGHT_STATE:
			flag_jacob = 1;
			break;
	
		default:
			flag_jacob = 0;
			break;
	}
}

/*! \brief fill the generic inputs
 *
 * \param[in,out] in_out inputs and outputs class
 */
void ForwardKinematics::fill_gen_inputs(KinematicsInOut &in_out)
{
	// IMU - rotation matrices
	for(int i=0; i<9; i++)
	{
		in_out.IMU_Orientation[i] = inputs->get_IMU_Orientation(i);
	}

	// IMU - angles velocity
	for(int i=0; i<3; i++)
	{
		in_out.IMU_Angular_Rate[i] = inputs->get_IMU_Angular_Rate(i);
	}

	// motor joints
	for(int i=0; i<in_out.nb_mot; i++)
	{
		in_out.q_mot[i]  = inputs->get_q_mot(i);
		in_out.qd_mot[i] = inputs->get_qd_mot(i);
	}
}

/*! \brief compute the feet roll-pitch-yaw angles and derivatives
 */
void ForwardKinematics::foot_abs_or()
{
	for(int i=0; i<3; i++)
	{
		inputs->set_theta_Rfoot(i, gen_in_out.theta_Rfoot[i]);
		inputs->set_theta_Lfoot(i, gen_in_out.theta_Lfoot[i]);

		inputs->set_omega_Rfoot(i, gen_in_out.omega_Rfoot[i]);
		inputs->set_omega_Lfoot(i, gen_in_out.omega_Lfoot[i]);
	}
}

/*! \brief get feet forces and torques in the initial frame
 */
void ForwardKinematics::force_torque_inertial_frame()
{
	// right foot forces
	inputs->set_F_Rfoot_IF(0, gen_in_out.Rfoot_or[0]*inputs->get_F_Rfoot(0) + gen_in_out.Rfoot_or[1]*inputs->get_F_Rfoot(1) + gen_in_out.Rfoot_or[2]*inputs->get_F_Rfoot(2));
	inputs->set_F_Rfoot_IF(1, gen_in_out.Rfoot_or[3]*inputs->get_F_Rfoot(0) + gen_in_out.Rfoot_or[4]*inputs->get_F_Rfoot(1) + gen_in_out.Rfoot_or[5]*inputs->get_F_Rfoot(2));
	inputs->set_F_Rfoot_IF(2, gen_in_out.Rfoot_or[6]*inputs->get_F_Rfoot(0) + gen_in_out.Rfoot_or[7]*inputs->get_F_Rfoot(1) + gen_in_out.Rfoot_or[8]*inputs->get_F_Rfoot(2));

	// left foot forces
	inputs->set_F_Lfoot_IF(0, gen_in_out.Lfoot_or[0]*inputs->get_F_Lfoot(0) + gen_in_out.Lfoot_or[1]*inputs->get_F_Lfoot(1) + gen_in_out.Lfoot_or[2]*inputs->get_F_Lfoot(2));
	inputs->set_F_Lfoot_IF(1, gen_in_out.Lfoot_or[3]*inputs->get_F_Lfoot(0) + gen_in_out.Lfoot_or[4]*inputs->get_F_Lfoot(1) + gen_in_out.Lfoot_or[5]*inputs->get_F_Lfoot(2));
	inputs->set_F_Lfoot_IF(2, gen_in_out.Lfoot_or[6]*inputs->get_F_Lfoot(0) + gen_in_out.Lfoot_or[7]*inputs->get_F_Lfoot(1) + gen_in_out.Lfoot_or[8]*inputs->get_F_Lfoot(2));

	// right foot torques
	inputs->set_T_Rfoot_IF(0, gen_in_out.Rfoot_or[0]*inputs->get_T_Rfoot(0) + gen_in_out.Rfoot_or[1]*inputs->get_T_Rfoot(1) + gen_in_out.Rfoot_or[2]*inputs->get_T_Rfoot(2));
	inputs->set_T_Rfoot_IF(1, gen_in_out.Rfoot_or[3]*inputs->get_T_Rfoot(0) + gen_in_out.Rfoot_or[4]*inputs->get_T_Rfoot(1) + gen_in_out.Rfoot_or[5]*inputs->get_T_Rfoot(2));
	inputs->set_T_Rfoot_IF(2, gen_in_out.Rfoot_or[6]*inputs->get_T_Rfoot(0) + gen_in_out.Rfoot_or[7]*inputs->get_T_Rfoot(1) + gen_in_out.Rfoot_or[8]*inputs->get_T_Rfoot(2));

	// left foot torques
	inputs->set_T_Lfoot_IF(0, gen_in_out.Lfoot_or[0]*inputs->get_T_Lfoot(0) + gen_in_out.Lfoot_or[1]*inputs->get_T_Lfoot(1) + gen_in_out.Lfoot_or[2]*inputs->get_T_Lfoot(2));
	inputs->set_T_Lfoot_IF(1, gen_in_out.Lfoot_or[3]*inputs->get_T_Lfoot(0) + gen_in_out.Lfoot_or[4]*inputs->get_T_Lfoot(1) + gen_in_out.Lfoot_or[5]*inputs->get_T_Lfoot(2));
	inputs->set_T_Lfoot_IF(2, gen_in_out.Lfoot_or[6]*inputs->get_T_Lfoot(0) + gen_in_out.Lfoot_or[7]*inputs->get_T_Lfoot(1) + gen_in_out.Lfoot_or[8]*inputs->get_T_Lfoot(2));
}

/*! \brief compute the torso orientation (and derivative)
 */
void ForwardKinematics::torso_orientation()
{
	for(int i=0; i<3; i++)
	{
		inputs->set_theta_waist(i, gen_in_out.theta_waist[i]);
		inputs->set_omega_waist(i, gen_in_out.omega_waist[i]);
		inputs->set_theta_torso(i, gen_in_out.theta_torso[i]);
		inputs->set_omega_torso(i, gen_in_out.omega_torso[i]);
	}
}

/*! \brief compute the com position relative to the feet
 *         in the IMU frame and in a frame aligned with
 *         the robot forward direction (i.e. yaw angle is zero)
 */
void ForwardKinematics::com_feet()
{
	double omega_z, cos_z, sin_z;

	// int the IMU frame
	for(int i=0; i<3; i++)
	{
		r_COM_Rfoot[i] = gen_in_out.r_COM[i] - gen_in_out.r_Rfoot[i];
		r_COM_Lfoot[i] = gen_in_out.r_COM[i] - gen_in_out.r_Lfoot[i];
		r_COM_feet[i]  = gen_in_out.r_COM[i] - 0.5 * (gen_in_out.r_Rfoot[i] + gen_in_out.r_Lfoot[i]);

		rp_COM_Rfoot[i] = gen_in_out.rp_COM[i] - gen_in_out.rp_Rfoot[i];
		rp_COM_Lfoot[i] = gen_in_out.rp_COM[i] - gen_in_out.rp_Lfoot[i];
		rp_COM_feet[i]  = gen_in_out.rp_COM[i] - 0.5 * (gen_in_out.rp_Rfoot[i] + gen_in_out.rp_Lfoot[i]);
	}

	omega_z = gen_in_out.omega_waist[2];

	cos_z = cos(gen_in_out.theta_waist[2]);
	sin_z = sin(gen_in_out.theta_waist[2]);

	// distances in the robot forward frame
	r_COM_Rfoot_yaw[0] =  cos_z*r_COM_Rfoot[0] + sin_z*r_COM_Rfoot[1];
	r_COM_Rfoot_yaw[1] = -sin_z*r_COM_Rfoot[0] + cos_z*r_COM_Rfoot[1];
	r_COM_Rfoot_yaw[2] =  r_COM_Rfoot[2];

	r_COM_Lfoot_yaw[0] =  cos_z*r_COM_Lfoot[0] + sin_z*r_COM_Lfoot[1];
	r_COM_Lfoot_yaw[1] = -sin_z*r_COM_Lfoot[0] + cos_z*r_COM_Lfoot[1];
	r_COM_Lfoot_yaw[2] =  r_COM_Lfoot[2];

	r_COM_feet_yaw[0] =  cos_z*r_COM_feet[0] + sin_z*r_COM_feet[1];
	r_COM_feet_yaw[1] = -sin_z*r_COM_feet[0] + cos_z*r_COM_feet[1];
	r_COM_feet_yaw[2] =  r_COM_feet[2];

	// derivatives in the robot forward frame
	rp_COM_Rfoot_yaw[0] = -sin_z*omega_z*r_COM_Rfoot[0] + cos_z*rp_COM_Rfoot[0] + cos_z*omega_z*r_COM_Rfoot[1] + sin_z*rp_COM_Rfoot[1];
	rp_COM_Rfoot_yaw[1] = -cos_z*omega_z*r_COM_Rfoot[0] - sin_z*rp_COM_Rfoot[0] - sin_z*omega_z*r_COM_Rfoot[1] + cos_z*rp_COM_Rfoot[1];
	rp_COM_Rfoot_yaw[2] =  rp_COM_Rfoot[2];

	rp_COM_Lfoot_yaw[0] = -sin_z*omega_z*r_COM_Lfoot[0] + cos_z*rp_COM_Lfoot[0] + cos_z*omega_z*r_COM_Lfoot[1] + sin_z*rp_COM_Lfoot[1];
	rp_COM_Lfoot_yaw[1] = -cos_z*omega_z*r_COM_Lfoot[0] - sin_z*rp_COM_Lfoot[0] - sin_z*omega_z*r_COM_Lfoot[1] + cos_z*rp_COM_Lfoot[1];
	rp_COM_Lfoot_yaw[2] =  rp_COM_Lfoot[2];

	rp_COM_feet_yaw[0] = -sin_z*omega_z*r_COM_feet[0] + cos_z*rp_COM_feet[0] + cos_z*omega_z*r_COM_feet[1] + sin_z*rp_COM_feet[1];
	rp_COM_feet_yaw[1] = -cos_z*omega_z*r_COM_feet[0] - sin_z*rp_COM_feet[0] - sin_z*omega_z*r_COM_feet[1] + cos_z*rp_COM_feet[1];
	rp_COM_feet_yaw[2] =  rp_COM_feet[2];
}

/*! \brief get the forward kinematics index
 * 
 * \param[in] ctrl_index index from the controller
 * \return forward kinematics index
 */
int ForwardKinematics::get_fwd_kin_index(int ctrl_index)
{
	switch (ctrl_index)
	{
		// right leg
		case CtrlIndex::RightHipPitch  : return RIGHT_HIP_PITCH;
		case CtrlIndex::RightHipRoll   : return RIGHT_HIP_ROLL;
		case CtrlIndex::RightHipYaw    : return RIGHT_HIP_YAW;
		case CtrlIndex::RightKneePitch : return RIGHT_KNEE_PITCH;
		case CtrlIndex::RightFootRoll  : return RIGHT_FOOT_ROLL;
		case CtrlIndex::RightFootPitch : return RIGHT_FOOT_PITCH;

		// left leg
		case CtrlIndex::LeftHipPitch  : return LEFT_HIP_PITCH;
		case CtrlIndex::LeftHipRoll   : return LEFT_HIP_ROLL;
		case CtrlIndex::LeftHipYaw    : return LEFT_HIP_YAW;
		case CtrlIndex::LeftKneePitch : return LEFT_KNEE_PITCH;
		case CtrlIndex::LeftFootRoll  : return LEFT_FOOT_ROLL;
		case CtrlIndex::LeftFootPitch : return LEFT_FOOT_PITCH;

		// torso
		case CtrlIndex::TorsoRoll  : return TORSO_ROLL;
		case CtrlIndex::TorsoPitch : return TORSO_PITCH;
		case CtrlIndex::TorsoYaw   : return TORSO_YAW;

		// right arm
		case CtrlIndex::RightShPitch  : return RIGHT_SH_PITCH;
		case CtrlIndex::RightShRoll   : return RIGHT_SH_ROLL;
		case CtrlIndex::RightShYaw    : return RIGHT_SH_YAW;
		case CtrlIndex::RightElbPitch : return RIGHT_ELB_PITCH;

		// left arm
		case CtrlIndex::LeftShPitch  : return LEFT_SH_PITCH;
		case CtrlIndex::LeftShRoll   : return LEFT_SH_ROLL;
		case CtrlIndex::LeftShYaw    : return LEFT_SH_YAW;
		case CtrlIndex::LeftElbPitch : return LEFT_ELB_PITCH;

		default:
			std::cout << "Error: unknown controller index " << ctrl_index << " !" << std::endl;
			exit(EXIT_FAILURE);
			break;
	}
}

/*! \brief get the contorller index
 * 
 * \param[in] ctrl_index index from the forward kinematics module
 * \return controller index
 */
int ForwardKinematics::get_ctrl_index(int fwd_kin_index)
{
	switch (fwd_kin_index)
	{
		// right leg
		case RIGHT_HIP_PITCH  : return CtrlIndex::RightHipPitch;
		case RIGHT_HIP_ROLL   : return CtrlIndex::RightHipRoll;
		case RIGHT_HIP_YAW    : return CtrlIndex::RightHipYaw;
		case RIGHT_KNEE_PITCH : return CtrlIndex::RightKneePitch;
		case RIGHT_FOOT_ROLL  : return CtrlIndex::RightFootRoll;
		case RIGHT_FOOT_PITCH : return CtrlIndex::RightFootPitch;

		// left leg
		case LEFT_HIP_PITCH  : return CtrlIndex::LeftHipPitch;
		case LEFT_HIP_ROLL   : return CtrlIndex::LeftHipRoll;
		case LEFT_HIP_YAW    : return CtrlIndex::LeftHipYaw;
		case LEFT_KNEE_PITCH : return CtrlIndex::LeftKneePitch;
		case LEFT_FOOT_ROLL  : return CtrlIndex::LeftFootRoll;
		case LEFT_FOOT_PITCH : return CtrlIndex::LeftFootPitch;

		// torso
		case TORSO_ROLL  : return CtrlIndex::TorsoRoll;
		case TORSO_PITCH : return CtrlIndex::TorsoPitch;
		case TORSO_YAW   : return CtrlIndex::TorsoYaw;

		// right arm
		case RIGHT_SH_PITCH  : return CtrlIndex::RightShPitch;
		case RIGHT_SH_ROLL   : return CtrlIndex::RightShRoll;
		case RIGHT_SH_YAW    : return CtrlIndex::RightShYaw;
		case RIGHT_ELB_PITCH : return CtrlIndex::RightElbPitch;

		// left arm
		case LEFT_SH_PITCH  : return CtrlIndex::LeftShPitch;
		case LEFT_SH_ROLL   : return CtrlIndex::LeftShRoll;
		case LEFT_SH_YAW    : return CtrlIndex::LeftShYaw;
		case LEFT_ELB_PITCH : return CtrlIndex::LeftElbPitch;

		default:
			std::cout << "Error: unknown forward kinematics index " << fwd_kin_index << " !" << std::endl;
			exit(EXIT_FAILURE);
			break;
	}
}

/*! \brief main kinematics computation
 *
 * \param[in,out] in_out inputs and outputs class
 *
 * computation of:
 *     COM (center of mass) position and velocity
 *     feet position, velocity and orientation
 *     waist and torso orientaion angles and derivatives
 *
 *   ////////////////////////
 *   //                    //
 *   //    17   16   21    //
 *   //  18     15     22  //
 *   //  19     14     23  //
 *   //  20     01     24  //
 *   //      02    08      //
 *   //     03      09     //
 *   //     04      10     //
 *   //     05      11     //
 *   //     06      12     //
 *   //     07      13     //
 *   //                    //
 *   ////////////////////////
 *
 * origin: in the waist, middle point between the two pitch hip rotations
 * inertial frame: located at the origin (waist), but aligned with the ground (info from IMU)
 *
 * Di   : position vector from the anchor point of the previous body to the current body i 
 *        (previous body is not always body i-1), expressed in the relative
 *        frame of the previous body
 * DGi  : position vector from the anchor point of body i to its COM (center of mass) G_i,
 *        expressed in the relative frame of the current body i
 * Omi  : rotational vector from the previous body to the current body i 
 *        (previous body is not always body i-1), expressed in the relative
 *        frame of the previous body
 * Rdi  : rotational matrix between body i and its predecessor
 * si   : sine of the relative angle before body i
 * ci   : cosine of the relative angle before body i
 *
 * xi   : absolute position vector (from origin, expressed in the inertial frame)
 *        of the anchor point of body i
 * xgi  : absolute position vector of the COM G_i of body i
 * xpi  : derivative of xi
 * xgpi : derivative of xgi
 * omi  : absolute rotational vector of body i
 * Ri   : absolute rotational matrix
 * Rti  : transpose matrix of Ri
 * xji  : jacobian of 'xi'
 * xgji : jacobian of 'xgi'
 * Rji  : jacobian of 'Ri'
 */
void ForwardKinematics::main_kinematics(KinematicsInOut &in_out)
{
	// -- variables declaration -- //

	double IMU11, IMU12, IMU13, IMU21, IMU22, IMU23, IMU31, IMU32, IMU33, omega_1, omega_2, omega_3, c2, c3, c4;
	double c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24, s2, s3;
	double s4, s5, s6, s7, s8, s9, s10, s11, s12, s13, s14, s15, s16, s17, s18, s19, s20, s21, s22, s23, s24, Om2;
	double Om3, Om4, Om5, Om6, Om7, Om8, Om9, Om10, Om11, Om12, Om13, Om14, Om15, Om16, Om17, Om18, Om19, Om20;
	double Om21, Om22, Om23, Om24, R1_11, R1_12, R1_13, R1_21, R1_22, R1_23, R1_31, R1_32, R1_33, R2_11, R2_12;
	double R2_13, R2_21, R2_22, R2_23, R2_31, R2_32, R2_33, R3_11, R3_12, R3_13, R3_21, R3_22, R3_23, R3_31, R3_32;
	double R3_33, R4_11, R4_12, R4_13, R4_21, R4_22, R4_23, R4_31, R4_32, R4_33, R5_11, R5_12, R5_13, R5_21, R5_22;
	double R5_23, R5_31, R5_32, R5_33, R6_11, R6_12, R6_13, R6_21, R6_22, R6_23, R6_31, R6_32, R6_33, R7_11, R7_12;
	double R7_13, R7_21, R7_22, R7_23, R7_31, R7_32, R7_33, R8_11, R8_12, R8_13, R8_21, R8_22, R8_23, R8_31, R8_32;
	double R8_33, R9_11, R9_12, R9_13, R9_21, R9_22, R9_23, R9_31, R9_32, R9_33, R10_11, R10_12, R10_13, R10_21;
	double R10_22, R10_23, R10_31, R10_32, R10_33, R11_11, R11_12, R11_13, R11_21, R11_22, R11_23, R11_31, R11_32;
	double R11_33, R12_11, R12_12, R12_13, R12_21, R12_22, R12_23, R12_31, R12_32, R12_33, R13_11, R13_12, R13_13;
	double R13_21, R13_22, R13_23, R13_31, R13_32, R13_33, R14_11, R14_12, R14_13, R14_21, R14_22, R14_23, R14_31;
	double R14_32, R14_33, R15_11, R15_12, R15_13, R15_21, R15_22, R15_23, R15_31, R15_32, R15_33, R16_11, R16_12;
	double R16_13, R16_21, R16_22, R16_23, R16_31, R16_32, R16_33, R17_11, R17_12, R17_13, R17_21, R17_22, R17_23;
	double R17_31, R17_32, R17_33, R18_11, R18_12, R18_13, R18_21, R18_22, R18_23, R18_31, R18_32, R18_33, R19_11;
	double R19_12, R19_13, R19_21, R19_22, R19_23, R19_31, R19_32, R19_33, R20_11, R20_12, R20_13, R20_21, R20_22;
	double R20_23, R20_31, R20_32, R20_33, R21_11, R21_12, R21_13, R21_21, R21_22, R21_23, R21_31, R21_32, R21_33;
	double R22_11, R22_12, R22_13, R22_21, R22_22, R22_23, R22_31, R22_32, R22_33, R23_11, R23_12, R23_13, R23_21;
	double R23_22, R23_23, R23_31, R23_32, R23_33, R24_11, R24_12, R24_13, R24_21, R24_22, R24_23, R24_31, R24_32;
	double R24_33, R2_11_d2, R2_12_d2, R2_13_d2, R2_31_d2, R2_32_d2, R2_33_d2, R3_11_d2, R3_12_d2, R3_13_d2, R3_21_d2;
	double R3_22_d2, R3_23_d2, R3_31_d2, R3_32_d2, R3_33_d2, R3_21_d3, R3_22_d3, R3_23_d3, R3_31_d3, R3_32_d3;
	double R3_33_d3, R4_11_d2, R4_12_d2, R4_13_d2, R4_21_d2, R4_22_d2, R4_23_d2, R4_31_d2, R4_32_d2, R4_33_d2;
	double R4_11_d3, R4_12_d3, R4_13_d3, R4_21_d3, R4_22_d3, R4_23_d3, R4_31_d3, R4_32_d3, R4_33_d3, R4_11_d4;
	double R4_12_d4, R4_13_d4, R4_21_d4, R4_22_d4, R4_23_d4, R5_11_d2, R5_12_d2, R5_13_d2, R5_21_d2, R5_22_d2;
	double R5_23_d2, R5_31_d2, R5_32_d2, R5_33_d2, R5_11_d3, R5_12_d3, R5_13_d3, R5_21_d3, R5_22_d3, R5_23_d3;
	double R5_31_d3, R5_32_d3, R5_33_d3, R5_11_d4, R5_12_d4, R5_13_d4, R5_21_d4, R5_22_d4, R5_23_d4, R5_31_d4;
	double R5_32_d4, R5_33_d4, R5_11_d5, R5_12_d5, R5_13_d5, R5_31_d5, R5_32_d5, R5_33_d5, R6_11_d2, R6_12_d2;
	double R6_13_d2, R6_21_d2, R6_22_d2, R6_23_d2, R6_31_d2, R6_32_d2, R6_33_d2, R6_11_d3, R6_12_d3, R6_13_d3;
	double R6_21_d3, R6_22_d3, R6_23_d3, R6_31_d3, R6_32_d3, R6_33_d3, R6_11_d4, R6_12_d4, R6_13_d4, R6_21_d4;
	double R6_22_d4, R6_23_d4, R6_31_d4, R6_32_d4, R6_33_d4, R6_11_d5, R6_12_d5, R6_13_d5, R6_21_d5, R6_22_d5;
	double R6_23_d5, R6_31_d5, R6_32_d5, R6_33_d5, R6_21_d6, R6_22_d6, R6_23_d6, R6_31_d6, R6_32_d6, R6_33_d6;
	double R7_11_d2, R7_12_d2, R7_13_d2, R7_21_d2, R7_22_d2, R7_23_d2, R7_31_d2, R7_32_d2, R7_33_d2, R7_11_d3;
	double R7_12_d3, R7_13_d3, R7_21_d3, R7_22_d3, R7_23_d3, R7_31_d3, R7_32_d3, R7_33_d3, R7_11_d4, R7_12_d4;
	double R7_13_d4, R7_21_d4, R7_22_d4, R7_23_d4, R7_31_d4, R7_32_d4, R7_33_d4, R7_11_d5, R7_12_d5, R7_13_d5;
	double R7_21_d5, R7_22_d5, R7_23_d5, R7_31_d5, R7_32_d5, R7_33_d5, R7_11_d6, R7_12_d6, R7_13_d6, R7_21_d6;
	double R7_22_d6, R7_23_d6, R7_31_d6, R7_32_d6, R7_33_d6, R7_11_d7, R7_12_d7, R7_13_d7, R7_31_d7, R7_32_d7;
	double R7_33_d7, R8_11_d8, R8_12_d8, R8_13_d8, R8_31_d8, R8_32_d8, R8_33_d8, R9_11_d8, R9_12_d8, R9_13_d8;
	double R9_21_d8, R9_22_d8, R9_23_d8, R9_31_d8, R9_32_d8, R9_33_d8, R9_21_d9, R9_22_d9, R9_23_d9, R9_31_d9;
	double R9_32_d9, R9_33_d9, R10_11_d8, R10_12_d8, R10_13_d8, R10_21_d8, R10_22_d8, R10_23_d8, R10_31_d8, R10_32_d8;
	double R10_33_d8, R10_11_d9, R10_12_d9, R10_13_d9, R10_21_d9, R10_22_d9, R10_23_d9, R10_31_d9, R10_32_d9, R10_33_d9;
	double R10_11_d10, R10_12_d10, R10_13_d10, R10_21_d10, R10_22_d10, R10_23_d10, R11_11_d8, R11_12_d8, R11_13_d8;
	double R11_21_d8, R11_22_d8, R11_23_d8, R11_31_d8, R11_32_d8, R11_33_d8, R11_11_d9, R11_12_d9, R11_13_d9, R11_21_d9;
	double R11_22_d9, R11_23_d9, R11_31_d9, R11_32_d9, R11_33_d9, R11_11_d10, R11_12_d10, R11_13_d10, R11_21_d10;
	double R11_22_d10, R11_23_d10, R11_31_d10, R11_32_d10, R11_33_d10, R11_11_d11, R11_12_d11, R11_13_d11, R11_31_d11;
	double R11_32_d11, R11_33_d11, R12_11_d8, R12_12_d8, R12_13_d8, R12_21_d8, R12_22_d8, R12_23_d8, R12_31_d8;
	double R12_32_d8, R12_33_d8, R12_11_d9, R12_12_d9, R12_13_d9, R12_21_d9, R12_22_d9, R12_23_d9, R12_31_d9, R12_32_d9;
	double R12_33_d9, R12_11_d10, R12_12_d10, R12_13_d10, R12_21_d10, R12_22_d10, R12_23_d10, R12_31_d10, R12_32_d10;
	double R12_33_d10, R12_11_d11, R12_12_d11, R12_13_d11, R12_21_d11, R12_22_d11, R12_23_d11, R12_31_d11, R12_32_d11;
	double R12_33_d11, R12_21_d12, R12_22_d12, R12_23_d12, R12_31_d12, R12_32_d12, R12_33_d12, R13_11_d8, R13_12_d8;
	double R13_13_d8, R13_21_d8, R13_22_d8, R13_23_d8, R13_31_d8, R13_32_d8, R13_33_d8, R13_11_d9, R13_12_d9, R13_13_d9;
	double R13_21_d9, R13_22_d9, R13_23_d9, R13_31_d9, R13_32_d9, R13_33_d9, R13_11_d10, R13_12_d10, R13_13_d10;
	double R13_21_d10, R13_22_d10, R13_23_d10, R13_31_d10, R13_32_d10, R13_33_d10, R13_11_d11, R13_12_d11, R13_13_d11;
	double R13_21_d11, R13_22_d11, R13_23_d11, R13_31_d11, R13_32_d11, R13_33_d11, R13_11_d12, R13_12_d12, R13_13_d12;
	double R13_21_d12, R13_22_d12, R13_23_d12, R13_31_d12, R13_32_d12, R13_33_d12, R13_11_d13, R13_12_d13, R13_13_d13;
	double R13_31_d13, R13_32_d13, R13_33_d13, R14_21_d14, R14_22_d14, R14_23_d14, R14_31_d14, R14_32_d14, R14_33_d14;
	double R15_11_d14, R15_12_d14, R15_13_d14, R15_21_d14, R15_22_d14, R15_23_d14, R15_31_d14, R15_32_d14, R15_33_d14;
	double R15_11_d15, R15_12_d15, R15_13_d15, R15_31_d15, R15_32_d15, R15_33_d15, R16_11_d14, R16_12_d14, R16_13_d14;
	double R16_21_d14, R16_22_d14, R16_23_d14, R16_31_d14, R16_32_d14, R16_33_d14, R16_11_d15, R16_12_d15, R16_13_d15;
	double R16_21_d15, R16_22_d15, R16_23_d15, R16_31_d15, R16_32_d15, R16_33_d15, R16_11_d16, R16_12_d16, R16_13_d16;
	double R16_21_d16, R16_22_d16, R16_23_d16, R17_11_d14, R17_12_d14, R17_13_d14, R17_21_d14, R17_22_d14, R17_23_d14;
	double R17_31_d14, R17_32_d14, R17_33_d14, R17_11_d15, R17_12_d15, R17_13_d15, R17_21_d15, R17_22_d15, R17_23_d15;
	double R17_31_d15, R17_32_d15, R17_33_d15, R17_11_d16, R17_12_d16, R17_13_d16, R17_21_d16, R17_22_d16, R17_23_d16;
	double R17_31_d16, R17_32_d16, R17_33_d16, R17_11_d17, R17_12_d17, R17_13_d17, R17_31_d17, R17_32_d17, R17_33_d17;
	double R18_11_d14, R18_12_d14, R18_13_d14, R18_21_d14, R18_22_d14, R18_23_d14, R18_31_d14, R18_32_d14, R18_33_d14;
	double R18_11_d15, R18_12_d15, R18_13_d15, R18_21_d15, R18_22_d15, R18_23_d15, R18_31_d15, R18_32_d15, R18_33_d15;
	double R18_11_d16, R18_12_d16, R18_13_d16, R18_21_d16, R18_22_d16, R18_23_d16, R18_31_d16, R18_32_d16, R18_33_d16;
	double R18_11_d17, R18_12_d17, R18_13_d17, R18_21_d17, R18_22_d17, R18_23_d17, R18_31_d17, R18_32_d17, R18_33_d17;
	double R18_21_d18, R18_22_d18, R18_23_d18, R18_31_d18, R18_32_d18, R18_33_d18, R19_11_d14, R19_12_d14, R19_13_d14;
	double R19_21_d14, R19_22_d14, R19_23_d14, R19_31_d14, R19_32_d14, R19_33_d14, R19_11_d15, R19_12_d15, R19_13_d15;
	double R19_21_d15, R19_22_d15, R19_23_d15, R19_31_d15, R19_32_d15, R19_33_d15, R19_11_d16, R19_12_d16, R19_13_d16;
	double R19_21_d16, R19_22_d16, R19_23_d16, R19_31_d16, R19_32_d16, R19_33_d16, R19_11_d17, R19_12_d17, R19_13_d17;
	double R19_21_d17, R19_22_d17, R19_23_d17, R19_31_d17, R19_32_d17, R19_33_d17, R19_11_d18, R19_12_d18, R19_13_d18;
	double R19_21_d18, R19_22_d18, R19_23_d18, R19_31_d18, R19_32_d18, R19_33_d18, R19_11_d19, R19_12_d19, R19_13_d19;
	double R19_21_d19, R19_22_d19, R19_23_d19, R20_11_d14, R20_12_d14, R20_13_d14, R20_21_d14, R20_22_d14, R20_23_d14;
	double R20_31_d14, R20_32_d14, R20_33_d14, R20_11_d15, R20_12_d15, R20_13_d15, R20_21_d15, R20_22_d15, R20_23_d15;
	double R20_31_d15, R20_32_d15, R20_33_d15, R20_11_d16, R20_12_d16, R20_13_d16, R20_21_d16, R20_22_d16, R20_23_d16;
	double R20_31_d16, R20_32_d16, R20_33_d16, R20_11_d17, R20_12_d17, R20_13_d17, R20_21_d17, R20_22_d17, R20_23_d17;
	double R20_31_d17, R20_32_d17, R20_33_d17, R20_11_d18, R20_12_d18, R20_13_d18, R20_21_d18, R20_22_d18, R20_23_d18;
	double R20_31_d18, R20_32_d18, R20_33_d18, R20_11_d19, R20_12_d19, R20_13_d19, R20_21_d19, R20_22_d19, R20_23_d19;
	double R20_31_d19, R20_32_d19, R20_33_d19, R20_11_d20, R20_12_d20, R20_13_d20, R20_31_d20, R20_32_d20, R20_33_d20;
	double R21_11_d14, R21_12_d14, R21_13_d14, R21_21_d14, R21_22_d14, R21_23_d14, R21_31_d14, R21_32_d14, R21_33_d14;
	double R21_11_d15, R21_12_d15, R21_13_d15, R21_21_d15, R21_22_d15, R21_23_d15, R21_31_d15, R21_32_d15, R21_33_d15;
	double R21_11_d16, R21_12_d16, R21_13_d16, R21_21_d16, R21_22_d16, R21_23_d16, R21_31_d16, R21_32_d16, R21_33_d16;
	double R21_11_d21, R21_12_d21, R21_13_d21, R21_31_d21, R21_32_d21, R21_33_d21, R22_11_d14, R22_12_d14, R22_13_d14;
	double R22_21_d14, R22_22_d14, R22_23_d14, R22_31_d14, R22_32_d14, R22_33_d14, R22_11_d15, R22_12_d15, R22_13_d15;
	double R22_21_d15, R22_22_d15, R22_23_d15, R22_31_d15, R22_32_d15, R22_33_d15, R22_11_d16, R22_12_d16, R22_13_d16;
	double R22_21_d16, R22_22_d16, R22_23_d16, R22_31_d16, R22_32_d16, R22_33_d16, R22_11_d21, R22_12_d21, R22_13_d21;
	double R22_21_d21, R22_22_d21, R22_23_d21, R22_31_d21, R22_32_d21, R22_33_d21, R22_21_d22, R22_22_d22, R22_23_d22;
	double R22_31_d22, R22_32_d22, R22_33_d22, R23_11_d14, R23_12_d14, R23_13_d14, R23_21_d14, R23_22_d14, R23_23_d14;
	double R23_31_d14, R23_32_d14, R23_33_d14, R23_11_d15, R23_12_d15, R23_13_d15, R23_21_d15, R23_22_d15, R23_23_d15;
	double R23_31_d15, R23_32_d15, R23_33_d15, R23_11_d16, R23_12_d16, R23_13_d16, R23_21_d16, R23_22_d16, R23_23_d16;
	double R23_31_d16, R23_32_d16, R23_33_d16, R23_11_d21, R23_12_d21, R23_13_d21, R23_21_d21, R23_22_d21, R23_23_d21;
	double R23_31_d21, R23_32_d21, R23_33_d21, R23_11_d22, R23_12_d22, R23_13_d22, R23_21_d22, R23_22_d22, R23_23_d22;
	double R23_31_d22, R23_32_d22, R23_33_d22, R23_11_d23, R23_12_d23, R23_13_d23, R23_21_d23, R23_22_d23, R23_23_d23;
	double R24_11_d14, R24_12_d14, R24_13_d14, R24_21_d14, R24_22_d14, R24_23_d14, R24_31_d14, R24_32_d14, R24_33_d14;
	double R24_11_d15, R24_12_d15, R24_13_d15, R24_21_d15, R24_22_d15, R24_23_d15, R24_31_d15, R24_32_d15, R24_33_d15;
	double R24_11_d16, R24_12_d16, R24_13_d16, R24_21_d16, R24_22_d16, R24_23_d16, R24_31_d16, R24_32_d16, R24_33_d16;
	double R24_11_d21, R24_12_d21, R24_13_d21, R24_21_d21, R24_22_d21, R24_23_d21, R24_31_d21, R24_32_d21, R24_33_d21;
	double R24_11_d22, R24_12_d22, R24_13_d22, R24_21_d22, R24_22_d22, R24_23_d22, R24_31_d22, R24_32_d22, R24_33_d22;
	double R24_11_d23, R24_12_d23, R24_13_d23, R24_21_d23, R24_22_d23, R24_23_d23, R24_31_d23, R24_32_d23, R24_33_d23;
	double R24_11_d24, R24_12_d24, R24_13_d24, R24_31_d24, R24_32_d24, R24_33_d24, om1_1, om1_2, om1_3, om2_1;
	double om2_2, om2_3, om3_1, om3_2, om3_3, om4_1, om4_2, om4_3, om5_1, om5_2, om5_3, om6_1, om6_2, om6_3, om7_1;
	double om7_2, om7_3, om8_1, om8_2, om8_3, om9_1, om9_2, om9_3, om10_1, om10_2, om10_3, om11_1, om11_2, om11_3;
	double om12_1, om12_2, om12_3, om13_1, om13_2, om13_3, om14_1, om14_2, om14_3, om15_1, om15_2, om15_3, om16_1;
	double om16_2, om16_3, om17_1, om17_2, om17_3, om18_1, om18_2, om18_3, om19_1, om19_2, om19_3, om20_1, om20_2;
	double om20_3, om21_1, om21_2, om21_3, om22_1, om22_2, om22_3, om23_1, om23_2, om23_3, om24_1, om24_2, om24_3;
	double x2_1, x2_2, x2_3, xp2_1, xp2_2, xp2_3, x3_1, x3_2, x3_3, xp3_1, xp3_2, xp3_3, x4_1, x4_2, x4_3, xp4_1;
	double xp4_2, xp4_3, x5_1, x5_2, x5_3, xp5_1, xp5_2, xp5_3, x6_1, x6_2, x6_3, xp6_1, xp6_2, xp6_3, x7_1, x7_2;
	double x7_3, xp7_1, xp7_2, xp7_3, x8_1, x8_2, x8_3, xp8_1, xp8_2, xp8_3, x9_1, x9_2, x9_3, xp9_1, xp9_2, xp9_3;
	double x10_1, x10_2, x10_3, xp10_1, xp10_2, xp10_3, x11_1, x11_2, x11_3, xp11_1, xp11_2, xp11_3, x12_1, x12_2;
	double x12_3, xp12_1, xp12_2, xp12_3, x13_1, x13_2, x13_3, xp13_1, xp13_2, xp13_3, x14_1, x14_2, x14_3, xp14_1;
	double xp14_2, xp14_3, x15_1, x15_2, x15_3, xp15_1, xp15_2, xp15_3, x16_1, x16_2, x16_3, xp16_1, xp16_2, xp16_3;
	double x17_1, x17_2, x17_3, xp17_1, xp17_2, xp17_3, x18_1, x18_2, x18_3, xp18_1, xp18_2, xp18_3, x19_1, x19_2;
	double x19_3, xp19_1, xp19_2, xp19_3, x20_1, x20_2, x20_3, xp20_1, xp20_2, xp20_3, x21_1, x21_2, x21_3, xp21_1;
	double xp21_2, xp21_3, x22_1, x22_2, x22_3, xp22_1, xp22_2, xp22_3, x23_1, x23_2, x23_3, xp23_1, xp23_2, xp23_3;
	double x24_1, x24_2, x24_3, xp24_1, xp24_2, xp24_3, x4_1_d2, x4_2_d2, x4_3_d2, x4_1_d3, x4_2_d3, x4_3_d3, x5_1_d2;
	double x5_2_d2, x5_3_d2, x5_1_d3, x5_2_d3, x5_3_d3, x6_1_d2, x6_2_d2, x6_3_d2, x6_1_d3, x6_2_d3, x6_3_d3, x6_1_d4;
	double x6_2_d4, x6_3_d4, x6_1_d5, x6_2_d5, x6_3_d5, x7_1_d2, x7_2_d2, x7_3_d2, x7_1_d3, x7_2_d3, x7_3_d3, x7_1_d4;
	double x7_2_d4, x7_3_d4, x7_1_d5, x7_2_d5, x7_3_d5, x10_1_d8, x10_2_d8, x10_3_d8, x10_1_d9, x10_2_d9, x10_3_d9;
	double x11_1_d8, x11_2_d8, x11_3_d8, x11_1_d9, x11_2_d9, x11_3_d9, x12_1_d8, x12_2_d8, x12_3_d8, x12_1_d9;
	double x12_2_d9, x12_3_d9, x12_1_d10, x12_2_d10, x12_3_d10, x12_1_d11, x12_2_d11, x12_3_d11, x13_1_d8, x13_2_d8;
	double x13_3_d8, x13_1_d9, x13_2_d9, x13_3_d9, x13_1_d10, x13_2_d10, x13_3_d10, x13_1_d11, x13_2_d11, x13_3_d11;
	double x16_1_d14, x16_2_d14, x16_3_d14, x16_1_d15, x16_2_d15, x16_3_d15, x17_1_d14, x17_2_d14, x17_3_d14, x17_1_d15;
	double x17_2_d15, x17_3_d15, x17_1_d16, x17_2_d16, x17_3_d16, x18_1_d14, x18_2_d14, x18_3_d14, x18_1_d15, x18_2_d15;
	double x18_3_d15, x18_1_d16, x18_2_d16, x18_3_d16, x19_1_d14, x19_2_d14, x19_3_d14, x19_1_d15, x19_2_d15, x19_3_d15;
	double x19_1_d16, x19_2_d16, x19_3_d16, x19_1_d17, x19_2_d17, x19_3_d17, x19_1_d18, x19_2_d18, x19_3_d18, x20_1_d14;
	double x20_2_d14, x20_3_d14, x20_1_d15, x20_2_d15, x20_3_d15, x20_1_d16, x20_2_d16, x20_3_d16, x20_1_d17, x20_2_d17;
	double x20_3_d17, x20_1_d18, x20_2_d18, x20_3_d18, x21_1_d14, x21_2_d14, x21_3_d14, x21_1_d15, x21_2_d15, x21_3_d15;
	double x21_1_d16, x21_2_d16, x21_3_d16, x22_1_d14, x22_2_d14, x22_3_d14, x22_1_d15, x22_2_d15, x22_3_d15, x22_1_d16;
	double x22_2_d16, x22_3_d16, x23_1_d14, x23_2_d14, x23_3_d14, x23_1_d15, x23_2_d15, x23_3_d15, x23_1_d16, x23_2_d16;
	double x23_3_d16, x23_1_d21, x23_2_d21, x23_3_d21, x23_1_d22, x23_2_d22, x23_3_d22, x24_1_d14, x24_2_d14, x24_3_d14;
	double x24_1_d15, x24_2_d15, x24_3_d15, x24_1_d16, x24_2_d16, x24_3_d16, x24_1_d21, x24_2_d21, x24_3_d21, x24_1_d22;
	double x24_2_d22, x24_3_d22, xg1_1, xg1_2, xg1_3, xgp1_1, xgp1_2, xgp1_3, xg2_1, xg2_2, xg2_3, xgp2_1, xgp2_2;
	double xgp2_3, xg3_1, xg3_2, xg3_3, xgp3_1, xgp3_2, xgp3_3, xg4_1, xg4_2, xg4_3, xgp4_1, xgp4_2, xgp4_3, xg5_1;
	double xg5_2, xg5_3, xgp5_1, xgp5_2, xgp5_3, xg6_1, xg6_2, xg6_3, xgp6_1, xgp6_2, xgp6_3, xg7_1, xg7_2, xg7_3;
	double xgp7_1, xgp7_2, xgp7_3, xg8_1, xg8_2, xg8_3, xgp8_1, xgp8_2, xgp8_3, xg9_1, xg9_2, xg9_3, xgp9_1, xgp9_2;
	double xgp9_3, xg10_1, xg10_2, xg10_3, xgp10_1, xgp10_2, xgp10_3, xg11_1, xg11_2, xg11_3, xgp11_1, xgp11_2;
	double xgp11_3, xg12_1, xg12_2, xg12_3, xgp12_1, xgp12_2, xgp12_3, xg13_1, xg13_2, xg13_3, xgp13_1, xgp13_2;
	double xgp13_3, xg14_1, xg14_2, xg14_3, xgp14_1, xgp14_2, xgp14_3, xg15_1, xg15_2, xg15_3, xgp15_1, xgp15_2;
	double xgp15_3, xg16_1, xg16_2, xg16_3, xgp16_1, xgp16_2, xgp16_3, xg17_1, xg17_2, xg17_3, xgp17_1, xgp17_2;
	double xgp17_3, xg18_1, xg18_2, xg18_3, xgp18_1, xgp18_2, xgp18_3, xg19_1, xg19_2, xg19_3, xgp19_1, xgp19_2;
	double xgp19_3, xg20_1, xg20_2, xg20_3, xgp20_1, xgp20_2, xgp20_3, xg21_1, xg21_2, xg21_3, xgp21_1, xgp21_2;
	double xgp21_3, xg22_1, xg22_2, xg22_3, xgp22_1, xgp22_2, xgp22_3, xg23_1, xg23_2, xg23_3, xgp23_1, xgp23_2;
	double xgp23_3, xg24_1, xg24_2, xg24_3, xgp24_1, xgp24_2, xgp24_3, xg2_1_d2, xg2_2_d2, xg2_3_d2, xg3_1_d2;
	double xg3_2_d2, xg3_3_d2, xg3_1_d3, xg3_2_d3, xg3_3_d3, xg4_1_d2, xg4_2_d2, xg4_3_d2, xg4_1_d3, xg4_2_d3;
	double xg4_3_d3, xg4_1_d4, xg4_2_d4, xg4_3_d4, xg5_1_d2, xg5_2_d2, xg5_3_d2, xg5_1_d3, xg5_2_d3, xg5_3_d3;
	double xg5_1_d4, xg5_2_d4, xg5_3_d4, xg5_1_d5, xg5_2_d5, xg5_3_d5, xg6_1_d2, xg6_2_d2, xg6_3_d2, xg6_1_d3;
	double xg6_2_d3, xg6_3_d3, xg6_1_d4, xg6_2_d4, xg6_3_d4, xg6_1_d5, xg6_2_d5, xg6_3_d5, xg6_1_d6, xg6_2_d6;
	double xg6_3_d6, xg7_1_d2, xg7_2_d2, xg7_3_d2, xg7_1_d3, xg7_2_d3, xg7_3_d3, xg7_1_d4, xg7_2_d4, xg7_3_d4;
	double xg7_1_d5, xg7_2_d5, xg7_3_d5, xg7_1_d6, xg7_2_d6, xg7_3_d6, xg7_1_d7, xg7_2_d7, xg7_3_d7, xg8_1_d8;
	double xg8_2_d8, xg8_3_d8, xg9_1_d8, xg9_2_d8, xg9_3_d8, xg9_1_d9, xg9_2_d9, xg9_3_d9, xg10_1_d8, xg10_2_d8;
	double xg10_3_d8, xg10_1_d9, xg10_2_d9, xg10_3_d9, xg10_1_d10, xg10_2_d10, xg10_3_d10, xg11_1_d8, xg11_2_d8;
	double xg11_3_d8, xg11_1_d9, xg11_2_d9, xg11_3_d9, xg11_1_d10, xg11_2_d10, xg11_3_d10, xg11_1_d11, xg11_2_d11;
	double xg11_3_d11, xg12_1_d8, xg12_2_d8, xg12_3_d8, xg12_1_d9, xg12_2_d9, xg12_3_d9, xg12_1_d10, xg12_2_d10;
	double xg12_3_d10, xg12_1_d11, xg12_2_d11, xg12_3_d11, xg12_1_d12, xg12_2_d12, xg12_3_d12, xg13_1_d8, xg13_2_d8;
	double xg13_3_d8, xg13_1_d9, xg13_2_d9, xg13_3_d9, xg13_1_d10, xg13_2_d10, xg13_3_d10, xg13_1_d11, xg13_2_d11;
	double xg13_3_d11, xg13_1_d12, xg13_2_d12, xg13_3_d12, xg13_1_d13, xg13_2_d13, xg13_3_d13, xg14_1_d14, xg14_2_d14;
	double xg14_3_d14, xg15_1_d14, xg15_2_d14, xg15_3_d14, xg15_1_d15, xg15_2_d15, xg15_3_d15, xg16_1_d14, xg16_2_d14;
	double xg16_3_d14, xg16_1_d15, xg16_2_d15, xg16_3_d15, xg16_1_d16, xg16_2_d16, xg16_3_d16, xg17_1_d14, xg17_2_d14;
	double xg17_3_d14, xg17_1_d15, xg17_2_d15, xg17_3_d15, xg17_1_d16, xg17_2_d16, xg17_3_d16, xg17_1_d17, xg17_2_d17;
	double xg17_3_d17, xg18_1_d14, xg18_2_d14, xg18_3_d14, xg18_1_d15, xg18_2_d15, xg18_3_d15, xg18_1_d16, xg18_2_d16;
	double xg18_3_d16, xg18_1_d17, xg18_2_d17, xg18_3_d17, xg18_1_d18, xg18_2_d18, xg18_3_d18, xg19_1_d14, xg19_2_d14;
	double xg19_3_d14, xg19_1_d15, xg19_2_d15, xg19_3_d15, xg19_1_d16, xg19_2_d16, xg19_3_d16, xg19_1_d17, xg19_2_d17;
	double xg19_3_d17, xg19_1_d18, xg19_2_d18, xg19_3_d18, xg19_1_d19, xg19_2_d19, xg19_3_d19, xg20_1_d14, xg20_2_d14;
	double xg20_3_d14, xg20_1_d15, xg20_2_d15, xg20_3_d15, xg20_1_d16, xg20_2_d16, xg20_3_d16, xg20_1_d17, xg20_2_d17;
	double xg20_3_d17, xg20_1_d18, xg20_2_d18, xg20_3_d18, xg20_1_d19, xg20_2_d19, xg20_3_d19, xg20_1_d20, xg20_2_d20;
	double xg20_3_d20, xg21_1_d14, xg21_2_d14, xg21_3_d14, xg21_1_d15, xg21_2_d15, xg21_3_d15, xg21_1_d16, xg21_2_d16;
	double xg21_3_d16, xg21_1_d21, xg21_2_d21, xg21_3_d21, xg22_1_d14, xg22_2_d14, xg22_3_d14, xg22_1_d15, xg22_2_d15;
	double xg22_3_d15, xg22_1_d16, xg22_2_d16, xg22_3_d16, xg22_1_d21, xg22_2_d21, xg22_3_d21, xg22_1_d22, xg22_2_d22;
	double xg22_3_d22, xg23_1_d14, xg23_2_d14, xg23_3_d14, xg23_1_d15, xg23_2_d15, xg23_3_d15, xg23_1_d16, xg23_2_d16;
	double xg23_3_d16, xg23_1_d21, xg23_2_d21, xg23_3_d21, xg23_1_d22, xg23_2_d22, xg23_3_d22, xg23_1_d23, xg23_2_d23;
	double xg23_3_d23, xg24_1_d14, xg24_2_d14, xg24_3_d14, xg24_1_d15, xg24_2_d15, xg24_3_d15, xg24_1_d16, xg24_2_d16;
	double xg24_3_d16, xg24_1_d21, xg24_2_d21, xg24_3_d21, xg24_1_d22, xg24_2_d22, xg24_3_d22, xg24_1_d23, xg24_2_d23;
	double xg24_3_d23, xg24_1_d24, xg24_2_d24, xg24_3_d24, m_tot, c_y_Rfoot, c_y_Lfoot, c_z_Rfoot, c_z_Lfoot, s_y_Rfoot;
	double s_y_Lfoot, s_z_Rfoot, s_z_Lfoot, inv_c_y_Rfoot, inv_c_y_Lfoot, c_y_waist, c_y_torso, c_z_waist, c_z_torso;
	double s_y_waist, s_y_torso, s_z_waist, s_z_torso, inv_c_y_waist, inv_c_y_torso;


	// -- variables initialization -- //

	// IMU - rotation matrices
	IMU11 = in_out.IMU_Orientation[0];
	IMU12 = in_out.IMU_Orientation[1];
	IMU13 = in_out.IMU_Orientation[2];
	IMU21 = in_out.IMU_Orientation[3];
	IMU22 = in_out.IMU_Orientation[4];
	IMU23 = in_out.IMU_Orientation[5];
	IMU31 = in_out.IMU_Orientation[6];
	IMU32 = in_out.IMU_Orientation[7];
	IMU33 = in_out.IMU_Orientation[8];

	// IMU - angles velocity
	omega_1 = in_out.IMU_Angular_Rate[0];
	omega_2 = in_out.IMU_Angular_Rate[1];
	omega_3 = in_out.IMU_Angular_Rate[2];

	// joint cosines
	c2 = cos(in_out.q_mot[RightHipPitch_id]);
	c3 = cos(in_out.q_mot[RightHipRoll_id]);
	c4 = cos(in_out.q_mot[RightHipYaw_id]);
	c5 = cos(in_out.q_mot[RightKneePitch_id]);
	c6 = cos(in_out.q_mot[RightFootRoll_id]);
	c7 = cos(in_out.q_mot[RightFootPitch_id]);
	c8 = cos(in_out.q_mot[LeftHipPitch_id]);
	c9 = cos(in_out.q_mot[LeftHipRoll_id]);
	c10 = cos(in_out.q_mot[LeftHipYaw_id]);
	c11 = cos(in_out.q_mot[LeftKneePitch_id]);
	c12 = cos(in_out.q_mot[LeftFootRoll_id]);
	c13 = cos(in_out.q_mot[LeftFootPitch_id]);
	c14 = cos(in_out.q_mot[TorsoRoll_id]);
	c15 = cos(in_out.q_mot[TorsoPitch_id]);
	c16 = cos(in_out.q_mot[TorsoYaw_id]);
	c17 = cos(in_out.q_mot[RightShPitch_id]);
	c18 = cos(in_out.q_mot[RightShRoll_id]);
	c19 = cos(in_out.q_mot[RightShYaw_id]);
	c20 = cos(in_out.q_mot[RightElbPitch_id]);
	c21 = cos(in_out.q_mot[LeftShPitch_id]);
	c22 = cos(in_out.q_mot[LeftShRoll_id]);
	c23 = cos(in_out.q_mot[LeftShYaw_id]);
	c24 = cos(in_out.q_mot[LeftElbPitch_id]);

	// joint sines
	s2 = sin(in_out.q_mot[RightHipPitch_id]);
	s3 = sin(in_out.q_mot[RightHipRoll_id]);
	s4 = sin(in_out.q_mot[RightHipYaw_id]);
	s5 = sin(in_out.q_mot[RightKneePitch_id]);
	s6 = sin(in_out.q_mot[RightFootRoll_id]);
	s7 = sin(in_out.q_mot[RightFootPitch_id]);
	s8 = sin(in_out.q_mot[LeftHipPitch_id]);
	s9 = sin(in_out.q_mot[LeftHipRoll_id]);
	s10 = sin(in_out.q_mot[LeftHipYaw_id]);
	s11 = sin(in_out.q_mot[LeftKneePitch_id]);
	s12 = sin(in_out.q_mot[LeftFootRoll_id]);
	s13 = sin(in_out.q_mot[LeftFootPitch_id]);
	s14 = sin(in_out.q_mot[TorsoRoll_id]);
	s15 = sin(in_out.q_mot[TorsoPitch_id]);
	s16 = sin(in_out.q_mot[TorsoYaw_id]);
	s17 = sin(in_out.q_mot[RightShPitch_id]);
	s18 = sin(in_out.q_mot[RightShRoll_id]);
	s19 = sin(in_out.q_mot[RightShYaw_id]);
	s20 = sin(in_out.q_mot[RightElbPitch_id]);
	s21 = sin(in_out.q_mot[LeftShPitch_id]);
	s22 = sin(in_out.q_mot[LeftShRoll_id]);
	s23 = sin(in_out.q_mot[LeftShYaw_id]);
	s24 = sin(in_out.q_mot[LeftElbPitch_id]);

	// joint relative velocities
	Om2 = in_out.qd_mot[RightHipPitch_id];
	Om3 = in_out.qd_mot[RightHipRoll_id];
	Om4 = in_out.qd_mot[RightHipYaw_id];
	Om5 = in_out.qd_mot[RightKneePitch_id];
	Om6 = in_out.qd_mot[RightFootRoll_id];
	Om7 = in_out.qd_mot[RightFootPitch_id];
	Om8 = in_out.qd_mot[LeftHipPitch_id];
	Om9 = in_out.qd_mot[LeftHipRoll_id];
	Om10 = in_out.qd_mot[LeftHipYaw_id];
	Om11 = in_out.qd_mot[LeftKneePitch_id];
	Om12 = in_out.qd_mot[LeftFootRoll_id];
	Om13 = in_out.qd_mot[LeftFootPitch_id];
	Om14 = in_out.qd_mot[TorsoRoll_id];
	Om15 = in_out.qd_mot[TorsoPitch_id];
	Om16 = in_out.qd_mot[TorsoYaw_id];
	Om17 = in_out.qd_mot[RightShPitch_id];
	Om18 = in_out.qd_mot[RightShRoll_id];
	Om19 = in_out.qd_mot[RightShYaw_id];
	Om20 = in_out.qd_mot[RightElbPitch_id];
	Om21 = in_out.qd_mot[LeftShPitch_id];
	Om22 = in_out.qd_mot[LeftShRoll_id];
	Om23 = in_out.qd_mot[LeftShYaw_id];
	Om24 = in_out.qd_mot[LeftElbPitch_id];


	// -- symbolic computation -- //

	// rotation matrices
	R1_11 = IMU11;
	R1_12 = IMU12;
	R1_13 = IMU13;
	R1_21 = IMU21;
	R1_22 = IMU22;
	R1_23 = IMU23;
	R1_31 = IMU31;
	R1_32 = IMU32;
	R1_33 = IMU33;

	R2_11 = R1_11*c2 - R1_31*s2;
	R2_12 = R1_12*c2 - R1_32*s2;
	R2_13 = R1_13*c2 - R1_33*s2;
	R2_21 = R1_21;
	R2_22 = R1_22;
	R2_23 = R1_23;
	R2_31 = R1_11*s2 + R1_31*c2;
	R2_32 = R1_12*s2 + R1_32*c2;
	R2_33 = R1_13*s2 + R1_33*c2;

	R3_11 = R2_11;
	R3_12 = R2_12;
	R3_13 = R2_13;
	R3_21 = R2_21*c3 + R2_31*s3;
	R3_22 = R2_22*c3 + R2_32*s3;
	R3_23 = R2_23*c3 + R2_33*s3;
	R3_31 = -R2_21*s3 + R2_31*c3;
	R3_32 = -R2_22*s3 + R2_32*c3;
	R3_33 = -R2_23*s3 + R2_33*c3;

	R4_11 = R3_11*c4 + R3_21*s4;
	R4_12 = R3_12*c4 + R3_22*s4;
	R4_13 = R3_13*c4 + R3_23*s4;
	R4_21 = -R3_11*s4 + R3_21*c4;
	R4_22 = -R3_12*s4 + R3_22*c4;
	R4_23 = -R3_13*s4 + R3_23*c4;
	R4_31 = R3_31;
	R4_32 = R3_32;
	R4_33 = R3_33;

	R5_11 = R4_11*c5 - R4_31*s5;
	R5_12 = R4_12*c5 - R4_32*s5;
	R5_13 = R4_13*c5 - R4_33*s5;
	R5_21 = R4_21;
	R5_22 = R4_22;
	R5_23 = R4_23;
	R5_31 = R4_11*s5 + R4_31*c5;
	R5_32 = R4_12*s5 + R4_32*c5;
	R5_33 = R4_13*s5 + R4_33*c5;

	R6_11 = R5_11;
	R6_12 = R5_12;
	R6_13 = R5_13;
	R6_21 = R5_21*c6 + R5_31*s6;
	R6_22 = R5_22*c6 + R5_32*s6;
	R6_23 = R5_23*c6 + R5_33*s6;
	R6_31 = -R5_21*s6 + R5_31*c6;
	R6_32 = -R5_22*s6 + R5_32*c6;
	R6_33 = -R5_23*s6 + R5_33*c6;

	R7_11 = R6_11*c7 - R6_31*s7;
	R7_12 = R6_12*c7 - R6_32*s7;
	R7_13 = R6_13*c7 - R6_33*s7;
	R7_21 = R6_21;
	R7_22 = R6_22;
	R7_23 = R6_23;
	R7_31 = R6_11*s7 + R6_31*c7;
	R7_32 = R6_12*s7 + R6_32*c7;
	R7_33 = R6_13*s7 + R6_33*c7;

	R8_11 = R1_11*c8 - R1_31*s8;
	R8_12 = R1_12*c8 - R1_32*s8;
	R8_13 = R1_13*c8 - R1_33*s8;
	R8_21 = R1_21;
	R8_22 = R1_22;
	R8_23 = R1_23;
	R8_31 = R1_11*s8 + R1_31*c8;
	R8_32 = R1_12*s8 + R1_32*c8;
	R8_33 = R1_13*s8 + R1_33*c8;

	R9_11 = R8_11;
	R9_12 = R8_12;
	R9_13 = R8_13;
	R9_21 = R8_21*c9 + R8_31*s9;
	R9_22 = R8_22*c9 + R8_32*s9;
	R9_23 = R8_23*c9 + R8_33*s9;
	R9_31 = -R8_21*s9 + R8_31*c9;
	R9_32 = -R8_22*s9 + R8_32*c9;
	R9_33 = -R8_23*s9 + R8_33*c9;

	R10_11 = R9_11*c10 + R9_21*s10;
	R10_12 = R9_12*c10 + R9_22*s10;
	R10_13 = R9_13*c10 + R9_23*s10;
	R10_21 = -R9_11*s10 + R9_21*c10;
	R10_22 = -R9_12*s10 + R9_22*c10;
	R10_23 = -R9_13*s10 + R9_23*c10;
	R10_31 = R9_31;
	R10_32 = R9_32;
	R10_33 = R9_33;

	R11_11 = R10_11*c11 - R10_31*s11;
	R11_12 = R10_12*c11 - R10_32*s11;
	R11_13 = R10_13*c11 - R10_33*s11;
	R11_21 = R10_21;
	R11_22 = R10_22;
	R11_23 = R10_23;
	R11_31 = R10_11*s11 + R10_31*c11;
	R11_32 = R10_12*s11 + R10_32*c11;
	R11_33 = R10_13*s11 + R10_33*c11;

	R12_11 = R11_11;
	R12_12 = R11_12;
	R12_13 = R11_13;
	R12_21 = R11_21*c12 + R11_31*s12;
	R12_22 = R11_22*c12 + R11_32*s12;
	R12_23 = R11_23*c12 + R11_33*s12;
	R12_31 = -R11_21*s12 + R11_31*c12;
	R12_32 = -R11_22*s12 + R11_32*c12;
	R12_33 = -R11_23*s12 + R11_33*c12;

	R13_11 = R12_11*c13 - R12_31*s13;
	R13_12 = R12_12*c13 - R12_32*s13;
	R13_13 = R12_13*c13 - R12_33*s13;
	R13_21 = R12_21;
	R13_22 = R12_22;
	R13_23 = R12_23;
	R13_31 = R12_11*s13 + R12_31*c13;
	R13_32 = R12_12*s13 + R12_32*c13;
	R13_33 = R12_13*s13 + R12_33*c13;

	R14_11 = R1_11;
	R14_12 = R1_12;
	R14_13 = R1_13;
	R14_21 = R1_21*c14 + R1_31*s14;
	R14_22 = R1_22*c14 + R1_32*s14;
	R14_23 = R1_23*c14 + R1_33*s14;
	R14_31 = -R1_21*s14 + R1_31*c14;
	R14_32 = -R1_22*s14 + R1_32*c14;
	R14_33 = -R1_23*s14 + R1_33*c14;

	R15_11 = R14_11*c15 - R14_31*s15;
	R15_12 = R14_12*c15 - R14_32*s15;
	R15_13 = R14_13*c15 - R14_33*s15;
	R15_21 = R14_21;
	R15_22 = R14_22;
	R15_23 = R14_23;
	R15_31 = R14_11*s15 + R14_31*c15;
	R15_32 = R14_12*s15 + R14_32*c15;
	R15_33 = R14_13*s15 + R14_33*c15;

	R16_11 = R15_11*c16 + R15_21*s16;
	R16_12 = R15_12*c16 + R15_22*s16;
	R16_13 = R15_13*c16 + R15_23*s16;
	R16_21 = -R15_11*s16 + R15_21*c16;
	R16_22 = -R15_12*s16 + R15_22*c16;
	R16_23 = -R15_13*s16 + R15_23*c16;
	R16_31 = R15_31;
	R16_32 = R15_32;
	R16_33 = R15_33;

	R17_11 = R16_11*c17 - R16_31*s17;
	R17_12 = R16_12*c17 - R16_32*s17;
	R17_13 = R16_13*c17 - R16_33*s17;
	R17_21 = R16_21;
	R17_22 = R16_22;
	R17_23 = R16_23;
	R17_31 = R16_11*s17 + R16_31*c17;
	R17_32 = R16_12*s17 + R16_32*c17;
	R17_33 = R16_13*s17 + R16_33*c17;

	R18_11 = R17_11;
	R18_12 = R17_12;
	R18_13 = R17_13;
	R18_21 = R17_21*c18 + R17_31*s18;
	R18_22 = R17_22*c18 + R17_32*s18;
	R18_23 = R17_23*c18 + R17_33*s18;
	R18_31 = -R17_21*s18 + R17_31*c18;
	R18_32 = -R17_22*s18 + R17_32*c18;
	R18_33 = -R17_23*s18 + R17_33*c18;

	R19_11 = R18_11*c19 + R18_21*s19;
	R19_12 = R18_12*c19 + R18_22*s19;
	R19_13 = R18_13*c19 + R18_23*s19;
	R19_21 = -R18_11*s19 + R18_21*c19;
	R19_22 = -R18_12*s19 + R18_22*c19;
	R19_23 = -R18_13*s19 + R18_23*c19;
	R19_31 = R18_31;
	R19_32 = R18_32;
	R19_33 = R18_33;

	R20_11 = R19_11*c20 - R19_31*s20;
	R20_12 = R19_12*c20 - R19_32*s20;
	R20_13 = R19_13*c20 - R19_33*s20;
	R20_21 = R19_21;
	R20_22 = R19_22;
	R20_23 = R19_23;
	R20_31 = R19_11*s20 + R19_31*c20;
	R20_32 = R19_12*s20 + R19_32*c20;
	R20_33 = R19_13*s20 + R19_33*c20;

	R21_11 = R16_11*c21 - R16_31*s21;
	R21_12 = R16_12*c21 - R16_32*s21;
	R21_13 = R16_13*c21 - R16_33*s21;
	R21_21 = R16_21;
	R21_22 = R16_22;
	R21_23 = R16_23;
	R21_31 = R16_11*s21 + R16_31*c21;
	R21_32 = R16_12*s21 + R16_32*c21;
	R21_33 = R16_13*s21 + R16_33*c21;

	R22_11 = R21_11;
	R22_12 = R21_12;
	R22_13 = R21_13;
	R22_21 = R21_21*c22 + R21_31*s22;
	R22_22 = R21_22*c22 + R21_32*s22;
	R22_23 = R21_23*c22 + R21_33*s22;
	R22_31 = -R21_21*s22 + R21_31*c22;
	R22_32 = -R21_22*s22 + R21_32*c22;
	R22_33 = -R21_23*s22 + R21_33*c22;

	R23_11 = R22_11*c23 + R22_21*s23;
	R23_12 = R22_12*c23 + R22_22*s23;
	R23_13 = R22_13*c23 + R22_23*s23;
	R23_21 = -R22_11*s23 + R22_21*c23;
	R23_22 = -R22_12*s23 + R22_22*c23;
	R23_23 = -R22_13*s23 + R22_23*c23;
	R23_31 = R22_31;
	R23_32 = R22_32;
	R23_33 = R22_33;

	R24_11 = R23_11*c24 - R23_31*s24;
	R24_12 = R23_12*c24 - R23_32*s24;
	R24_13 = R23_13*c24 - R23_33*s24;
	R24_21 = R23_21;
	R24_22 = R23_22;
	R24_23 = R23_23;
	R24_31 = R23_11*s24 + R23_31*c24;
	R24_32 = R23_12*s24 + R23_32*c24;
	R24_33 = R23_13*s24 + R23_33*c24;


	// jacobian rotation matrices
	if (flag_jacob)
	{
		R2_11_d2 = -R1_31*c2 - R1_11*s2;
		R2_12_d2 = -R1_32*c2 - R1_12*s2;
		R2_13_d2 = -R1_33*c2 - R1_13*s2;
		R2_31_d2 = R1_11*c2 - R1_31*s2;
		R2_32_d2 = R1_12*c2 - R1_32*s2;
		R2_33_d2 = R1_13*c2 - R1_33*s2;

		R3_11_d2 = R2_11_d2;
		R3_12_d2 = R2_12_d2;
		R3_13_d2 = R2_13_d2;
		R3_21_d2 = R2_31_d2*s3;
		R3_22_d2 = R2_32_d2*s3;
		R3_23_d2 = R2_33_d2*s3;
		R3_31_d2 = R2_31_d2*c3;
		R3_32_d2 = R2_32_d2*c3;
		R3_33_d2 = R2_33_d2*c3;

		R3_21_d3 = -R2_21*s3 + R2_31*c3;
		R3_22_d3 = -R2_22*s3 + R2_32*c3;
		R3_23_d3 = -R2_23*s3 + R2_33*c3;
		R3_31_d3 = -R2_21*c3 - R2_31*s3;
		R3_32_d3 = -R2_22*c3 - R2_32*s3;
		R3_33_d3 = -R2_23*c3 - R2_33*s3;

		R4_11_d2 = R3_11_d2*c4 + R3_21_d2*s4;
		R4_12_d2 = R3_12_d2*c4 + R3_22_d2*s4;
		R4_13_d2 = R3_13_d2*c4 + R3_23_d2*s4;
		R4_21_d2 = -R3_11_d2*s4 + R3_21_d2*c4;
		R4_22_d2 = -R3_12_d2*s4 + R3_22_d2*c4;
		R4_23_d2 = -R3_13_d2*s4 + R3_23_d2*c4;
		R4_31_d2 = R3_31_d2;
		R4_32_d2 = R3_32_d2;
		R4_33_d2 = R3_33_d2;

		R4_11_d3 = R3_21_d3*s4;
		R4_12_d3 = R3_22_d3*s4;
		R4_13_d3 = R3_23_d3*s4;
		R4_21_d3 = R3_21_d3*c4;
		R4_22_d3 = R3_22_d3*c4;
		R4_23_d3 = R3_23_d3*c4;
		R4_31_d3 = R3_31_d3;
		R4_32_d3 = R3_32_d3;
		R4_33_d3 = R3_33_d3;

		R4_11_d4 = -R3_11*s4 + R3_21*c4;
		R4_12_d4 = -R3_12*s4 + R3_22*c4;
		R4_13_d4 = -R3_13*s4 + R3_23*c4;
		R4_21_d4 = -R3_11*c4 - R3_21*s4;
		R4_22_d4 = -R3_12*c4 - R3_22*s4;
		R4_23_d4 = -R3_13*c4 - R3_23*s4;

		R5_11_d2 = R4_11_d2*c5 - R4_31_d2*s5;
		R5_12_d2 = R4_12_d2*c5 - R4_32_d2*s5;
		R5_13_d2 = R4_13_d2*c5 - R4_33_d2*s5;
		R5_21_d2 = R4_21_d2;
		R5_22_d2 = R4_22_d2;
		R5_23_d2 = R4_23_d2;
		R5_31_d2 = R4_11_d2*s5 + R4_31_d2*c5;
		R5_32_d2 = R4_12_d2*s5 + R4_32_d2*c5;
		R5_33_d2 = R4_13_d2*s5 + R4_33_d2*c5;

		R5_11_d3 = R4_11_d3*c5 - R4_31_d3*s5;
		R5_12_d3 = R4_12_d3*c5 - R4_32_d3*s5;
		R5_13_d3 = R4_13_d3*c5 - R4_33_d3*s5;
		R5_21_d3 = R4_21_d3;
		R5_22_d3 = R4_22_d3;
		R5_23_d3 = R4_23_d3;
		R5_31_d3 = R4_11_d3*s5 + R4_31_d3*c5;
		R5_32_d3 = R4_12_d3*s5 + R4_32_d3*c5;
		R5_33_d3 = R4_13_d3*s5 + R4_33_d3*c5;

		R5_11_d4 = R4_11_d4*c5;
		R5_12_d4 = R4_12_d4*c5;
		R5_13_d4 = R4_13_d4*c5;
		R5_21_d4 = R4_21_d4;
		R5_22_d4 = R4_22_d4;
		R5_23_d4 = R4_23_d4;
		R5_31_d4 = R4_11_d4*s5;
		R5_32_d4 = R4_12_d4*s5;
		R5_33_d4 = R4_13_d4*s5;

		R5_11_d5 = -R4_31*c5 - R4_11*s5;
		R5_12_d5 = -R4_32*c5 - R4_12*s5;
		R5_13_d5 = -R4_33*c5 - R4_13*s5;
		R5_31_d5 = R4_11*c5 - R4_31*s5;
		R5_32_d5 = R4_12*c5 - R4_32*s5;
		R5_33_d5 = R4_13*c5 - R4_33*s5;

		R6_11_d2 = R5_11_d2;
		R6_12_d2 = R5_12_d2;
		R6_13_d2 = R5_13_d2;
		R6_21_d2 = R5_21_d2*c6 + R5_31_d2*s6;
		R6_22_d2 = R5_22_d2*c6 + R5_32_d2*s6;
		R6_23_d2 = R5_23_d2*c6 + R5_33_d2*s6;
		R6_31_d2 = -R5_21_d2*s6 + R5_31_d2*c6;
		R6_32_d2 = -R5_22_d2*s6 + R5_32_d2*c6;
		R6_33_d2 = -R5_23_d2*s6 + R5_33_d2*c6;

		R6_11_d3 = R5_11_d3;
		R6_12_d3 = R5_12_d3;
		R6_13_d3 = R5_13_d3;
		R6_21_d3 = R5_21_d3*c6 + R5_31_d3*s6;
		R6_22_d3 = R5_22_d3*c6 + R5_32_d3*s6;
		R6_23_d3 = R5_23_d3*c6 + R5_33_d3*s6;
		R6_31_d3 = -R5_21_d3*s6 + R5_31_d3*c6;
		R6_32_d3 = -R5_22_d3*s6 + R5_32_d3*c6;
		R6_33_d3 = -R5_23_d3*s6 + R5_33_d3*c6;

		R6_11_d4 = R5_11_d4;
		R6_12_d4 = R5_12_d4;
		R6_13_d4 = R5_13_d4;
		R6_21_d4 = R5_21_d4*c6 + R5_31_d4*s6;
		R6_22_d4 = R5_22_d4*c6 + R5_32_d4*s6;
		R6_23_d4 = R5_23_d4*c6 + R5_33_d4*s6;
		R6_31_d4 = -R5_21_d4*s6 + R5_31_d4*c6;
		R6_32_d4 = -R5_22_d4*s6 + R5_32_d4*c6;
		R6_33_d4 = -R5_23_d4*s6 + R5_33_d4*c6;

		R6_11_d5 = R5_11_d5;
		R6_12_d5 = R5_12_d5;
		R6_13_d5 = R5_13_d5;
		R6_21_d5 = R5_31_d5*s6;
		R6_22_d5 = R5_32_d5*s6;
		R6_23_d5 = R5_33_d5*s6;
		R6_31_d5 = R5_31_d5*c6;
		R6_32_d5 = R5_32_d5*c6;
		R6_33_d5 = R5_33_d5*c6;

		R6_21_d6 = -R5_21*s6 + R5_31*c6;
		R6_22_d6 = -R5_22*s6 + R5_32*c6;
		R6_23_d6 = -R5_23*s6 + R5_33*c6;
		R6_31_d6 = -R5_21*c6 - R5_31*s6;
		R6_32_d6 = -R5_22*c6 - R5_32*s6;
		R6_33_d6 = -R5_23*c6 - R5_33*s6;

		R7_11_d2 = R6_11_d2*c7 - R6_31_d2*s7;
		R7_12_d2 = R6_12_d2*c7 - R6_32_d2*s7;
		R7_13_d2 = R6_13_d2*c7 - R6_33_d2*s7;
		R7_21_d2 = R6_21_d2;
		R7_22_d2 = R6_22_d2;
		R7_23_d2 = R6_23_d2;
		R7_31_d2 = R6_11_d2*s7 + R6_31_d2*c7;
		R7_32_d2 = R6_12_d2*s7 + R6_32_d2*c7;
		R7_33_d2 = R6_13_d2*s7 + R6_33_d2*c7;

		R7_11_d3 = R6_11_d3*c7 - R6_31_d3*s7;
		R7_12_d3 = R6_12_d3*c7 - R6_32_d3*s7;
		R7_13_d3 = R6_13_d3*c7 - R6_33_d3*s7;
		R7_21_d3 = R6_21_d3;
		R7_22_d3 = R6_22_d3;
		R7_23_d3 = R6_23_d3;
		R7_31_d3 = R6_11_d3*s7 + R6_31_d3*c7;
		R7_32_d3 = R6_12_d3*s7 + R6_32_d3*c7;
		R7_33_d3 = R6_13_d3*s7 + R6_33_d3*c7;

		R7_11_d4 = R6_11_d4*c7 - R6_31_d4*s7;
		R7_12_d4 = R6_12_d4*c7 - R6_32_d4*s7;
		R7_13_d4 = R6_13_d4*c7 - R6_33_d4*s7;
		R7_21_d4 = R6_21_d4;
		R7_22_d4 = R6_22_d4;
		R7_23_d4 = R6_23_d4;
		R7_31_d4 = R6_11_d4*s7 + R6_31_d4*c7;
		R7_32_d4 = R6_12_d4*s7 + R6_32_d4*c7;
		R7_33_d4 = R6_13_d4*s7 + R6_33_d4*c7;

		R7_11_d5 = R6_11_d5*c7 - R6_31_d5*s7;
		R7_12_d5 = R6_12_d5*c7 - R6_32_d5*s7;
		R7_13_d5 = R6_13_d5*c7 - R6_33_d5*s7;
		R7_21_d5 = R6_21_d5;
		R7_22_d5 = R6_22_d5;
		R7_23_d5 = R6_23_d5;
		R7_31_d5 = R6_11_d5*s7 + R6_31_d5*c7;
		R7_32_d5 = R6_12_d5*s7 + R6_32_d5*c7;
		R7_33_d5 = R6_13_d5*s7 + R6_33_d5*c7;

		R7_11_d6 = -R6_31_d6*s7;
		R7_12_d6 = -R6_32_d6*s7;
		R7_13_d6 = -R6_33_d6*s7;
		R7_21_d6 = R6_21_d6;
		R7_22_d6 = R6_22_d6;
		R7_23_d6 = R6_23_d6;
		R7_31_d6 = R6_31_d6*c7;
		R7_32_d6 = R6_32_d6*c7;
		R7_33_d6 = R6_33_d6*c7;

		R7_11_d7 = -R6_31*c7 - R6_11*s7;
		R7_12_d7 = -R6_32*c7 - R6_12*s7;
		R7_13_d7 = -R6_33*c7 - R6_13*s7;
		R7_31_d7 = R6_11*c7 - R6_31*s7;
		R7_32_d7 = R6_12*c7 - R6_32*s7;
		R7_33_d7 = R6_13*c7 - R6_33*s7;

		R8_11_d8 = -R1_31*c8 - R1_11*s8;
		R8_12_d8 = -R1_32*c8 - R1_12*s8;
		R8_13_d8 = -R1_33*c8 - R1_13*s8;
		R8_31_d8 = R1_11*c8 - R1_31*s8;
		R8_32_d8 = R1_12*c8 - R1_32*s8;
		R8_33_d8 = R1_13*c8 - R1_33*s8;

		R9_11_d8 = R8_11_d8;
		R9_12_d8 = R8_12_d8;
		R9_13_d8 = R8_13_d8;
		R9_21_d8 = R8_31_d8*s9;
		R9_22_d8 = R8_32_d8*s9;
		R9_23_d8 = R8_33_d8*s9;
		R9_31_d8 = R8_31_d8*c9;
		R9_32_d8 = R8_32_d8*c9;
		R9_33_d8 = R8_33_d8*c9;

		R9_21_d9 = -R8_21*s9 + R8_31*c9;
		R9_22_d9 = -R8_22*s9 + R8_32*c9;
		R9_23_d9 = -R8_23*s9 + R8_33*c9;
		R9_31_d9 = -R8_21*c9 - R8_31*s9;
		R9_32_d9 = -R8_22*c9 - R8_32*s9;
		R9_33_d9 = -R8_23*c9 - R8_33*s9;

		R10_11_d8 = R9_11_d8*c10 + R9_21_d8*s10;
		R10_12_d8 = R9_12_d8*c10 + R9_22_d8*s10;
		R10_13_d8 = R9_13_d8*c10 + R9_23_d8*s10;
		R10_21_d8 = -R9_11_d8*s10 + R9_21_d8*c10;
		R10_22_d8 = -R9_12_d8*s10 + R9_22_d8*c10;
		R10_23_d8 = -R9_13_d8*s10 + R9_23_d8*c10;
		R10_31_d8 = R9_31_d8;
		R10_32_d8 = R9_32_d8;
		R10_33_d8 = R9_33_d8;

		R10_11_d9 = R9_21_d9*s10;
		R10_12_d9 = R9_22_d9*s10;
		R10_13_d9 = R9_23_d9*s10;
		R10_21_d9 = R9_21_d9*c10;
		R10_22_d9 = R9_22_d9*c10;
		R10_23_d9 = R9_23_d9*c10;
		R10_31_d9 = R9_31_d9;
		R10_32_d9 = R9_32_d9;
		R10_33_d9 = R9_33_d9;

		R10_11_d10 = -R9_11*s10 + R9_21*c10;
		R10_12_d10 = -R9_12*s10 + R9_22*c10;
		R10_13_d10 = -R9_13*s10 + R9_23*c10;
		R10_21_d10 = -R9_11*c10 - R9_21*s10;
		R10_22_d10 = -R9_12*c10 - R9_22*s10;
		R10_23_d10 = -R9_13*c10 - R9_23*s10;

		R11_11_d8 = R10_11_d8*c11 - R10_31_d8*s11;
		R11_12_d8 = R10_12_d8*c11 - R10_32_d8*s11;
		R11_13_d8 = R10_13_d8*c11 - R10_33_d8*s11;
		R11_21_d8 = R10_21_d8;
		R11_22_d8 = R10_22_d8;
		R11_23_d8 = R10_23_d8;
		R11_31_d8 = R10_11_d8*s11 + R10_31_d8*c11;
		R11_32_d8 = R10_12_d8*s11 + R10_32_d8*c11;
		R11_33_d8 = R10_13_d8*s11 + R10_33_d8*c11;

		R11_11_d9 = R10_11_d9*c11 - R10_31_d9*s11;
		R11_12_d9 = R10_12_d9*c11 - R10_32_d9*s11;
		R11_13_d9 = R10_13_d9*c11 - R10_33_d9*s11;
		R11_21_d9 = R10_21_d9;
		R11_22_d9 = R10_22_d9;
		R11_23_d9 = R10_23_d9;
		R11_31_d9 = R10_11_d9*s11 + R10_31_d9*c11;
		R11_32_d9 = R10_12_d9*s11 + R10_32_d9*c11;
		R11_33_d9 = R10_13_d9*s11 + R10_33_d9*c11;

		R11_11_d10 = R10_11_d10*c11;
		R11_12_d10 = R10_12_d10*c11;
		R11_13_d10 = R10_13_d10*c11;
		R11_21_d10 = R10_21_d10;
		R11_22_d10 = R10_22_d10;
		R11_23_d10 = R10_23_d10;
		R11_31_d10 = R10_11_d10*s11;
		R11_32_d10 = R10_12_d10*s11;
		R11_33_d10 = R10_13_d10*s11;

		R11_11_d11 = -R10_31*c11 - R10_11*s11;
		R11_12_d11 = -R10_32*c11 - R10_12*s11;
		R11_13_d11 = -R10_33*c11 - R10_13*s11;
		R11_31_d11 = R10_11*c11 - R10_31*s11;
		R11_32_d11 = R10_12*c11 - R10_32*s11;
		R11_33_d11 = R10_13*c11 - R10_33*s11;

		R12_11_d8 = R11_11_d8;
		R12_12_d8 = R11_12_d8;
		R12_13_d8 = R11_13_d8;
		R12_21_d8 = R11_21_d8*c12 + R11_31_d8*s12;
		R12_22_d8 = R11_22_d8*c12 + R11_32_d8*s12;
		R12_23_d8 = R11_23_d8*c12 + R11_33_d8*s12;
		R12_31_d8 = -R11_21_d8*s12 + R11_31_d8*c12;
		R12_32_d8 = -R11_22_d8*s12 + R11_32_d8*c12;
		R12_33_d8 = -R11_23_d8*s12 + R11_33_d8*c12;

		R12_11_d9 = R11_11_d9;
		R12_12_d9 = R11_12_d9;
		R12_13_d9 = R11_13_d9;
		R12_21_d9 = R11_21_d9*c12 + R11_31_d9*s12;
		R12_22_d9 = R11_22_d9*c12 + R11_32_d9*s12;
		R12_23_d9 = R11_23_d9*c12 + R11_33_d9*s12;
		R12_31_d9 = -R11_21_d9*s12 + R11_31_d9*c12;
		R12_32_d9 = -R11_22_d9*s12 + R11_32_d9*c12;
		R12_33_d9 = -R11_23_d9*s12 + R11_33_d9*c12;

		R12_11_d10 = R11_11_d10;
		R12_12_d10 = R11_12_d10;
		R12_13_d10 = R11_13_d10;
		R12_21_d10 = R11_21_d10*c12 + R11_31_d10*s12;
		R12_22_d10 = R11_22_d10*c12 + R11_32_d10*s12;
		R12_23_d10 = R11_23_d10*c12 + R11_33_d10*s12;
		R12_31_d10 = -R11_21_d10*s12 + R11_31_d10*c12;
		R12_32_d10 = -R11_22_d10*s12 + R11_32_d10*c12;
		R12_33_d10 = -R11_23_d10*s12 + R11_33_d10*c12;

		R12_11_d11 = R11_11_d11;
		R12_12_d11 = R11_12_d11;
		R12_13_d11 = R11_13_d11;
		R12_21_d11 = R11_31_d11*s12;
		R12_22_d11 = R11_32_d11*s12;
		R12_23_d11 = R11_33_d11*s12;
		R12_31_d11 = R11_31_d11*c12;
		R12_32_d11 = R11_32_d11*c12;
		R12_33_d11 = R11_33_d11*c12;

		R12_21_d12 = -R11_21*s12 + R11_31*c12;
		R12_22_d12 = -R11_22*s12 + R11_32*c12;
		R12_23_d12 = -R11_23*s12 + R11_33*c12;
		R12_31_d12 = -R11_21*c12 - R11_31*s12;
		R12_32_d12 = -R11_22*c12 - R11_32*s12;
		R12_33_d12 = -R11_23*c12 - R11_33*s12;

		R13_11_d8 = R12_11_d8*c13 - R12_31_d8*s13;
		R13_12_d8 = R12_12_d8*c13 - R12_32_d8*s13;
		R13_13_d8 = R12_13_d8*c13 - R12_33_d8*s13;
		R13_21_d8 = R12_21_d8;
		R13_22_d8 = R12_22_d8;
		R13_23_d8 = R12_23_d8;
		R13_31_d8 = R12_11_d8*s13 + R12_31_d8*c13;
		R13_32_d8 = R12_12_d8*s13 + R12_32_d8*c13;
		R13_33_d8 = R12_13_d8*s13 + R12_33_d8*c13;

		R13_11_d9 = R12_11_d9*c13 - R12_31_d9*s13;
		R13_12_d9 = R12_12_d9*c13 - R12_32_d9*s13;
		R13_13_d9 = R12_13_d9*c13 - R12_33_d9*s13;
		R13_21_d9 = R12_21_d9;
		R13_22_d9 = R12_22_d9;
		R13_23_d9 = R12_23_d9;
		R13_31_d9 = R12_11_d9*s13 + R12_31_d9*c13;
		R13_32_d9 = R12_12_d9*s13 + R12_32_d9*c13;
		R13_33_d9 = R12_13_d9*s13 + R12_33_d9*c13;

		R13_11_d10 = R12_11_d10*c13 - R12_31_d10*s13;
		R13_12_d10 = R12_12_d10*c13 - R12_32_d10*s13;
		R13_13_d10 = R12_13_d10*c13 - R12_33_d10*s13;
		R13_21_d10 = R12_21_d10;
		R13_22_d10 = R12_22_d10;
		R13_23_d10 = R12_23_d10;
		R13_31_d10 = R12_11_d10*s13 + R12_31_d10*c13;
		R13_32_d10 = R12_12_d10*s13 + R12_32_d10*c13;
		R13_33_d10 = R12_13_d10*s13 + R12_33_d10*c13;

		R13_11_d11 = R12_11_d11*c13 - R12_31_d11*s13;
		R13_12_d11 = R12_12_d11*c13 - R12_32_d11*s13;
		R13_13_d11 = R12_13_d11*c13 - R12_33_d11*s13;
		R13_21_d11 = R12_21_d11;
		R13_22_d11 = R12_22_d11;
		R13_23_d11 = R12_23_d11;
		R13_31_d11 = R12_11_d11*s13 + R12_31_d11*c13;
		R13_32_d11 = R12_12_d11*s13 + R12_32_d11*c13;
		R13_33_d11 = R12_13_d11*s13 + R12_33_d11*c13;

		R13_11_d12 = -R12_31_d12*s13;
		R13_12_d12 = -R12_32_d12*s13;
		R13_13_d12 = -R12_33_d12*s13;
		R13_21_d12 = R12_21_d12;
		R13_22_d12 = R12_22_d12;
		R13_23_d12 = R12_23_d12;
		R13_31_d12 = R12_31_d12*c13;
		R13_32_d12 = R12_32_d12*c13;
		R13_33_d12 = R12_33_d12*c13;

		R13_11_d13 = -R12_31*c13 - R12_11*s13;
		R13_12_d13 = -R12_32*c13 - R12_12*s13;
		R13_13_d13 = -R12_33*c13 - R12_13*s13;
		R13_31_d13 = R12_11*c13 - R12_31*s13;
		R13_32_d13 = R12_12*c13 - R12_32*s13;
		R13_33_d13 = R12_13*c13 - R12_33*s13;

		R14_21_d14 = -R1_21*s14 + R1_31*c14;
		R14_22_d14 = -R1_22*s14 + R1_32*c14;
		R14_23_d14 = -R1_23*s14 + R1_33*c14;
		R14_31_d14 = -R1_21*c14 - R1_31*s14;
		R14_32_d14 = -R1_22*c14 - R1_32*s14;
		R14_33_d14 = -R1_23*c14 - R1_33*s14;

		R15_11_d14 = -R14_31_d14*s15;
		R15_12_d14 = -R14_32_d14*s15;
		R15_13_d14 = -R14_33_d14*s15;
		R15_21_d14 = R14_21_d14;
		R15_22_d14 = R14_22_d14;
		R15_23_d14 = R14_23_d14;
		R15_31_d14 = R14_31_d14*c15;
		R15_32_d14 = R14_32_d14*c15;
		R15_33_d14 = R14_33_d14*c15;

		R15_11_d15 = -R14_31*c15 - R14_11*s15;
		R15_12_d15 = -R14_32*c15 - R14_12*s15;
		R15_13_d15 = -R14_33*c15 - R14_13*s15;
		R15_31_d15 = R14_11*c15 - R14_31*s15;
		R15_32_d15 = R14_12*c15 - R14_32*s15;
		R15_33_d15 = R14_13*c15 - R14_33*s15;

		R16_11_d14 = R15_11_d14*c16 + R15_21_d14*s16;
		R16_12_d14 = R15_12_d14*c16 + R15_22_d14*s16;
		R16_13_d14 = R15_13_d14*c16 + R15_23_d14*s16;
		R16_21_d14 = -R15_11_d14*s16 + R15_21_d14*c16;
		R16_22_d14 = -R15_12_d14*s16 + R15_22_d14*c16;
		R16_23_d14 = -R15_13_d14*s16 + R15_23_d14*c16;
		R16_31_d14 = R15_31_d14;
		R16_32_d14 = R15_32_d14;
		R16_33_d14 = R15_33_d14;

		R16_11_d15 = R15_11_d15*c16;
		R16_12_d15 = R15_12_d15*c16;
		R16_13_d15 = R15_13_d15*c16;
		R16_21_d15 = -R15_11_d15*s16;
		R16_22_d15 = -R15_12_d15*s16;
		R16_23_d15 = -R15_13_d15*s16;
		R16_31_d15 = R15_31_d15;
		R16_32_d15 = R15_32_d15;
		R16_33_d15 = R15_33_d15;

		R16_11_d16 = -R15_11*s16 + R15_21*c16;
		R16_12_d16 = -R15_12*s16 + R15_22*c16;
		R16_13_d16 = -R15_13*s16 + R15_23*c16;
		R16_21_d16 = -R15_11*c16 - R15_21*s16;
		R16_22_d16 = -R15_12*c16 - R15_22*s16;
		R16_23_d16 = -R15_13*c16 - R15_23*s16;

		R17_11_d14 = R16_11_d14*c17 - R16_31_d14*s17;
		R17_12_d14 = R16_12_d14*c17 - R16_32_d14*s17;
		R17_13_d14 = R16_13_d14*c17 - R16_33_d14*s17;
		R17_21_d14 = R16_21_d14;
		R17_22_d14 = R16_22_d14;
		R17_23_d14 = R16_23_d14;
		R17_31_d14 = R16_11_d14*s17 + R16_31_d14*c17;
		R17_32_d14 = R16_12_d14*s17 + R16_32_d14*c17;
		R17_33_d14 = R16_13_d14*s17 + R16_33_d14*c17;

		R17_11_d15 = R16_11_d15*c17 - R16_31_d15*s17;
		R17_12_d15 = R16_12_d15*c17 - R16_32_d15*s17;
		R17_13_d15 = R16_13_d15*c17 - R16_33_d15*s17;
		R17_21_d15 = R16_21_d15;
		R17_22_d15 = R16_22_d15;
		R17_23_d15 = R16_23_d15;
		R17_31_d15 = R16_11_d15*s17 + R16_31_d15*c17;
		R17_32_d15 = R16_12_d15*s17 + R16_32_d15*c17;
		R17_33_d15 = R16_13_d15*s17 + R16_33_d15*c17;

		R17_11_d16 = R16_11_d16*c17;
		R17_12_d16 = R16_12_d16*c17;
		R17_13_d16 = R16_13_d16*c17;
		R17_21_d16 = R16_21_d16;
		R17_22_d16 = R16_22_d16;
		R17_23_d16 = R16_23_d16;
		R17_31_d16 = R16_11_d16*s17;
		R17_32_d16 = R16_12_d16*s17;
		R17_33_d16 = R16_13_d16*s17;

		R17_11_d17 = -R16_31*c17 - R16_11*s17;
		R17_12_d17 = -R16_32*c17 - R16_12*s17;
		R17_13_d17 = -R16_33*c17 - R16_13*s17;
		R17_31_d17 = R16_11*c17 - R16_31*s17;
		R17_32_d17 = R16_12*c17 - R16_32*s17;
		R17_33_d17 = R16_13*c17 - R16_33*s17;

		R18_11_d14 = R17_11_d14;
		R18_12_d14 = R17_12_d14;
		R18_13_d14 = R17_13_d14;
		R18_21_d14 = R17_21_d14*c18 + R17_31_d14*s18;
		R18_22_d14 = R17_22_d14*c18 + R17_32_d14*s18;
		R18_23_d14 = R17_23_d14*c18 + R17_33_d14*s18;
		R18_31_d14 = -R17_21_d14*s18 + R17_31_d14*c18;
		R18_32_d14 = -R17_22_d14*s18 + R17_32_d14*c18;
		R18_33_d14 = -R17_23_d14*s18 + R17_33_d14*c18;

		R18_11_d15 = R17_11_d15;
		R18_12_d15 = R17_12_d15;
		R18_13_d15 = R17_13_d15;
		R18_21_d15 = R17_21_d15*c18 + R17_31_d15*s18;
		R18_22_d15 = R17_22_d15*c18 + R17_32_d15*s18;
		R18_23_d15 = R17_23_d15*c18 + R17_33_d15*s18;
		R18_31_d15 = -R17_21_d15*s18 + R17_31_d15*c18;
		R18_32_d15 = -R17_22_d15*s18 + R17_32_d15*c18;
		R18_33_d15 = -R17_23_d15*s18 + R17_33_d15*c18;

		R18_11_d16 = R17_11_d16;
		R18_12_d16 = R17_12_d16;
		R18_13_d16 = R17_13_d16;
		R18_21_d16 = R17_21_d16*c18 + R17_31_d16*s18;
		R18_22_d16 = R17_22_d16*c18 + R17_32_d16*s18;
		R18_23_d16 = R17_23_d16*c18 + R17_33_d16*s18;
		R18_31_d16 = -R17_21_d16*s18 + R17_31_d16*c18;
		R18_32_d16 = -R17_22_d16*s18 + R17_32_d16*c18;
		R18_33_d16 = -R17_23_d16*s18 + R17_33_d16*c18;

		R18_11_d17 = R17_11_d17;
		R18_12_d17 = R17_12_d17;
		R18_13_d17 = R17_13_d17;
		R18_21_d17 = R17_31_d17*s18;
		R18_22_d17 = R17_32_d17*s18;
		R18_23_d17 = R17_33_d17*s18;
		R18_31_d17 = R17_31_d17*c18;
		R18_32_d17 = R17_32_d17*c18;
		R18_33_d17 = R17_33_d17*c18;

		R18_21_d18 = -R17_21*s18 + R17_31*c18;
		R18_22_d18 = -R17_22*s18 + R17_32*c18;
		R18_23_d18 = -R17_23*s18 + R17_33*c18;
		R18_31_d18 = -R17_21*c18 - R17_31*s18;
		R18_32_d18 = -R17_22*c18 - R17_32*s18;
		R18_33_d18 = -R17_23*c18 - R17_33*s18;

		R19_11_d14 = R18_11_d14*c19 + R18_21_d14*s19;
		R19_12_d14 = R18_12_d14*c19 + R18_22_d14*s19;
		R19_13_d14 = R18_13_d14*c19 + R18_23_d14*s19;
		R19_21_d14 = -R18_11_d14*s19 + R18_21_d14*c19;
		R19_22_d14 = -R18_12_d14*s19 + R18_22_d14*c19;
		R19_23_d14 = -R18_13_d14*s19 + R18_23_d14*c19;
		R19_31_d14 = R18_31_d14;
		R19_32_d14 = R18_32_d14;
		R19_33_d14 = R18_33_d14;

		R19_11_d15 = R18_11_d15*c19 + R18_21_d15*s19;
		R19_12_d15 = R18_12_d15*c19 + R18_22_d15*s19;
		R19_13_d15 = R18_13_d15*c19 + R18_23_d15*s19;
		R19_21_d15 = -R18_11_d15*s19 + R18_21_d15*c19;
		R19_22_d15 = -R18_12_d15*s19 + R18_22_d15*c19;
		R19_23_d15 = -R18_13_d15*s19 + R18_23_d15*c19;
		R19_31_d15 = R18_31_d15;
		R19_32_d15 = R18_32_d15;
		R19_33_d15 = R18_33_d15;

		R19_11_d16 = R18_11_d16*c19 + R18_21_d16*s19;
		R19_12_d16 = R18_12_d16*c19 + R18_22_d16*s19;
		R19_13_d16 = R18_13_d16*c19 + R18_23_d16*s19;
		R19_21_d16 = -R18_11_d16*s19 + R18_21_d16*c19;
		R19_22_d16 = -R18_12_d16*s19 + R18_22_d16*c19;
		R19_23_d16 = -R18_13_d16*s19 + R18_23_d16*c19;
		R19_31_d16 = R18_31_d16;
		R19_32_d16 = R18_32_d16;
		R19_33_d16 = R18_33_d16;

		R19_11_d17 = R18_11_d17*c19 + R18_21_d17*s19;
		R19_12_d17 = R18_12_d17*c19 + R18_22_d17*s19;
		R19_13_d17 = R18_13_d17*c19 + R18_23_d17*s19;
		R19_21_d17 = -R18_11_d17*s19 + R18_21_d17*c19;
		R19_22_d17 = -R18_12_d17*s19 + R18_22_d17*c19;
		R19_23_d17 = -R18_13_d17*s19 + R18_23_d17*c19;
		R19_31_d17 = R18_31_d17;
		R19_32_d17 = R18_32_d17;
		R19_33_d17 = R18_33_d17;

		R19_11_d18 = R18_21_d18*s19;
		R19_12_d18 = R18_22_d18*s19;
		R19_13_d18 = R18_23_d18*s19;
		R19_21_d18 = R18_21_d18*c19;
		R19_22_d18 = R18_22_d18*c19;
		R19_23_d18 = R18_23_d18*c19;
		R19_31_d18 = R18_31_d18;
		R19_32_d18 = R18_32_d18;
		R19_33_d18 = R18_33_d18;

		R19_11_d19 = -R18_11*s19 + R18_21*c19;
		R19_12_d19 = -R18_12*s19 + R18_22*c19;
		R19_13_d19 = -R18_13*s19 + R18_23*c19;
		R19_21_d19 = -R18_11*c19 - R18_21*s19;
		R19_22_d19 = -R18_12*c19 - R18_22*s19;
		R19_23_d19 = -R18_13*c19 - R18_23*s19;

		R20_11_d14 = R19_11_d14*c20 - R19_31_d14*s20;
		R20_12_d14 = R19_12_d14*c20 - R19_32_d14*s20;
		R20_13_d14 = R19_13_d14*c20 - R19_33_d14*s20;
		R20_21_d14 = R19_21_d14;
		R20_22_d14 = R19_22_d14;
		R20_23_d14 = R19_23_d14;
		R20_31_d14 = R19_11_d14*s20 + R19_31_d14*c20;
		R20_32_d14 = R19_12_d14*s20 + R19_32_d14*c20;
		R20_33_d14 = R19_13_d14*s20 + R19_33_d14*c20;

		R20_11_d15 = R19_11_d15*c20 - R19_31_d15*s20;
		R20_12_d15 = R19_12_d15*c20 - R19_32_d15*s20;
		R20_13_d15 = R19_13_d15*c20 - R19_33_d15*s20;
		R20_21_d15 = R19_21_d15;
		R20_22_d15 = R19_22_d15;
		R20_23_d15 = R19_23_d15;
		R20_31_d15 = R19_11_d15*s20 + R19_31_d15*c20;
		R20_32_d15 = R19_12_d15*s20 + R19_32_d15*c20;
		R20_33_d15 = R19_13_d15*s20 + R19_33_d15*c20;

		R20_11_d16 = R19_11_d16*c20 - R19_31_d16*s20;
		R20_12_d16 = R19_12_d16*c20 - R19_32_d16*s20;
		R20_13_d16 = R19_13_d16*c20 - R19_33_d16*s20;
		R20_21_d16 = R19_21_d16;
		R20_22_d16 = R19_22_d16;
		R20_23_d16 = R19_23_d16;
		R20_31_d16 = R19_11_d16*s20 + R19_31_d16*c20;
		R20_32_d16 = R19_12_d16*s20 + R19_32_d16*c20;
		R20_33_d16 = R19_13_d16*s20 + R19_33_d16*c20;

		R20_11_d17 = R19_11_d17*c20 - R19_31_d17*s20;
		R20_12_d17 = R19_12_d17*c20 - R19_32_d17*s20;
		R20_13_d17 = R19_13_d17*c20 - R19_33_d17*s20;
		R20_21_d17 = R19_21_d17;
		R20_22_d17 = R19_22_d17;
		R20_23_d17 = R19_23_d17;
		R20_31_d17 = R19_11_d17*s20 + R19_31_d17*c20;
		R20_32_d17 = R19_12_d17*s20 + R19_32_d17*c20;
		R20_33_d17 = R19_13_d17*s20 + R19_33_d17*c20;

		R20_11_d18 = R19_11_d18*c20 - R19_31_d18*s20;
		R20_12_d18 = R19_12_d18*c20 - R19_32_d18*s20;
		R20_13_d18 = R19_13_d18*c20 - R19_33_d18*s20;
		R20_21_d18 = R19_21_d18;
		R20_22_d18 = R19_22_d18;
		R20_23_d18 = R19_23_d18;
		R20_31_d18 = R19_11_d18*s20 + R19_31_d18*c20;
		R20_32_d18 = R19_12_d18*s20 + R19_32_d18*c20;
		R20_33_d18 = R19_13_d18*s20 + R19_33_d18*c20;

		R20_11_d19 = R19_11_d19*c20;
		R20_12_d19 = R19_12_d19*c20;
		R20_13_d19 = R19_13_d19*c20;
		R20_21_d19 = R19_21_d19;
		R20_22_d19 = R19_22_d19;
		R20_23_d19 = R19_23_d19;
		R20_31_d19 = R19_11_d19*s20;
		R20_32_d19 = R19_12_d19*s20;
		R20_33_d19 = R19_13_d19*s20;

		R20_11_d20 = -R19_31*c20 - R19_11*s20;
		R20_12_d20 = -R19_32*c20 - R19_12*s20;
		R20_13_d20 = -R19_33*c20 - R19_13*s20;
		R20_31_d20 = R19_11*c20 - R19_31*s20;
		R20_32_d20 = R19_12*c20 - R19_32*s20;
		R20_33_d20 = R19_13*c20 - R19_33*s20;

		R21_11_d14 = R16_11_d14*c21 - R16_31_d14*s21;
		R21_12_d14 = R16_12_d14*c21 - R16_32_d14*s21;
		R21_13_d14 = R16_13_d14*c21 - R16_33_d14*s21;
		R21_21_d14 = R16_21_d14;
		R21_22_d14 = R16_22_d14;
		R21_23_d14 = R16_23_d14;
		R21_31_d14 = R16_11_d14*s21 + R16_31_d14*c21;
		R21_32_d14 = R16_12_d14*s21 + R16_32_d14*c21;
		R21_33_d14 = R16_13_d14*s21 + R16_33_d14*c21;

		R21_11_d15 = R16_11_d15*c21 - R16_31_d15*s21;
		R21_12_d15 = R16_12_d15*c21 - R16_32_d15*s21;
		R21_13_d15 = R16_13_d15*c21 - R16_33_d15*s21;
		R21_21_d15 = R16_21_d15;
		R21_22_d15 = R16_22_d15;
		R21_23_d15 = R16_23_d15;
		R21_31_d15 = R16_11_d15*s21 + R16_31_d15*c21;
		R21_32_d15 = R16_12_d15*s21 + R16_32_d15*c21;
		R21_33_d15 = R16_13_d15*s21 + R16_33_d15*c21;

		R21_11_d16 = R16_11_d16*c21;
		R21_12_d16 = R16_12_d16*c21;
		R21_13_d16 = R16_13_d16*c21;
		R21_21_d16 = R16_21_d16;
		R21_22_d16 = R16_22_d16;
		R21_23_d16 = R16_23_d16;
		R21_31_d16 = R16_11_d16*s21;
		R21_32_d16 = R16_12_d16*s21;
		R21_33_d16 = R16_13_d16*s21;

		R21_11_d21 = -R16_31*c21 - R16_11*s21;
		R21_12_d21 = -R16_32*c21 - R16_12*s21;
		R21_13_d21 = -R16_33*c21 - R16_13*s21;
		R21_31_d21 = R16_11*c21 - R16_31*s21;
		R21_32_d21 = R16_12*c21 - R16_32*s21;
		R21_33_d21 = R16_13*c21 - R16_33*s21;

		R22_11_d14 = R21_11_d14;
		R22_12_d14 = R21_12_d14;
		R22_13_d14 = R21_13_d14;
		R22_21_d14 = R21_21_d14*c22 + R21_31_d14*s22;
		R22_22_d14 = R21_22_d14*c22 + R21_32_d14*s22;
		R22_23_d14 = R21_23_d14*c22 + R21_33_d14*s22;
		R22_31_d14 = -R21_21_d14*s22 + R21_31_d14*c22;
		R22_32_d14 = -R21_22_d14*s22 + R21_32_d14*c22;
		R22_33_d14 = -R21_23_d14*s22 + R21_33_d14*c22;

		R22_11_d15 = R21_11_d15;
		R22_12_d15 = R21_12_d15;
		R22_13_d15 = R21_13_d15;
		R22_21_d15 = R21_21_d15*c22 + R21_31_d15*s22;
		R22_22_d15 = R21_22_d15*c22 + R21_32_d15*s22;
		R22_23_d15 = R21_23_d15*c22 + R21_33_d15*s22;
		R22_31_d15 = -R21_21_d15*s22 + R21_31_d15*c22;
		R22_32_d15 = -R21_22_d15*s22 + R21_32_d15*c22;
		R22_33_d15 = -R21_23_d15*s22 + R21_33_d15*c22;

		R22_11_d16 = R21_11_d16;
		R22_12_d16 = R21_12_d16;
		R22_13_d16 = R21_13_d16;
		R22_21_d16 = R21_21_d16*c22 + R21_31_d16*s22;
		R22_22_d16 = R21_22_d16*c22 + R21_32_d16*s22;
		R22_23_d16 = R21_23_d16*c22 + R21_33_d16*s22;
		R22_31_d16 = -R21_21_d16*s22 + R21_31_d16*c22;
		R22_32_d16 = -R21_22_d16*s22 + R21_32_d16*c22;
		R22_33_d16 = -R21_23_d16*s22 + R21_33_d16*c22;

		R22_11_d21 = R21_11_d21;
		R22_12_d21 = R21_12_d21;
		R22_13_d21 = R21_13_d21;
		R22_21_d21 = R21_31_d21*s22;
		R22_22_d21 = R21_32_d21*s22;
		R22_23_d21 = R21_33_d21*s22;
		R22_31_d21 = R21_31_d21*c22;
		R22_32_d21 = R21_32_d21*c22;
		R22_33_d21 = R21_33_d21*c22;

		R22_21_d22 = -R21_21*s22 + R21_31*c22;
		R22_22_d22 = -R21_22*s22 + R21_32*c22;
		R22_23_d22 = -R21_23*s22 + R21_33*c22;
		R22_31_d22 = -R21_21*c22 - R21_31*s22;
		R22_32_d22 = -R21_22*c22 - R21_32*s22;
		R22_33_d22 = -R21_23*c22 - R21_33*s22;

		R23_11_d14 = R22_11_d14*c23 + R22_21_d14*s23;
		R23_12_d14 = R22_12_d14*c23 + R22_22_d14*s23;
		R23_13_d14 = R22_13_d14*c23 + R22_23_d14*s23;
		R23_21_d14 = -R22_11_d14*s23 + R22_21_d14*c23;
		R23_22_d14 = -R22_12_d14*s23 + R22_22_d14*c23;
		R23_23_d14 = -R22_13_d14*s23 + R22_23_d14*c23;
		R23_31_d14 = R22_31_d14;
		R23_32_d14 = R22_32_d14;
		R23_33_d14 = R22_33_d14;

		R23_11_d15 = R22_11_d15*c23 + R22_21_d15*s23;
		R23_12_d15 = R22_12_d15*c23 + R22_22_d15*s23;
		R23_13_d15 = R22_13_d15*c23 + R22_23_d15*s23;
		R23_21_d15 = -R22_11_d15*s23 + R22_21_d15*c23;
		R23_22_d15 = -R22_12_d15*s23 + R22_22_d15*c23;
		R23_23_d15 = -R22_13_d15*s23 + R22_23_d15*c23;
		R23_31_d15 = R22_31_d15;
		R23_32_d15 = R22_32_d15;
		R23_33_d15 = R22_33_d15;

		R23_11_d16 = R22_11_d16*c23 + R22_21_d16*s23;
		R23_12_d16 = R22_12_d16*c23 + R22_22_d16*s23;
		R23_13_d16 = R22_13_d16*c23 + R22_23_d16*s23;
		R23_21_d16 = -R22_11_d16*s23 + R22_21_d16*c23;
		R23_22_d16 = -R22_12_d16*s23 + R22_22_d16*c23;
		R23_23_d16 = -R22_13_d16*s23 + R22_23_d16*c23;
		R23_31_d16 = R22_31_d16;
		R23_32_d16 = R22_32_d16;
		R23_33_d16 = R22_33_d16;

		R23_11_d21 = R22_11_d21*c23 + R22_21_d21*s23;
		R23_12_d21 = R22_12_d21*c23 + R22_22_d21*s23;
		R23_13_d21 = R22_13_d21*c23 + R22_23_d21*s23;
		R23_21_d21 = -R22_11_d21*s23 + R22_21_d21*c23;
		R23_22_d21 = -R22_12_d21*s23 + R22_22_d21*c23;
		R23_23_d21 = -R22_13_d21*s23 + R22_23_d21*c23;
		R23_31_d21 = R22_31_d21;
		R23_32_d21 = R22_32_d21;
		R23_33_d21 = R22_33_d21;

		R23_11_d22 = R22_21_d22*s23;
		R23_12_d22 = R22_22_d22*s23;
		R23_13_d22 = R22_23_d22*s23;
		R23_21_d22 = R22_21_d22*c23;
		R23_22_d22 = R22_22_d22*c23;
		R23_23_d22 = R22_23_d22*c23;
		R23_31_d22 = R22_31_d22;
		R23_32_d22 = R22_32_d22;
		R23_33_d22 = R22_33_d22;

		R23_11_d23 = -R22_11*s23 + R22_21*c23;
		R23_12_d23 = -R22_12*s23 + R22_22*c23;
		R23_13_d23 = -R22_13*s23 + R22_23*c23;
		R23_21_d23 = -R22_11*c23 - R22_21*s23;
		R23_22_d23 = -R22_12*c23 - R22_22*s23;
		R23_23_d23 = -R22_13*c23 - R22_23*s23;

		R24_11_d14 = R23_11_d14*c24 - R23_31_d14*s24;
		R24_12_d14 = R23_12_d14*c24 - R23_32_d14*s24;
		R24_13_d14 = R23_13_d14*c24 - R23_33_d14*s24;
		R24_21_d14 = R23_21_d14;
		R24_22_d14 = R23_22_d14;
		R24_23_d14 = R23_23_d14;
		R24_31_d14 = R23_11_d14*s24 + R23_31_d14*c24;
		R24_32_d14 = R23_12_d14*s24 + R23_32_d14*c24;
		R24_33_d14 = R23_13_d14*s24 + R23_33_d14*c24;

		R24_11_d15 = R23_11_d15*c24 - R23_31_d15*s24;
		R24_12_d15 = R23_12_d15*c24 - R23_32_d15*s24;
		R24_13_d15 = R23_13_d15*c24 - R23_33_d15*s24;
		R24_21_d15 = R23_21_d15;
		R24_22_d15 = R23_22_d15;
		R24_23_d15 = R23_23_d15;
		R24_31_d15 = R23_11_d15*s24 + R23_31_d15*c24;
		R24_32_d15 = R23_12_d15*s24 + R23_32_d15*c24;
		R24_33_d15 = R23_13_d15*s24 + R23_33_d15*c24;

		R24_11_d16 = R23_11_d16*c24 - R23_31_d16*s24;
		R24_12_d16 = R23_12_d16*c24 - R23_32_d16*s24;
		R24_13_d16 = R23_13_d16*c24 - R23_33_d16*s24;
		R24_21_d16 = R23_21_d16;
		R24_22_d16 = R23_22_d16;
		R24_23_d16 = R23_23_d16;
		R24_31_d16 = R23_11_d16*s24 + R23_31_d16*c24;
		R24_32_d16 = R23_12_d16*s24 + R23_32_d16*c24;
		R24_33_d16 = R23_13_d16*s24 + R23_33_d16*c24;

		R24_11_d21 = R23_11_d21*c24 - R23_31_d21*s24;
		R24_12_d21 = R23_12_d21*c24 - R23_32_d21*s24;
		R24_13_d21 = R23_13_d21*c24 - R23_33_d21*s24;
		R24_21_d21 = R23_21_d21;
		R24_22_d21 = R23_22_d21;
		R24_23_d21 = R23_23_d21;
		R24_31_d21 = R23_11_d21*s24 + R23_31_d21*c24;
		R24_32_d21 = R23_12_d21*s24 + R23_32_d21*c24;
		R24_33_d21 = R23_13_d21*s24 + R23_33_d21*c24;

		R24_11_d22 = R23_11_d22*c24 - R23_31_d22*s24;
		R24_12_d22 = R23_12_d22*c24 - R23_32_d22*s24;
		R24_13_d22 = R23_13_d22*c24 - R23_33_d22*s24;
		R24_21_d22 = R23_21_d22;
		R24_22_d22 = R23_22_d22;
		R24_23_d22 = R23_23_d22;
		R24_31_d22 = R23_11_d22*s24 + R23_31_d22*c24;
		R24_32_d22 = R23_12_d22*s24 + R23_32_d22*c24;
		R24_33_d22 = R23_13_d22*s24 + R23_33_d22*c24;

		R24_11_d23 = R23_11_d23*c24;
		R24_12_d23 = R23_12_d23*c24;
		R24_13_d23 = R23_13_d23*c24;
		R24_21_d23 = R23_21_d23;
		R24_22_d23 = R23_22_d23;
		R24_23_d23 = R23_23_d23;
		R24_31_d23 = R23_11_d23*s24;
		R24_32_d23 = R23_12_d23*s24;
		R24_33_d23 = R23_13_d23*s24;

		R24_11_d24 = -R23_31*c24 - R23_11*s24;
		R24_12_d24 = -R23_32*c24 - R23_12*s24;
		R24_13_d24 = -R23_33*c24 - R23_13*s24;
		R24_31_d24 = R23_11*c24 - R23_31*s24;
		R24_32_d24 = R23_12*c24 - R23_32*s24;
		R24_33_d24 = R23_13*c24 - R23_33*s24;
	}

	// joint absolute velocities
	om1_1 = omega_1;
	om1_2 = omega_2;
	om1_3 = omega_3;

	om2_1 = Om2*R1_21 + om1_1;
	om2_2 = Om2*R1_22 + om1_2;
	om2_3 = Om2*R1_23 + om1_3;

	om3_1 = Om3*R2_11 + om2_1;
	om3_2 = Om3*R2_12 + om2_2;
	om3_3 = Om3*R2_13 + om2_3;

	om4_1 = Om4*R3_31 + om3_1;
	om4_2 = Om4*R3_32 + om3_2;
	om4_3 = Om4*R3_33 + om3_3;

	om5_1 = Om5*R4_21 + om4_1;
	om5_2 = Om5*R4_22 + om4_2;
	om5_3 = Om5*R4_23 + om4_3;

	om6_1 = Om6*R5_11 + om5_1;
	om6_2 = Om6*R5_12 + om5_2;
	om6_3 = Om6*R5_13 + om5_3;

	om7_1 = Om7*R6_21 + om6_1;
	om7_2 = Om7*R6_22 + om6_2;
	om7_3 = Om7*R6_23 + om6_3;

	om8_1 = Om8*R1_21 + om1_1;
	om8_2 = Om8*R1_22 + om1_2;
	om8_3 = Om8*R1_23 + om1_3;

	om9_1 = Om9*R8_11 + om8_1;
	om9_2 = Om9*R8_12 + om8_2;
	om9_3 = Om9*R8_13 + om8_3;

	om10_1 = Om10*R9_31 + om9_1;
	om10_2 = Om10*R9_32 + om9_2;
	om10_3 = Om10*R9_33 + om9_3;

	om11_1 = Om11*R10_21 + om10_1;
	om11_2 = Om11*R10_22 + om10_2;
	om11_3 = Om11*R10_23 + om10_3;

	om12_1 = Om12*R11_11 + om11_1;
	om12_2 = Om12*R11_12 + om11_2;
	om12_3 = Om12*R11_13 + om11_3;

	om13_1 = Om13*R12_21 + om12_1;
	om13_2 = Om13*R12_22 + om12_2;
	om13_3 = Om13*R12_23 + om12_3;

	om14_1 = Om14*R1_11 + om1_1;
	om14_2 = Om14*R1_12 + om1_2;
	om14_3 = Om14*R1_13 + om1_3;

	om15_1 = Om15*R14_21 + om14_1;
	om15_2 = Om15*R14_22 + om14_2;
	om15_3 = Om15*R14_23 + om14_3;

	om16_1 = Om16*R15_31 + om15_1;
	om16_2 = Om16*R15_32 + om15_2;
	om16_3 = Om16*R15_33 + om15_3;

	om17_1 = Om17*R16_21 + om16_1;
	om17_2 = Om17*R16_22 + om16_2;
	om17_3 = Om17*R16_23 + om16_3;

	om18_1 = Om18*R17_11 + om17_1;
	om18_2 = Om18*R17_12 + om17_2;
	om18_3 = Om18*R17_13 + om17_3;

	om19_1 = Om19*R18_31 + om18_1;
	om19_2 = Om19*R18_32 + om18_2;
	om19_3 = Om19*R18_33 + om18_3;

	om20_1 = Om20*R19_21 + om19_1;
	om20_2 = Om20*R19_22 + om19_2;
	om20_3 = Om20*R19_23 + om19_3;

	om21_1 = Om21*R16_21 + om16_1;
	om21_2 = Om21*R16_22 + om16_2;
	om21_3 = Om21*R16_23 + om16_3;

	om22_1 = Om22*R21_11 + om21_1;
	om22_2 = Om22*R21_12 + om21_2;
	om22_3 = Om22*R21_13 + om21_3;

	om23_1 = Om23*R22_31 + om22_1;
	om23_2 = Om23*R22_32 + om22_2;
	om23_3 = Om23*R22_33 + om22_3;

	om24_1 = Om24*R23_21 + om23_1;
	om24_2 = Om24*R23_22 + om23_2;
	om24_3 = Om24*R23_23 + om23_3;


	// anchor point absolute positions and velocities
	x2_1 = DPT_2_2*R1_21;
	x2_2 = DPT_2_2*R1_22;
	x2_3 = DPT_2_2*R1_23;

	xp2_1 = -DPT_2_2*R1_22*om1_3 + DPT_2_2*R1_23*om1_2;
	xp2_2 = DPT_2_2*R1_21*om1_3 - DPT_2_2*R1_23*om1_1;
	xp2_3 = -DPT_2_2*R1_21*om1_2 + DPT_2_2*R1_22*om1_1;

	x3_1 = DPT_2_6*R2_21 + x2_1;
	x3_2 = DPT_2_6*R2_22 + x2_2;
	x3_3 = DPT_2_6*R2_23 + x2_3;

	xp3_1 = -DPT_2_6*R2_22*om2_3 + DPT_2_6*R2_23*om2_2 + xp2_1;
	xp3_2 = DPT_2_6*R2_21*om2_3 - DPT_2_6*R2_23*om2_1 + xp2_2;
	xp3_3 = -DPT_2_6*R2_21*om2_2 + DPT_2_6*R2_22*om2_1 + xp2_3;

	x4_1 = DPT_3_8*R3_31 + x3_1;
	x4_2 = DPT_3_8*R3_32 + x3_2;
	x4_3 = DPT_3_8*R3_33 + x3_3;

	xp4_1 = -DPT_3_8*R3_32*om3_3 + DPT_3_8*R3_33*om3_2 + xp3_1;
	xp4_2 = DPT_3_8*R3_31*om3_3 - DPT_3_8*R3_33*om3_1 + xp3_2;
	xp4_3 = -DPT_3_8*R3_31*om3_2 + DPT_3_8*R3_32*om3_1 + xp3_3;

	x5_1 = DPT_3_10*R4_31 + x4_1;
	x5_2 = DPT_3_10*R4_32 + x4_2;
	x5_3 = DPT_3_10*R4_33 + x4_3;

	xp5_1 = -DPT_3_10*R4_32*om4_3 + DPT_3_10*R4_33*om4_2 + xp4_1;
	xp5_2 = DPT_3_10*R4_31*om4_3 - DPT_3_10*R4_33*om4_1 + xp4_2;
	xp5_3 = -DPT_3_10*R4_31*om4_2 + DPT_3_10*R4_32*om4_1 + xp4_3;

	x6_1 = DPT_3_12*R5_31 + x5_1;
	x6_2 = DPT_3_12*R5_32 + x5_2;
	x6_3 = DPT_3_12*R5_33 + x5_3;

	xp6_1 = -DPT_3_12*R5_32*om5_3 + DPT_3_12*R5_33*om5_2 + xp5_1;
	xp6_2 = DPT_3_12*R5_31*om5_3 - DPT_3_12*R5_33*om5_1 + xp5_2;
	xp6_3 = -DPT_3_12*R5_31*om5_2 + DPT_3_12*R5_32*om5_1 + xp5_3;

	x7_1 = x6_1;
	x7_2 = x6_2;
	x7_3 = x6_3;

	xp7_1 = xp6_1;
	xp7_2 = xp6_2;
	xp7_3 = xp6_3;

	x8_1 = DPT_2_3*R1_21;
	x8_2 = DPT_2_3*R1_22;
	x8_3 = DPT_2_3*R1_23;

	xp8_1 = -DPT_2_3*R1_22*om1_3 + DPT_2_3*R1_23*om1_2;
	xp8_2 = DPT_2_3*R1_21*om1_3 - DPT_2_3*R1_23*om1_1;
	xp8_3 = -DPT_2_3*R1_21*om1_2 + DPT_2_3*R1_22*om1_1;

	x9_1 = DPT_2_18*R8_21 + x8_1;
	x9_2 = DPT_2_18*R8_22 + x8_2;
	x9_3 = DPT_2_18*R8_23 + x8_3;

	xp9_1 = -DPT_2_18*R8_22*om8_3 + DPT_2_18*R8_23*om8_2 + xp8_1;
	xp9_2 = DPT_2_18*R8_21*om8_3 - DPT_2_18*R8_23*om8_1 + xp8_2;
	xp9_3 = -DPT_2_18*R8_21*om8_2 + DPT_2_18*R8_22*om8_1 + xp8_3;

	x10_1 = DPT_3_20*R9_31 + x9_1;
	x10_2 = DPT_3_20*R9_32 + x9_2;
	x10_3 = DPT_3_20*R9_33 + x9_3;

	xp10_1 = -DPT_3_20*R9_32*om9_3 + DPT_3_20*R9_33*om9_2 + xp9_1;
	xp10_2 = DPT_3_20*R9_31*om9_3 - DPT_3_20*R9_33*om9_1 + xp9_2;
	xp10_3 = -DPT_3_20*R9_31*om9_2 + DPT_3_20*R9_32*om9_1 + xp9_3;

	x11_1 = DPT_3_22*R10_31 + x10_1;
	x11_2 = DPT_3_22*R10_32 + x10_2;
	x11_3 = DPT_3_22*R10_33 + x10_3;

	xp11_1 = -DPT_3_22*R10_32*om10_3 + DPT_3_22*R10_33*om10_2 + xp10_1;
	xp11_2 = DPT_3_22*R10_31*om10_3 - DPT_3_22*R10_33*om10_1 + xp10_2;
	xp11_3 = -DPT_3_22*R10_31*om10_2 + DPT_3_22*R10_32*om10_1 + xp10_3;

	x12_1 = DPT_3_24*R11_31 + x11_1;
	x12_2 = DPT_3_24*R11_32 + x11_2;
	x12_3 = DPT_3_24*R11_33 + x11_3;

	xp12_1 = -DPT_3_24*R11_32*om11_3 + DPT_3_24*R11_33*om11_2 + xp11_1;
	xp12_2 = DPT_3_24*R11_31*om11_3 - DPT_3_24*R11_33*om11_1 + xp11_2;
	xp12_3 = -DPT_3_24*R11_31*om11_2 + DPT_3_24*R11_32*om11_1 + xp11_3;

	x13_1 = x12_1;
	x13_2 = x12_2;
	x13_3 = x12_3;

	xp13_1 = xp12_1;
	xp13_2 = xp12_2;
	xp13_3 = xp12_3;

	x14_1 = DPT_1_4*R1_11 + DPT_3_4*R1_31;
	x14_2 = DPT_1_4*R1_12 + DPT_3_4*R1_32;
	x14_3 = DPT_1_4*R1_13 + DPT_3_4*R1_33;

	xp14_1 = om1_2*(DPT_1_4*R1_13 + DPT_3_4*R1_33) - om1_3*(DPT_1_4*R1_12 + DPT_3_4*R1_32);
	xp14_2 = -om1_1*(DPT_1_4*R1_13 + DPT_3_4*R1_33) + om1_3*(DPT_1_4*R1_11 + DPT_3_4*R1_31);
	xp14_3 = om1_1*(DPT_1_4*R1_12 + DPT_3_4*R1_32) - om1_2*(DPT_1_4*R1_11 + DPT_3_4*R1_31);

	x15_1 = x14_1;
	x15_2 = x14_2;
	x15_3 = x14_3;

	xp15_1 = xp14_1;
	xp15_2 = xp14_2;
	xp15_3 = xp14_3;

	x16_1 = DPT_3_32*R15_31 + x15_1;
	x16_2 = DPT_3_32*R15_32 + x15_2;
	x16_3 = DPT_3_32*R15_33 + x15_3;

	xp16_1 = -DPT_3_32*R15_32*om15_3 + DPT_3_32*R15_33*om15_2 + xp15_1;
	xp16_2 = DPT_3_32*R15_31*om15_3 - DPT_3_32*R15_33*om15_1 + xp15_2;
	xp16_3 = -DPT_3_32*R15_31*om15_2 + DPT_3_32*R15_32*om15_1 + xp15_3;

	x17_1 = DPT_1_36*R16_11 + DPT_2_36*R16_21 + DPT_3_36*R16_31 + x16_1;
	x17_2 = DPT_1_36*R16_12 + DPT_2_36*R16_22 + DPT_3_36*R16_32 + x16_2;
	x17_3 = DPT_1_36*R16_13 + DPT_2_36*R16_23 + DPT_3_36*R16_33 + x16_3;

	xp17_1 = om16_2*(DPT_1_36*R16_13 + DPT_2_36*R16_23 + DPT_3_36*R16_33) - om16_3*(DPT_1_36*R16_12 + DPT_2_36*R16_22 + DPT_3_36*R16_32) + xp16_1;
	xp17_2 = -om16_1*(DPT_1_36*R16_13 + DPT_2_36*R16_23 + DPT_3_36*R16_33) + om16_3*(DPT_1_36*R16_11 + DPT_2_36*R16_21 + DPT_3_36*R16_31) + xp16_2;
	xp17_3 = om16_1*(DPT_1_36*R16_12 + DPT_2_36*R16_22 + DPT_3_36*R16_32) - om16_2*(DPT_1_36*R16_11 + DPT_2_36*R16_21 + DPT_3_36*R16_31) + xp16_3;

	x18_1 = DPT_2_39*R17_21 + x17_1;
	x18_2 = DPT_2_39*R17_22 + x17_2;
	x18_3 = DPT_2_39*R17_23 + x17_3;

	xp18_1 = -DPT_2_39*R17_22*om17_3 + DPT_2_39*R17_23*om17_2 + xp17_1;
	xp18_2 = DPT_2_39*R17_21*om17_3 - DPT_2_39*R17_23*om17_1 + xp17_2;
	xp18_3 = -DPT_2_39*R17_21*om17_2 + DPT_2_39*R17_22*om17_1 + xp17_3;

	x19_1 = DPT_3_41*R18_31 + x18_1;
	x19_2 = DPT_3_41*R18_32 + x18_2;
	x19_3 = DPT_3_41*R18_33 + x18_3;

	xp19_1 = -DPT_3_41*R18_32*om18_3 + DPT_3_41*R18_33*om18_2 + xp18_1;
	xp19_2 = DPT_3_41*R18_31*om18_3 - DPT_3_41*R18_33*om18_1 + xp18_2;
	xp19_3 = -DPT_3_41*R18_31*om18_2 + DPT_3_41*R18_32*om18_1 + xp18_3;

	x20_1 = DPT_3_43*R19_31 + x19_1;
	x20_2 = DPT_3_43*R19_32 + x19_2;
	x20_3 = DPT_3_43*R19_33 + x19_3;

	xp20_1 = -DPT_3_43*R19_32*om19_3 + DPT_3_43*R19_33*om19_2 + xp19_1;
	xp20_2 = DPT_3_43*R19_31*om19_3 - DPT_3_43*R19_33*om19_1 + xp19_2;
	xp20_3 = -DPT_3_43*R19_31*om19_2 + DPT_3_43*R19_32*om19_1 + xp19_3;

	x21_1 = DPT_1_37*R16_11 + DPT_2_37*R16_21 + DPT_3_37*R16_31 + x16_1;
	x21_2 = DPT_1_37*R16_12 + DPT_2_37*R16_22 + DPT_3_37*R16_32 + x16_2;
	x21_3 = DPT_1_37*R16_13 + DPT_2_37*R16_23 + DPT_3_37*R16_33 + x16_3;

	xp21_1 = om16_2*(DPT_1_37*R16_13 + DPT_2_37*R16_23 + DPT_3_37*R16_33) - om16_3*(DPT_1_37*R16_12 + DPT_2_37*R16_22 + DPT_3_37*R16_32) + xp16_1;
	xp21_2 = -om16_1*(DPT_1_37*R16_13 + DPT_2_37*R16_23 + DPT_3_37*R16_33) + om16_3*(DPT_1_37*R16_11 + DPT_2_37*R16_21 + DPT_3_37*R16_31) + xp16_2;
	xp21_3 = om16_1*(DPT_1_37*R16_12 + DPT_2_37*R16_22 + DPT_3_37*R16_32) - om16_2*(DPT_1_37*R16_11 + DPT_2_37*R16_21 + DPT_3_37*R16_31) + xp16_3;

	x22_1 = DPT_2_46*R21_21 + x21_1;
	x22_2 = DPT_2_46*R21_22 + x21_2;
	x22_3 = DPT_2_46*R21_23 + x21_3;

	xp22_1 = -DPT_2_46*R21_22*om21_3 + DPT_2_46*R21_23*om21_2 + xp21_1;
	xp22_2 = DPT_2_46*R21_21*om21_3 - DPT_2_46*R21_23*om21_1 + xp21_2;
	xp22_3 = -DPT_2_46*R21_21*om21_2 + DPT_2_46*R21_22*om21_1 + xp21_3;

	x23_1 = DPT_3_48*R22_31 + x22_1;
	x23_2 = DPT_3_48*R22_32 + x22_2;
	x23_3 = DPT_3_48*R22_33 + x22_3;

	xp23_1 = -DPT_3_48*R22_32*om22_3 + DPT_3_48*R22_33*om22_2 + xp22_1;
	xp23_2 = DPT_3_48*R22_31*om22_3 - DPT_3_48*R22_33*om22_1 + xp22_2;
	xp23_3 = -DPT_3_48*R22_31*om22_2 + DPT_3_48*R22_32*om22_1 + xp22_3;

	x24_1 = DPT_3_50*R23_31 + x23_1;
	x24_2 = DPT_3_50*R23_32 + x23_2;
	x24_3 = DPT_3_50*R23_33 + x23_3;

	xp24_1 = -DPT_3_50*R23_32*om23_3 + DPT_3_50*R23_33*om23_2 + xp23_1;
	xp24_2 = DPT_3_50*R23_31*om23_3 - DPT_3_50*R23_33*om23_1 + xp23_2;
	xp24_3 = -DPT_3_50*R23_31*om23_2 + DPT_3_50*R23_32*om23_1 + xp23_3;


	// jacobian anchor point positions
	if (flag_jacob)
	{
		x4_1_d2 = DPT_3_8*R3_31_d2;
		x4_2_d2 = DPT_3_8*R3_32_d2;
		x4_3_d2 = DPT_3_8*R3_33_d2;

		x4_1_d3 = DPT_3_8*R3_31_d3;
		x4_2_d3 = DPT_3_8*R3_32_d3;
		x4_3_d3 = DPT_3_8*R3_33_d3;

		x5_1_d2 = DPT_3_10*R4_31_d2 + x4_1_d2;
		x5_2_d2 = DPT_3_10*R4_32_d2 + x4_2_d2;
		x5_3_d2 = DPT_3_10*R4_33_d2 + x4_3_d2;

		x5_1_d3 = DPT_3_10*R4_31_d3 + x4_1_d3;
		x5_2_d3 = DPT_3_10*R4_32_d3 + x4_2_d3;
		x5_3_d3 = DPT_3_10*R4_33_d3 + x4_3_d3;

		x6_1_d2 = DPT_3_12*R5_31_d2 + x5_1_d2;
		x6_2_d2 = DPT_3_12*R5_32_d2 + x5_2_d2;
		x6_3_d2 = DPT_3_12*R5_33_d2 + x5_3_d2;

		x6_1_d3 = DPT_3_12*R5_31_d3 + x5_1_d3;
		x6_2_d3 = DPT_3_12*R5_32_d3 + x5_2_d3;
		x6_3_d3 = DPT_3_12*R5_33_d3 + x5_3_d3;

		x6_1_d4 = DPT_3_12*R5_31_d4;
		x6_2_d4 = DPT_3_12*R5_32_d4;
		x6_3_d4 = DPT_3_12*R5_33_d4;

		x6_1_d5 = DPT_3_12*R5_31_d5;
		x6_2_d5 = DPT_3_12*R5_32_d5;
		x6_3_d5 = DPT_3_12*R5_33_d5;

		x7_1_d2 = x6_1_d2;
		x7_2_d2 = x6_2_d2;
		x7_3_d2 = x6_3_d2;

		x7_1_d3 = x6_1_d3;
		x7_2_d3 = x6_2_d3;
		x7_3_d3 = x6_3_d3;

		x7_1_d4 = x6_1_d4;
		x7_2_d4 = x6_2_d4;
		x7_3_d4 = x6_3_d4;

		x7_1_d5 = x6_1_d5;
		x7_2_d5 = x6_2_d5;
		x7_3_d5 = x6_3_d5;

		x10_1_d8 = DPT_3_20*R9_31_d8;
		x10_2_d8 = DPT_3_20*R9_32_d8;
		x10_3_d8 = DPT_3_20*R9_33_d8;

		x10_1_d9 = DPT_3_20*R9_31_d9;
		x10_2_d9 = DPT_3_20*R9_32_d9;
		x10_3_d9 = DPT_3_20*R9_33_d9;

		x11_1_d8 = DPT_3_22*R10_31_d8 + x10_1_d8;
		x11_2_d8 = DPT_3_22*R10_32_d8 + x10_2_d8;
		x11_3_d8 = DPT_3_22*R10_33_d8 + x10_3_d8;

		x11_1_d9 = DPT_3_22*R10_31_d9 + x10_1_d9;
		x11_2_d9 = DPT_3_22*R10_32_d9 + x10_2_d9;
		x11_3_d9 = DPT_3_22*R10_33_d9 + x10_3_d9;

		x12_1_d8 = DPT_3_24*R11_31_d8 + x11_1_d8;
		x12_2_d8 = DPT_3_24*R11_32_d8 + x11_2_d8;
		x12_3_d8 = DPT_3_24*R11_33_d8 + x11_3_d8;

		x12_1_d9 = DPT_3_24*R11_31_d9 + x11_1_d9;
		x12_2_d9 = DPT_3_24*R11_32_d9 + x11_2_d9;
		x12_3_d9 = DPT_3_24*R11_33_d9 + x11_3_d9;

		x12_1_d10 = DPT_3_24*R11_31_d10;
		x12_2_d10 = DPT_3_24*R11_32_d10;
		x12_3_d10 = DPT_3_24*R11_33_d10;

		x12_1_d11 = DPT_3_24*R11_31_d11;
		x12_2_d11 = DPT_3_24*R11_32_d11;
		x12_3_d11 = DPT_3_24*R11_33_d11;

		x13_1_d8 = x12_1_d8;
		x13_2_d8 = x12_2_d8;
		x13_3_d8 = x12_3_d8;

		x13_1_d9 = x12_1_d9;
		x13_2_d9 = x12_2_d9;
		x13_3_d9 = x12_3_d9;

		x13_1_d10 = x12_1_d10;
		x13_2_d10 = x12_2_d10;
		x13_3_d10 = x12_3_d10;

		x13_1_d11 = x12_1_d11;
		x13_2_d11 = x12_2_d11;
		x13_3_d11 = x12_3_d11;

		x16_1_d14 = DPT_3_32*R15_31_d14;
		x16_2_d14 = DPT_3_32*R15_32_d14;
		x16_3_d14 = DPT_3_32*R15_33_d14;

		x16_1_d15 = DPT_3_32*R15_31_d15;
		x16_2_d15 = DPT_3_32*R15_32_d15;
		x16_3_d15 = DPT_3_32*R15_33_d15;

		x17_1_d14 = DPT_1_36*R16_11_d14 + DPT_2_36*R16_21_d14 + DPT_3_36*R16_31_d14 + x16_1_d14;
		x17_2_d14 = DPT_1_36*R16_12_d14 + DPT_2_36*R16_22_d14 + DPT_3_36*R16_32_d14 + x16_2_d14;
		x17_3_d14 = DPT_1_36*R16_13_d14 + DPT_2_36*R16_23_d14 + DPT_3_36*R16_33_d14 + x16_3_d14;

		x17_1_d15 = DPT_1_36*R16_11_d15 + DPT_2_36*R16_21_d15 + DPT_3_36*R16_31_d15 + x16_1_d15;
		x17_2_d15 = DPT_1_36*R16_12_d15 + DPT_2_36*R16_22_d15 + DPT_3_36*R16_32_d15 + x16_2_d15;
		x17_3_d15 = DPT_1_36*R16_13_d15 + DPT_2_36*R16_23_d15 + DPT_3_36*R16_33_d15 + x16_3_d15;

		x17_1_d16 = DPT_1_36*R16_11_d16 + DPT_2_36*R16_21_d16;
		x17_2_d16 = DPT_1_36*R16_12_d16 + DPT_2_36*R16_22_d16;
		x17_3_d16 = DPT_1_36*R16_13_d16 + DPT_2_36*R16_23_d16;

		x18_1_d14 = DPT_2_39*R17_21_d14 + x17_1_d14;
		x18_2_d14 = DPT_2_39*R17_22_d14 + x17_2_d14;
		x18_3_d14 = DPT_2_39*R17_23_d14 + x17_3_d14;

		x18_1_d15 = DPT_2_39*R17_21_d15 + x17_1_d15;
		x18_2_d15 = DPT_2_39*R17_22_d15 + x17_2_d15;
		x18_3_d15 = DPT_2_39*R17_23_d15 + x17_3_d15;

		x18_1_d16 = DPT_2_39*R17_21_d16 + x17_1_d16;
		x18_2_d16 = DPT_2_39*R17_22_d16 + x17_2_d16;
		x18_3_d16 = DPT_2_39*R17_23_d16 + x17_3_d16;

		x19_1_d14 = DPT_3_41*R18_31_d14 + x18_1_d14;
		x19_2_d14 = DPT_3_41*R18_32_d14 + x18_2_d14;
		x19_3_d14 = DPT_3_41*R18_33_d14 + x18_3_d14;

		x19_1_d15 = DPT_3_41*R18_31_d15 + x18_1_d15;
		x19_2_d15 = DPT_3_41*R18_32_d15 + x18_2_d15;
		x19_3_d15 = DPT_3_41*R18_33_d15 + x18_3_d15;

		x19_1_d16 = DPT_3_41*R18_31_d16 + x18_1_d16;
		x19_2_d16 = DPT_3_41*R18_32_d16 + x18_2_d16;
		x19_3_d16 = DPT_3_41*R18_33_d16 + x18_3_d16;

		x19_1_d17 = DPT_3_41*R18_31_d17;
		x19_2_d17 = DPT_3_41*R18_32_d17;
		x19_3_d17 = DPT_3_41*R18_33_d17;

		x19_1_d18 = DPT_3_41*R18_31_d18;
		x19_2_d18 = DPT_3_41*R18_32_d18;
		x19_3_d18 = DPT_3_41*R18_33_d18;

		x20_1_d14 = DPT_3_43*R19_31_d14 + x19_1_d14;
		x20_2_d14 = DPT_3_43*R19_32_d14 + x19_2_d14;
		x20_3_d14 = DPT_3_43*R19_33_d14 + x19_3_d14;

		x20_1_d15 = DPT_3_43*R19_31_d15 + x19_1_d15;
		x20_2_d15 = DPT_3_43*R19_32_d15 + x19_2_d15;
		x20_3_d15 = DPT_3_43*R19_33_d15 + x19_3_d15;

		x20_1_d16 = DPT_3_43*R19_31_d16 + x19_1_d16;
		x20_2_d16 = DPT_3_43*R19_32_d16 + x19_2_d16;
		x20_3_d16 = DPT_3_43*R19_33_d16 + x19_3_d16;

		x20_1_d17 = DPT_3_43*R19_31_d17 + x19_1_d17;
		x20_2_d17 = DPT_3_43*R19_32_d17 + x19_2_d17;
		x20_3_d17 = DPT_3_43*R19_33_d17 + x19_3_d17;

		x20_1_d18 = DPT_3_43*R19_31_d18 + x19_1_d18;
		x20_2_d18 = DPT_3_43*R19_32_d18 + x19_2_d18;
		x20_3_d18 = DPT_3_43*R19_33_d18 + x19_3_d18;

		x21_1_d14 = DPT_1_37*R16_11_d14 + DPT_2_37*R16_21_d14 + DPT_3_37*R16_31_d14 + x16_1_d14;
		x21_2_d14 = DPT_1_37*R16_12_d14 + DPT_2_37*R16_22_d14 + DPT_3_37*R16_32_d14 + x16_2_d14;
		x21_3_d14 = DPT_1_37*R16_13_d14 + DPT_2_37*R16_23_d14 + DPT_3_37*R16_33_d14 + x16_3_d14;

		x21_1_d15 = DPT_1_37*R16_11_d15 + DPT_2_37*R16_21_d15 + DPT_3_37*R16_31_d15 + x16_1_d15;
		x21_2_d15 = DPT_1_37*R16_12_d15 + DPT_2_37*R16_22_d15 + DPT_3_37*R16_32_d15 + x16_2_d15;
		x21_3_d15 = DPT_1_37*R16_13_d15 + DPT_2_37*R16_23_d15 + DPT_3_37*R16_33_d15 + x16_3_d15;

		x21_1_d16 = DPT_1_37*R16_11_d16 + DPT_2_37*R16_21_d16;
		x21_2_d16 = DPT_1_37*R16_12_d16 + DPT_2_37*R16_22_d16;
		x21_3_d16 = DPT_1_37*R16_13_d16 + DPT_2_37*R16_23_d16;

		x22_1_d14 = DPT_2_46*R21_21_d14 + x21_1_d14;
		x22_2_d14 = DPT_2_46*R21_22_d14 + x21_2_d14;
		x22_3_d14 = DPT_2_46*R21_23_d14 + x21_3_d14;

		x22_1_d15 = DPT_2_46*R21_21_d15 + x21_1_d15;
		x22_2_d15 = DPT_2_46*R21_22_d15 + x21_2_d15;
		x22_3_d15 = DPT_2_46*R21_23_d15 + x21_3_d15;

		x22_1_d16 = DPT_2_46*R21_21_d16 + x21_1_d16;
		x22_2_d16 = DPT_2_46*R21_22_d16 + x21_2_d16;
		x22_3_d16 = DPT_2_46*R21_23_d16 + x21_3_d16;

		x23_1_d14 = DPT_3_48*R22_31_d14 + x22_1_d14;
		x23_2_d14 = DPT_3_48*R22_32_d14 + x22_2_d14;
		x23_3_d14 = DPT_3_48*R22_33_d14 + x22_3_d14;

		x23_1_d15 = DPT_3_48*R22_31_d15 + x22_1_d15;
		x23_2_d15 = DPT_3_48*R22_32_d15 + x22_2_d15;
		x23_3_d15 = DPT_3_48*R22_33_d15 + x22_3_d15;

		x23_1_d16 = DPT_3_48*R22_31_d16 + x22_1_d16;
		x23_2_d16 = DPT_3_48*R22_32_d16 + x22_2_d16;
		x23_3_d16 = DPT_3_48*R22_33_d16 + x22_3_d16;

		x23_1_d21 = DPT_3_48*R22_31_d21;
		x23_2_d21 = DPT_3_48*R22_32_d21;
		x23_3_d21 = DPT_3_48*R22_33_d21;

		x23_1_d22 = DPT_3_48*R22_31_d22;
		x23_2_d22 = DPT_3_48*R22_32_d22;
		x23_3_d22 = DPT_3_48*R22_33_d22;

		x24_1_d14 = DPT_3_50*R23_31_d14 + x23_1_d14;
		x24_2_d14 = DPT_3_50*R23_32_d14 + x23_2_d14;
		x24_3_d14 = DPT_3_50*R23_33_d14 + x23_3_d14;

		x24_1_d15 = DPT_3_50*R23_31_d15 + x23_1_d15;
		x24_2_d15 = DPT_3_50*R23_32_d15 + x23_2_d15;
		x24_3_d15 = DPT_3_50*R23_33_d15 + x23_3_d15;

		x24_1_d16 = DPT_3_50*R23_31_d16 + x23_1_d16;
		x24_2_d16 = DPT_3_50*R23_32_d16 + x23_2_d16;
		x24_3_d16 = DPT_3_50*R23_33_d16 + x23_3_d16;

		x24_1_d21 = DPT_3_50*R23_31_d21 + x23_1_d21;
		x24_2_d21 = DPT_3_50*R23_32_d21 + x23_2_d21;
		x24_3_d21 = DPT_3_50*R23_33_d21 + x23_3_d21;

		x24_1_d22 = DPT_3_50*R23_31_d22 + x23_1_d22;
		x24_2_d22 = DPT_3_50*R23_32_d22 + x23_2_d22;
		x24_3_d22 = DPT_3_50*R23_33_d22 + x23_3_d22;
	}

	// com absolute positions and velocities
	xg1_1 = L_1_6*R1_11 + L_2_6*R1_21 + L_3_6*R1_31;
	xg1_2 = L_1_6*R1_12 + L_2_6*R1_22 + L_3_6*R1_32;
	xg1_3 = L_1_6*R1_13 + L_2_6*R1_23 + L_3_6*R1_33;

	xgp1_1 = om1_2*(L_1_6*R1_13 + L_2_6*R1_23 + L_3_6*R1_33) - om1_3*(L_1_6*R1_12 + L_2_6*R1_22 + L_3_6*R1_32);
	xgp1_2 = -om1_1*(L_1_6*R1_13 + L_2_6*R1_23 + L_3_6*R1_33) + om1_3*(L_1_6*R1_11 + L_2_6*R1_21 + L_3_6*R1_31);
	xgp1_3 = om1_1*(L_1_6*R1_12 + L_2_6*R1_22 + L_3_6*R1_32) - om1_2*(L_1_6*R1_11 + L_2_6*R1_21 + L_3_6*R1_31);

	xg2_1 = L_1_7*R2_11 + L_2_7*R2_21 + L_3_7*R2_31 + x2_1;
	xg2_2 = L_1_7*R2_12 + L_2_7*R2_22 + L_3_7*R2_32 + x2_2;
	xg2_3 = L_1_7*R2_13 + L_2_7*R2_23 + L_3_7*R2_33 + x2_3;

	xgp2_1 = om2_2*(L_1_7*R2_13 + L_2_7*R2_23 + L_3_7*R2_33) - om2_3*(L_1_7*R2_12 + L_2_7*R2_22 + L_3_7*R2_32) + xp2_1;
	xgp2_2 = -om2_1*(L_1_7*R2_13 + L_2_7*R2_23 + L_3_7*R2_33) + om2_3*(L_1_7*R2_11 + L_2_7*R2_21 + L_3_7*R2_31) + xp2_2;
	xgp2_3 = om2_1*(L_1_7*R2_12 + L_2_7*R2_22 + L_3_7*R2_32) - om2_2*(L_1_7*R2_11 + L_2_7*R2_21 + L_3_7*R2_31) + xp2_3;

	xg3_1 = L_1_8*R3_11 + L_2_8*R3_21 + L_3_8*R3_31 + x3_1;
	xg3_2 = L_1_8*R3_12 + L_2_8*R3_22 + L_3_8*R3_32 + x3_2;
	xg3_3 = L_1_8*R3_13 + L_2_8*R3_23 + L_3_8*R3_33 + x3_3;

	xgp3_1 = om3_2*(L_1_8*R3_13 + L_2_8*R3_23 + L_3_8*R3_33) - om3_3*(L_1_8*R3_12 + L_2_8*R3_22 + L_3_8*R3_32) + xp3_1;
	xgp3_2 = -om3_1*(L_1_8*R3_13 + L_2_8*R3_23 + L_3_8*R3_33) + om3_3*(L_1_8*R3_11 + L_2_8*R3_21 + L_3_8*R3_31) + xp3_2;
	xgp3_3 = om3_1*(L_1_8*R3_12 + L_2_8*R3_22 + L_3_8*R3_32) - om3_2*(L_1_8*R3_11 + L_2_8*R3_21 + L_3_8*R3_31) + xp3_3;

	xg4_1 = L_1_9*R4_11 + L_2_9*R4_21 + L_3_9*R4_31 + x4_1;
	xg4_2 = L_1_9*R4_12 + L_2_9*R4_22 + L_3_9*R4_32 + x4_2;
	xg4_3 = L_1_9*R4_13 + L_2_9*R4_23 + L_3_9*R4_33 + x4_3;

	xgp4_1 = om4_2*(L_1_9*R4_13 + L_2_9*R4_23 + L_3_9*R4_33) - om4_3*(L_1_9*R4_12 + L_2_9*R4_22 + L_3_9*R4_32) + xp4_1;
	xgp4_2 = -om4_1*(L_1_9*R4_13 + L_2_9*R4_23 + L_3_9*R4_33) + om4_3*(L_1_9*R4_11 + L_2_9*R4_21 + L_3_9*R4_31) + xp4_2;
	xgp4_3 = om4_1*(L_1_9*R4_12 + L_2_9*R4_22 + L_3_9*R4_32) - om4_2*(L_1_9*R4_11 + L_2_9*R4_21 + L_3_9*R4_31) + xp4_3;

	xg5_1 = L_1_10*R5_11 + L_2_10*R5_21 + L_3_10*R5_31 + x5_1;
	xg5_2 = L_1_10*R5_12 + L_2_10*R5_22 + L_3_10*R5_32 + x5_2;
	xg5_3 = L_1_10*R5_13 + L_2_10*R5_23 + L_3_10*R5_33 + x5_3;

	xgp5_1 = om5_2*(L_1_10*R5_13 + L_2_10*R5_23 + L_3_10*R5_33) - om5_3*(L_1_10*R5_12 + L_2_10*R5_22 + L_3_10*R5_32) + xp5_1;
	xgp5_2 = -om5_1*(L_1_10*R5_13 + L_2_10*R5_23 + L_3_10*R5_33) + om5_3*(L_1_10*R5_11 + L_2_10*R5_21 + L_3_10*R5_31) + xp5_2;
	xgp5_3 = om5_1*(L_1_10*R5_12 + L_2_10*R5_22 + L_3_10*R5_32) - om5_2*(L_1_10*R5_11 + L_2_10*R5_21 + L_3_10*R5_31) + xp5_3;

	xg6_1 = L_1_11*R6_11 + L_2_11*R6_21 + L_3_11*R6_31 + x6_1;
	xg6_2 = L_1_11*R6_12 + L_2_11*R6_22 + L_3_11*R6_32 + x6_2;
	xg6_3 = L_1_11*R6_13 + L_2_11*R6_23 + L_3_11*R6_33 + x6_3;

	xgp6_1 = om6_2*(L_1_11*R6_13 + L_2_11*R6_23 + L_3_11*R6_33) - om6_3*(L_1_11*R6_12 + L_2_11*R6_22 + L_3_11*R6_32) + xp6_1;
	xgp6_2 = -om6_1*(L_1_11*R6_13 + L_2_11*R6_23 + L_3_11*R6_33) + om6_3*(L_1_11*R6_11 + L_2_11*R6_21 + L_3_11*R6_31) + xp6_2;
	xgp6_3 = om6_1*(L_1_11*R6_12 + L_2_11*R6_22 + L_3_11*R6_32) - om6_2*(L_1_11*R6_11 + L_2_11*R6_21 + L_3_11*R6_31) + xp6_3;

	xg7_1 = L_1_12*R7_11 + L_3_12*R7_31 + x7_1;
	xg7_2 = L_1_12*R7_12 + L_3_12*R7_32 + x7_2;
	xg7_3 = L_1_12*R7_13 + L_3_12*R7_33 + x7_3;

	xgp7_1 = om7_2*(L_1_12*R7_13 + L_3_12*R7_33) - om7_3*(L_1_12*R7_12 + L_3_12*R7_32) + xp7_1;
	xgp7_2 = -om7_1*(L_1_12*R7_13 + L_3_12*R7_33) + om7_3*(L_1_12*R7_11 + L_3_12*R7_31) + xp7_2;
	xgp7_3 = om7_1*(L_1_12*R7_12 + L_3_12*R7_32) - om7_2*(L_1_12*R7_11 + L_3_12*R7_31) + xp7_3;

	xg8_1 = L_1_13*R8_11 + L_2_13*R8_21 + L_3_13*R8_31 + x8_1;
	xg8_2 = L_1_13*R8_12 + L_2_13*R8_22 + L_3_13*R8_32 + x8_2;
	xg8_3 = L_1_13*R8_13 + L_2_13*R8_23 + L_3_13*R8_33 + x8_3;

	xgp8_1 = om8_2*(L_1_13*R8_13 + L_2_13*R8_23 + L_3_13*R8_33) - om8_3*(L_1_13*R8_12 + L_2_13*R8_22 + L_3_13*R8_32) + xp8_1;
	xgp8_2 = -om8_1*(L_1_13*R8_13 + L_2_13*R8_23 + L_3_13*R8_33) + om8_3*(L_1_13*R8_11 + L_2_13*R8_21 + L_3_13*R8_31) + xp8_2;
	xgp8_3 = om8_1*(L_1_13*R8_12 + L_2_13*R8_22 + L_3_13*R8_32) - om8_2*(L_1_13*R8_11 + L_2_13*R8_21 + L_3_13*R8_31) + xp8_3;

	xg9_1 = L_1_14*R9_11 + L_2_14*R9_21 + L_3_14*R9_31 + x9_1;
	xg9_2 = L_1_14*R9_12 + L_2_14*R9_22 + L_3_14*R9_32 + x9_2;
	xg9_3 = L_1_14*R9_13 + L_2_14*R9_23 + L_3_14*R9_33 + x9_3;

	xgp9_1 = om9_2*(L_1_14*R9_13 + L_2_14*R9_23 + L_3_14*R9_33) - om9_3*(L_1_14*R9_12 + L_2_14*R9_22 + L_3_14*R9_32) + xp9_1;
	xgp9_2 = -om9_1*(L_1_14*R9_13 + L_2_14*R9_23 + L_3_14*R9_33) + om9_3*(L_1_14*R9_11 + L_2_14*R9_21 + L_3_14*R9_31) + xp9_2;
	xgp9_3 = om9_1*(L_1_14*R9_12 + L_2_14*R9_22 + L_3_14*R9_32) - om9_2*(L_1_14*R9_11 + L_2_14*R9_21 + L_3_14*R9_31) + xp9_3;

	xg10_1 = L_1_15*R10_11 + L_2_15*R10_21 + L_3_15*R10_31 + x10_1;
	xg10_2 = L_1_15*R10_12 + L_2_15*R10_22 + L_3_15*R10_32 + x10_2;
	xg10_3 = L_1_15*R10_13 + L_2_15*R10_23 + L_3_15*R10_33 + x10_3;

	xgp10_1 = om10_2*(L_1_15*R10_13 + L_2_15*R10_23 + L_3_15*R10_33) - om10_3*(L_1_15*R10_12 + L_2_15*R10_22 + L_3_15*R10_32) + xp10_1;
	xgp10_2 = -om10_1*(L_1_15*R10_13 + L_2_15*R10_23 + L_3_15*R10_33) + om10_3*(L_1_15*R10_11 + L_2_15*R10_21 + L_3_15*R10_31) + xp10_2;
	xgp10_3 = om10_1*(L_1_15*R10_12 + L_2_15*R10_22 + L_3_15*R10_32) - om10_2*(L_1_15*R10_11 + L_2_15*R10_21 + L_3_15*R10_31) + xp10_3;

	xg11_1 = L_1_16*R11_11 + L_2_16*R11_21 + L_3_16*R11_31 + x11_1;
	xg11_2 = L_1_16*R11_12 + L_2_16*R11_22 + L_3_16*R11_32 + x11_2;
	xg11_3 = L_1_16*R11_13 + L_2_16*R11_23 + L_3_16*R11_33 + x11_3;

	xgp11_1 = om11_2*(L_1_16*R11_13 + L_2_16*R11_23 + L_3_16*R11_33) - om11_3*(L_1_16*R11_12 + L_2_16*R11_22 + L_3_16*R11_32) + xp11_1;
	xgp11_2 = -om11_1*(L_1_16*R11_13 + L_2_16*R11_23 + L_3_16*R11_33) + om11_3*(L_1_16*R11_11 + L_2_16*R11_21 + L_3_16*R11_31) + xp11_2;
	xgp11_3 = om11_1*(L_1_16*R11_12 + L_2_16*R11_22 + L_3_16*R11_32) - om11_2*(L_1_16*R11_11 + L_2_16*R11_21 + L_3_16*R11_31) + xp11_3;

	xg12_1 = L_1_17*R12_11 + L_2_17*R12_21 + L_3_17*R12_31 + x12_1;
	xg12_2 = L_1_17*R12_12 + L_2_17*R12_22 + L_3_17*R12_32 + x12_2;
	xg12_3 = L_1_17*R12_13 + L_2_17*R12_23 + L_3_17*R12_33 + x12_3;

	xgp12_1 = om12_2*(L_1_17*R12_13 + L_2_17*R12_23 + L_3_17*R12_33) - om12_3*(L_1_17*R12_12 + L_2_17*R12_22 + L_3_17*R12_32) + xp12_1;
	xgp12_2 = -om12_1*(L_1_17*R12_13 + L_2_17*R12_23 + L_3_17*R12_33) + om12_3*(L_1_17*R12_11 + L_2_17*R12_21 + L_3_17*R12_31) + xp12_2;
	xgp12_3 = om12_1*(L_1_17*R12_12 + L_2_17*R12_22 + L_3_17*R12_32) - om12_2*(L_1_17*R12_11 + L_2_17*R12_21 + L_3_17*R12_31) + xp12_3;

	xg13_1 = L_1_18*R13_11 + L_3_18*R13_31 + x13_1;
	xg13_2 = L_1_18*R13_12 + L_3_18*R13_32 + x13_2;
	xg13_3 = L_1_18*R13_13 + L_3_18*R13_33 + x13_3;

	xgp13_1 = om13_2*(L_1_18*R13_13 + L_3_18*R13_33) - om13_3*(L_1_18*R13_12 + L_3_18*R13_32) + xp13_1;
	xgp13_2 = -om13_1*(L_1_18*R13_13 + L_3_18*R13_33) + om13_3*(L_1_18*R13_11 + L_3_18*R13_31) + xp13_2;
	xgp13_3 = om13_1*(L_1_18*R13_12 + L_3_18*R13_32) - om13_2*(L_1_18*R13_11 + L_3_18*R13_31) + xp13_3;

	xg14_1 = L_1_19*R14_11 + L_2_19*R14_21 + L_3_19*R14_31 + x14_1;
	xg14_2 = L_1_19*R14_12 + L_2_19*R14_22 + L_3_19*R14_32 + x14_2;
	xg14_3 = L_1_19*R14_13 + L_2_19*R14_23 + L_3_19*R14_33 + x14_3;

	xgp14_1 = om14_2*(L_1_19*R14_13 + L_2_19*R14_23 + L_3_19*R14_33) - om14_3*(L_1_19*R14_12 + L_2_19*R14_22 + L_3_19*R14_32) + xp14_1;
	xgp14_2 = -om14_1*(L_1_19*R14_13 + L_2_19*R14_23 + L_3_19*R14_33) + om14_3*(L_1_19*R14_11 + L_2_19*R14_21 + L_3_19*R14_31) + xp14_2;
	xgp14_3 = om14_1*(L_1_19*R14_12 + L_2_19*R14_22 + L_3_19*R14_32) - om14_2*(L_1_19*R14_11 + L_2_19*R14_21 + L_3_19*R14_31) + xp14_3;

	xg15_1 = L_1_20*R15_11 + L_2_20*R15_21 + L_3_20*R15_31 + x15_1;
	xg15_2 = L_1_20*R15_12 + L_2_20*R15_22 + L_3_20*R15_32 + x15_2;
	xg15_3 = L_1_20*R15_13 + L_2_20*R15_23 + L_3_20*R15_33 + x15_3;

	xgp15_1 = om15_2*(L_1_20*R15_13 + L_2_20*R15_23 + L_3_20*R15_33) - om15_3*(L_1_20*R15_12 + L_2_20*R15_22 + L_3_20*R15_32) + xp15_1;
	xgp15_2 = -om15_1*(L_1_20*R15_13 + L_2_20*R15_23 + L_3_20*R15_33) + om15_3*(L_1_20*R15_11 + L_2_20*R15_21 + L_3_20*R15_31) + xp15_2;
	xgp15_3 = om15_1*(L_1_20*R15_12 + L_2_20*R15_22 + L_3_20*R15_32) - om15_2*(L_1_20*R15_11 + L_2_20*R15_21 + L_3_20*R15_31) + xp15_3;

	xg16_1 = L_1_21*R16_11 + L_2_21*R16_21 + L_3_21*R16_31 + x16_1;
	xg16_2 = L_1_21*R16_12 + L_2_21*R16_22 + L_3_21*R16_32 + x16_2;
	xg16_3 = L_1_21*R16_13 + L_2_21*R16_23 + L_3_21*R16_33 + x16_3;

	xgp16_1 = om16_2*(L_1_21*R16_13 + L_2_21*R16_23 + L_3_21*R16_33) - om16_3*(L_1_21*R16_12 + L_2_21*R16_22 + L_3_21*R16_32) + xp16_1;
	xgp16_2 = -om16_1*(L_1_21*R16_13 + L_2_21*R16_23 + L_3_21*R16_33) + om16_3*(L_1_21*R16_11 + L_2_21*R16_21 + L_3_21*R16_31) + xp16_2;
	xgp16_3 = om16_1*(L_1_21*R16_12 + L_2_21*R16_22 + L_3_21*R16_32) - om16_2*(L_1_21*R16_11 + L_2_21*R16_21 + L_3_21*R16_31) + xp16_3;

	xg17_1 = L_1_22*R17_11 + L_2_22*R17_21 + L_3_22*R17_31 + x17_1;
	xg17_2 = L_1_22*R17_12 + L_2_22*R17_22 + L_3_22*R17_32 + x17_2;
	xg17_3 = L_1_22*R17_13 + L_2_22*R17_23 + L_3_22*R17_33 + x17_3;

	xgp17_1 = om17_2*(L_1_22*R17_13 + L_2_22*R17_23 + L_3_22*R17_33) - om17_3*(L_1_22*R17_12 + L_2_22*R17_22 + L_3_22*R17_32) + xp17_1;
	xgp17_2 = -om17_1*(L_1_22*R17_13 + L_2_22*R17_23 + L_3_22*R17_33) + om17_3*(L_1_22*R17_11 + L_2_22*R17_21 + L_3_22*R17_31) + xp17_2;
	xgp17_3 = om17_1*(L_1_22*R17_12 + L_2_22*R17_22 + L_3_22*R17_32) - om17_2*(L_1_22*R17_11 + L_2_22*R17_21 + L_3_22*R17_31) + xp17_3;

	xg18_1 = L_1_23*R18_11 + L_2_23*R18_21 + L_3_23*R18_31 + x18_1;
	xg18_2 = L_1_23*R18_12 + L_2_23*R18_22 + L_3_23*R18_32 + x18_2;
	xg18_3 = L_1_23*R18_13 + L_2_23*R18_23 + L_3_23*R18_33 + x18_3;

	xgp18_1 = om18_2*(L_1_23*R18_13 + L_2_23*R18_23 + L_3_23*R18_33) - om18_3*(L_1_23*R18_12 + L_2_23*R18_22 + L_3_23*R18_32) + xp18_1;
	xgp18_2 = -om18_1*(L_1_23*R18_13 + L_2_23*R18_23 + L_3_23*R18_33) + om18_3*(L_1_23*R18_11 + L_2_23*R18_21 + L_3_23*R18_31) + xp18_2;
	xgp18_3 = om18_1*(L_1_23*R18_12 + L_2_23*R18_22 + L_3_23*R18_32) - om18_2*(L_1_23*R18_11 + L_2_23*R18_21 + L_3_23*R18_31) + xp18_3;

	xg19_1 = L_1_24*R19_11 + L_2_24*R19_21 + L_3_24*R19_31 + x19_1;
	xg19_2 = L_1_24*R19_12 + L_2_24*R19_22 + L_3_24*R19_32 + x19_2;
	xg19_3 = L_1_24*R19_13 + L_2_24*R19_23 + L_3_24*R19_33 + x19_3;

	xgp19_1 = om19_2*(L_1_24*R19_13 + L_2_24*R19_23 + L_3_24*R19_33) - om19_3*(L_1_24*R19_12 + L_2_24*R19_22 + L_3_24*R19_32) + xp19_1;
	xgp19_2 = -om19_1*(L_1_24*R19_13 + L_2_24*R19_23 + L_3_24*R19_33) + om19_3*(L_1_24*R19_11 + L_2_24*R19_21 + L_3_24*R19_31) + xp19_2;
	xgp19_3 = om19_1*(L_1_24*R19_12 + L_2_24*R19_22 + L_3_24*R19_32) - om19_2*(L_1_24*R19_11 + L_2_24*R19_21 + L_3_24*R19_31) + xp19_3;

	xg20_1 = L_1_25*R20_11 + L_2_25*R20_21 + L_3_25*R20_31 + x20_1;
	xg20_2 = L_1_25*R20_12 + L_2_25*R20_22 + L_3_25*R20_32 + x20_2;
	xg20_3 = L_1_25*R20_13 + L_2_25*R20_23 + L_3_25*R20_33 + x20_3;

	xgp20_1 = om20_2*(L_1_25*R20_13 + L_2_25*R20_23 + L_3_25*R20_33) - om20_3*(L_1_25*R20_12 + L_2_25*R20_22 + L_3_25*R20_32) + xp20_1;
	xgp20_2 = -om20_1*(L_1_25*R20_13 + L_2_25*R20_23 + L_3_25*R20_33) + om20_3*(L_1_25*R20_11 + L_2_25*R20_21 + L_3_25*R20_31) + xp20_2;
	xgp20_3 = om20_1*(L_1_25*R20_12 + L_2_25*R20_22 + L_3_25*R20_32) - om20_2*(L_1_25*R20_11 + L_2_25*R20_21 + L_3_25*R20_31) + xp20_3;

	xg21_1 = L_1_26*R21_11 + L_2_26*R21_21 + L_3_26*R21_31 + x21_1;
	xg21_2 = L_1_26*R21_12 + L_2_26*R21_22 + L_3_26*R21_32 + x21_2;
	xg21_3 = L_1_26*R21_13 + L_2_26*R21_23 + L_3_26*R21_33 + x21_3;

	xgp21_1 = om21_2*(L_1_26*R21_13 + L_2_26*R21_23 + L_3_26*R21_33) - om21_3*(L_1_26*R21_12 + L_2_26*R21_22 + L_3_26*R21_32) + xp21_1;
	xgp21_2 = -om21_1*(L_1_26*R21_13 + L_2_26*R21_23 + L_3_26*R21_33) + om21_3*(L_1_26*R21_11 + L_2_26*R21_21 + L_3_26*R21_31) + xp21_2;
	xgp21_3 = om21_1*(L_1_26*R21_12 + L_2_26*R21_22 + L_3_26*R21_32) - om21_2*(L_1_26*R21_11 + L_2_26*R21_21 + L_3_26*R21_31) + xp21_3;

	xg22_1 = L_1_27*R22_11 + L_2_27*R22_21 + L_3_27*R22_31 + x22_1;
	xg22_2 = L_1_27*R22_12 + L_2_27*R22_22 + L_3_27*R22_32 + x22_2;
	xg22_3 = L_1_27*R22_13 + L_2_27*R22_23 + L_3_27*R22_33 + x22_3;

	xgp22_1 = om22_2*(L_1_27*R22_13 + L_2_27*R22_23 + L_3_27*R22_33) - om22_3*(L_1_27*R22_12 + L_2_27*R22_22 + L_3_27*R22_32) + xp22_1;
	xgp22_2 = -om22_1*(L_1_27*R22_13 + L_2_27*R22_23 + L_3_27*R22_33) + om22_3*(L_1_27*R22_11 + L_2_27*R22_21 + L_3_27*R22_31) + xp22_2;
	xgp22_3 = om22_1*(L_1_27*R22_12 + L_2_27*R22_22 + L_3_27*R22_32) - om22_2*(L_1_27*R22_11 + L_2_27*R22_21 + L_3_27*R22_31) + xp22_3;

	xg23_1 = L_1_28*R23_11 + L_2_28*R23_21 + L_3_28*R23_31 + x23_1;
	xg23_2 = L_1_28*R23_12 + L_2_28*R23_22 + L_3_28*R23_32 + x23_2;
	xg23_3 = L_1_28*R23_13 + L_2_28*R23_23 + L_3_28*R23_33 + x23_3;

	xgp23_1 = om23_2*(L_1_28*R23_13 + L_2_28*R23_23 + L_3_28*R23_33) - om23_3*(L_1_28*R23_12 + L_2_28*R23_22 + L_3_28*R23_32) + xp23_1;
	xgp23_2 = -om23_1*(L_1_28*R23_13 + L_2_28*R23_23 + L_3_28*R23_33) + om23_3*(L_1_28*R23_11 + L_2_28*R23_21 + L_3_28*R23_31) + xp23_2;
	xgp23_3 = om23_1*(L_1_28*R23_12 + L_2_28*R23_22 + L_3_28*R23_32) - om23_2*(L_1_28*R23_11 + L_2_28*R23_21 + L_3_28*R23_31) + xp23_3;

	xg24_1 = L_1_29*R24_11 + L_2_29*R24_21 + L_3_29*R24_31 + x24_1;
	xg24_2 = L_1_29*R24_12 + L_2_29*R24_22 + L_3_29*R24_32 + x24_2;
	xg24_3 = L_1_29*R24_13 + L_2_29*R24_23 + L_3_29*R24_33 + x24_3;

	xgp24_1 = om24_2*(L_1_29*R24_13 + L_2_29*R24_23 + L_3_29*R24_33) - om24_3*(L_1_29*R24_12 + L_2_29*R24_22 + L_3_29*R24_32) + xp24_1;
	xgp24_2 = -om24_1*(L_1_29*R24_13 + L_2_29*R24_23 + L_3_29*R24_33) + om24_3*(L_1_29*R24_11 + L_2_29*R24_21 + L_3_29*R24_31) + xp24_2;
	xgp24_3 = om24_1*(L_1_29*R24_12 + L_2_29*R24_22 + L_3_29*R24_32) - om24_2*(L_1_29*R24_11 + L_2_29*R24_21 + L_3_29*R24_31) + xp24_3;


	// jacobian com absolute positions
	if (flag_jacob)
	{
		xg2_1_d2 = L_1_7*R2_11_d2 + L_3_7*R2_31_d2;
		xg2_2_d2 = L_1_7*R2_12_d2 + L_3_7*R2_32_d2;
		xg2_3_d2 = L_1_7*R2_13_d2 + L_3_7*R2_33_d2;

		xg3_1_d2 = L_1_8*R3_11_d2 + L_2_8*R3_21_d2 + L_3_8*R3_31_d2;
		xg3_2_d2 = L_1_8*R3_12_d2 + L_2_8*R3_22_d2 + L_3_8*R3_32_d2;
		xg3_3_d2 = L_1_8*R3_13_d2 + L_2_8*R3_23_d2 + L_3_8*R3_33_d2;

		xg3_1_d3 = L_2_8*R3_21_d3 + L_3_8*R3_31_d3;
		xg3_2_d3 = L_2_8*R3_22_d3 + L_3_8*R3_32_d3;
		xg3_3_d3 = L_2_8*R3_23_d3 + L_3_8*R3_33_d3;

		xg4_1_d2 = L_1_9*R4_11_d2 + L_2_9*R4_21_d2 + L_3_9*R4_31_d2 + x4_1_d2;
		xg4_2_d2 = L_1_9*R4_12_d2 + L_2_9*R4_22_d2 + L_3_9*R4_32_d2 + x4_2_d2;
		xg4_3_d2 = L_1_9*R4_13_d2 + L_2_9*R4_23_d2 + L_3_9*R4_33_d2 + x4_3_d2;

		xg4_1_d3 = L_1_9*R4_11_d3 + L_2_9*R4_21_d3 + L_3_9*R4_31_d3 + x4_1_d3;
		xg4_2_d3 = L_1_9*R4_12_d3 + L_2_9*R4_22_d3 + L_3_9*R4_32_d3 + x4_2_d3;
		xg4_3_d3 = L_1_9*R4_13_d3 + L_2_9*R4_23_d3 + L_3_9*R4_33_d3 + x4_3_d3;

		xg4_1_d4 = L_1_9*R4_11_d4 + L_2_9*R4_21_d4;
		xg4_2_d4 = L_1_9*R4_12_d4 + L_2_9*R4_22_d4;
		xg4_3_d4 = L_1_9*R4_13_d4 + L_2_9*R4_23_d4;

		xg5_1_d2 = L_1_10*R5_11_d2 + L_2_10*R5_21_d2 + L_3_10*R5_31_d2 + x5_1_d2;
		xg5_2_d2 = L_1_10*R5_12_d2 + L_2_10*R5_22_d2 + L_3_10*R5_32_d2 + x5_2_d2;
		xg5_3_d2 = L_1_10*R5_13_d2 + L_2_10*R5_23_d2 + L_3_10*R5_33_d2 + x5_3_d2;

		xg5_1_d3 = L_1_10*R5_11_d3 + L_2_10*R5_21_d3 + L_3_10*R5_31_d3 + x5_1_d3;
		xg5_2_d3 = L_1_10*R5_12_d3 + L_2_10*R5_22_d3 + L_3_10*R5_32_d3 + x5_2_d3;
		xg5_3_d3 = L_1_10*R5_13_d3 + L_2_10*R5_23_d3 + L_3_10*R5_33_d3 + x5_3_d3;

		xg5_1_d4 = L_1_10*R5_11_d4 + L_2_10*R5_21_d4 + L_3_10*R5_31_d4;
		xg5_2_d4 = L_1_10*R5_12_d4 + L_2_10*R5_22_d4 + L_3_10*R5_32_d4;
		xg5_3_d4 = L_1_10*R5_13_d4 + L_2_10*R5_23_d4 + L_3_10*R5_33_d4;

		xg5_1_d5 = L_1_10*R5_11_d5 + L_3_10*R5_31_d5;
		xg5_2_d5 = L_1_10*R5_12_d5 + L_3_10*R5_32_d5;
		xg5_3_d5 = L_1_10*R5_13_d5 + L_3_10*R5_33_d5;

		xg6_1_d2 = L_1_11*R6_11_d2 + L_2_11*R6_21_d2 + L_3_11*R6_31_d2 + x6_1_d2;
		xg6_2_d2 = L_1_11*R6_12_d2 + L_2_11*R6_22_d2 + L_3_11*R6_32_d2 + x6_2_d2;
		xg6_3_d2 = L_1_11*R6_13_d2 + L_2_11*R6_23_d2 + L_3_11*R6_33_d2 + x6_3_d2;

		xg6_1_d3 = L_1_11*R6_11_d3 + L_2_11*R6_21_d3 + L_3_11*R6_31_d3 + x6_1_d3;
		xg6_2_d3 = L_1_11*R6_12_d3 + L_2_11*R6_22_d3 + L_3_11*R6_32_d3 + x6_2_d3;
		xg6_3_d3 = L_1_11*R6_13_d3 + L_2_11*R6_23_d3 + L_3_11*R6_33_d3 + x6_3_d3;

		xg6_1_d4 = L_1_11*R6_11_d4 + L_2_11*R6_21_d4 + L_3_11*R6_31_d4 + x6_1_d4;
		xg6_2_d4 = L_1_11*R6_12_d4 + L_2_11*R6_22_d4 + L_3_11*R6_32_d4 + x6_2_d4;
		xg6_3_d4 = L_1_11*R6_13_d4 + L_2_11*R6_23_d4 + L_3_11*R6_33_d4 + x6_3_d4;

		xg6_1_d5 = L_1_11*R6_11_d5 + L_2_11*R6_21_d5 + L_3_11*R6_31_d5 + x6_1_d5;
		xg6_2_d5 = L_1_11*R6_12_d5 + L_2_11*R6_22_d5 + L_3_11*R6_32_d5 + x6_2_d5;
		xg6_3_d5 = L_1_11*R6_13_d5 + L_2_11*R6_23_d5 + L_3_11*R6_33_d5 + x6_3_d5;

		xg6_1_d6 = L_2_11*R6_21_d6 + L_3_11*R6_31_d6;
		xg6_2_d6 = L_2_11*R6_22_d6 + L_3_11*R6_32_d6;
		xg6_3_d6 = L_2_11*R6_23_d6 + L_3_11*R6_33_d6;

		xg7_1_d2 = L_1_12*R7_11_d2 + L_3_12*R7_31_d2 + x7_1_d2;
		xg7_2_d2 = L_1_12*R7_12_d2 + L_3_12*R7_32_d2 + x7_2_d2;
		xg7_3_d2 = L_1_12*R7_13_d2 + L_3_12*R7_33_d2 + x7_3_d2;

		xg7_1_d3 = L_1_12*R7_11_d3 + L_3_12*R7_31_d3 + x7_1_d3;
		xg7_2_d3 = L_1_12*R7_12_d3 + L_3_12*R7_32_d3 + x7_2_d3;
		xg7_3_d3 = L_1_12*R7_13_d3 + L_3_12*R7_33_d3 + x7_3_d3;

		xg7_1_d4 = L_1_12*R7_11_d4 + L_3_12*R7_31_d4 + x7_1_d4;
		xg7_2_d4 = L_1_12*R7_12_d4 + L_3_12*R7_32_d4 + x7_2_d4;
		xg7_3_d4 = L_1_12*R7_13_d4 + L_3_12*R7_33_d4 + x7_3_d4;

		xg7_1_d5 = L_1_12*R7_11_d5 + L_3_12*R7_31_d5 + x7_1_d5;
		xg7_2_d5 = L_1_12*R7_12_d5 + L_3_12*R7_32_d5 + x7_2_d5;
		xg7_3_d5 = L_1_12*R7_13_d5 + L_3_12*R7_33_d5 + x7_3_d5;

		xg7_1_d6 = L_1_12*R7_11_d6 + L_3_12*R7_31_d6;
		xg7_2_d6 = L_1_12*R7_12_d6 + L_3_12*R7_32_d6;
		xg7_3_d6 = L_1_12*R7_13_d6 + L_3_12*R7_33_d6;

		xg7_1_d7 = L_1_12*R7_11_d7 + L_3_12*R7_31_d7;
		xg7_2_d7 = L_1_12*R7_12_d7 + L_3_12*R7_32_d7;
		xg7_3_d7 = L_1_12*R7_13_d7 + L_3_12*R7_33_d7;

		xg8_1_d8 = L_1_13*R8_11_d8 + L_3_13*R8_31_d8;
		xg8_2_d8 = L_1_13*R8_12_d8 + L_3_13*R8_32_d8;
		xg8_3_d8 = L_1_13*R8_13_d8 + L_3_13*R8_33_d8;

		xg9_1_d8 = L_1_14*R9_11_d8 + L_2_14*R9_21_d8 + L_3_14*R9_31_d8;
		xg9_2_d8 = L_1_14*R9_12_d8 + L_2_14*R9_22_d8 + L_3_14*R9_32_d8;
		xg9_3_d8 = L_1_14*R9_13_d8 + L_2_14*R9_23_d8 + L_3_14*R9_33_d8;

		xg9_1_d9 = L_2_14*R9_21_d9 + L_3_14*R9_31_d9;
		xg9_2_d9 = L_2_14*R9_22_d9 + L_3_14*R9_32_d9;
		xg9_3_d9 = L_2_14*R9_23_d9 + L_3_14*R9_33_d9;

		xg10_1_d8 = L_1_15*R10_11_d8 + L_2_15*R10_21_d8 + L_3_15*R10_31_d8 + x10_1_d8;
		xg10_2_d8 = L_1_15*R10_12_d8 + L_2_15*R10_22_d8 + L_3_15*R10_32_d8 + x10_2_d8;
		xg10_3_d8 = L_1_15*R10_13_d8 + L_2_15*R10_23_d8 + L_3_15*R10_33_d8 + x10_3_d8;

		xg10_1_d9 = L_1_15*R10_11_d9 + L_2_15*R10_21_d9 + L_3_15*R10_31_d9 + x10_1_d9;
		xg10_2_d9 = L_1_15*R10_12_d9 + L_2_15*R10_22_d9 + L_3_15*R10_32_d9 + x10_2_d9;
		xg10_3_d9 = L_1_15*R10_13_d9 + L_2_15*R10_23_d9 + L_3_15*R10_33_d9 + x10_3_d9;

		xg10_1_d10 = L_1_15*R10_11_d10 + L_2_15*R10_21_d10;
		xg10_2_d10 = L_1_15*R10_12_d10 + L_2_15*R10_22_d10;
		xg10_3_d10 = L_1_15*R10_13_d10 + L_2_15*R10_23_d10;

		xg11_1_d8 = L_1_16*R11_11_d8 + L_2_16*R11_21_d8 + L_3_16*R11_31_d8 + x11_1_d8;
		xg11_2_d8 = L_1_16*R11_12_d8 + L_2_16*R11_22_d8 + L_3_16*R11_32_d8 + x11_2_d8;
		xg11_3_d8 = L_1_16*R11_13_d8 + L_2_16*R11_23_d8 + L_3_16*R11_33_d8 + x11_3_d8;

		xg11_1_d9 = L_1_16*R11_11_d9 + L_2_16*R11_21_d9 + L_3_16*R11_31_d9 + x11_1_d9;
		xg11_2_d9 = L_1_16*R11_12_d9 + L_2_16*R11_22_d9 + L_3_16*R11_32_d9 + x11_2_d9;
		xg11_3_d9 = L_1_16*R11_13_d9 + L_2_16*R11_23_d9 + L_3_16*R11_33_d9 + x11_3_d9;

		xg11_1_d10 = L_1_16*R11_11_d10 + L_2_16*R11_21_d10 + L_3_16*R11_31_d10;
		xg11_2_d10 = L_1_16*R11_12_d10 + L_2_16*R11_22_d10 + L_3_16*R11_32_d10;
		xg11_3_d10 = L_1_16*R11_13_d10 + L_2_16*R11_23_d10 + L_3_16*R11_33_d10;

		xg11_1_d11 = L_1_16*R11_11_d11 + L_3_16*R11_31_d11;
		xg11_2_d11 = L_1_16*R11_12_d11 + L_3_16*R11_32_d11;
		xg11_3_d11 = L_1_16*R11_13_d11 + L_3_16*R11_33_d11;

		xg12_1_d8 = L_1_17*R12_11_d8 + L_2_17*R12_21_d8 + L_3_17*R12_31_d8 + x12_1_d8;
		xg12_2_d8 = L_1_17*R12_12_d8 + L_2_17*R12_22_d8 + L_3_17*R12_32_d8 + x12_2_d8;
		xg12_3_d8 = L_1_17*R12_13_d8 + L_2_17*R12_23_d8 + L_3_17*R12_33_d8 + x12_3_d8;

		xg12_1_d9 = L_1_17*R12_11_d9 + L_2_17*R12_21_d9 + L_3_17*R12_31_d9 + x12_1_d9;
		xg12_2_d9 = L_1_17*R12_12_d9 + L_2_17*R12_22_d9 + L_3_17*R12_32_d9 + x12_2_d9;
		xg12_3_d9 = L_1_17*R12_13_d9 + L_2_17*R12_23_d9 + L_3_17*R12_33_d9 + x12_3_d9;

		xg12_1_d10 = L_1_17*R12_11_d10 + L_2_17*R12_21_d10 + L_3_17*R12_31_d10 + x12_1_d10;
		xg12_2_d10 = L_1_17*R12_12_d10 + L_2_17*R12_22_d10 + L_3_17*R12_32_d10 + x12_2_d10;
		xg12_3_d10 = L_1_17*R12_13_d10 + L_2_17*R12_23_d10 + L_3_17*R12_33_d10 + x12_3_d10;

		xg12_1_d11 = L_1_17*R12_11_d11 + L_2_17*R12_21_d11 + L_3_17*R12_31_d11 + x12_1_d11;
		xg12_2_d11 = L_1_17*R12_12_d11 + L_2_17*R12_22_d11 + L_3_17*R12_32_d11 + x12_2_d11;
		xg12_3_d11 = L_1_17*R12_13_d11 + L_2_17*R12_23_d11 + L_3_17*R12_33_d11 + x12_3_d11;

		xg12_1_d12 = L_2_17*R12_21_d12 + L_3_17*R12_31_d12;
		xg12_2_d12 = L_2_17*R12_22_d12 + L_3_17*R12_32_d12;
		xg12_3_d12 = L_2_17*R12_23_d12 + L_3_17*R12_33_d12;

		xg13_1_d8 = L_1_18*R13_11_d8 + L_3_18*R13_31_d8 + x13_1_d8;
		xg13_2_d8 = L_1_18*R13_12_d8 + L_3_18*R13_32_d8 + x13_2_d8;
		xg13_3_d8 = L_1_18*R13_13_d8 + L_3_18*R13_33_d8 + x13_3_d8;

		xg13_1_d9 = L_1_18*R13_11_d9 + L_3_18*R13_31_d9 + x13_1_d9;
		xg13_2_d9 = L_1_18*R13_12_d9 + L_3_18*R13_32_d9 + x13_2_d9;
		xg13_3_d9 = L_1_18*R13_13_d9 + L_3_18*R13_33_d9 + x13_3_d9;

		xg13_1_d10 = L_1_18*R13_11_d10 + L_3_18*R13_31_d10 + x13_1_d10;
		xg13_2_d10 = L_1_18*R13_12_d10 + L_3_18*R13_32_d10 + x13_2_d10;
		xg13_3_d10 = L_1_18*R13_13_d10 + L_3_18*R13_33_d10 + x13_3_d10;

		xg13_1_d11 = L_1_18*R13_11_d11 + L_3_18*R13_31_d11 + x13_1_d11;
		xg13_2_d11 = L_1_18*R13_12_d11 + L_3_18*R13_32_d11 + x13_2_d11;
		xg13_3_d11 = L_1_18*R13_13_d11 + L_3_18*R13_33_d11 + x13_3_d11;

		xg13_1_d12 = L_1_18*R13_11_d12 + L_3_18*R13_31_d12;
		xg13_2_d12 = L_1_18*R13_12_d12 + L_3_18*R13_32_d12;
		xg13_3_d12 = L_1_18*R13_13_d12 + L_3_18*R13_33_d12;

		xg13_1_d13 = L_1_18*R13_11_d13 + L_3_18*R13_31_d13;
		xg13_2_d13 = L_1_18*R13_12_d13 + L_3_18*R13_32_d13;
		xg13_3_d13 = L_1_18*R13_13_d13 + L_3_18*R13_33_d13;

		xg14_1_d14 = L_2_19*R14_21_d14 + L_3_19*R14_31_d14;
		xg14_2_d14 = L_2_19*R14_22_d14 + L_3_19*R14_32_d14;
		xg14_3_d14 = L_2_19*R14_23_d14 + L_3_19*R14_33_d14;

		xg15_1_d14 = L_1_20*R15_11_d14 + L_2_20*R15_21_d14 + L_3_20*R15_31_d14;
		xg15_2_d14 = L_1_20*R15_12_d14 + L_2_20*R15_22_d14 + L_3_20*R15_32_d14;
		xg15_3_d14 = L_1_20*R15_13_d14 + L_2_20*R15_23_d14 + L_3_20*R15_33_d14;

		xg15_1_d15 = L_1_20*R15_11_d15 + L_3_20*R15_31_d15;
		xg15_2_d15 = L_1_20*R15_12_d15 + L_3_20*R15_32_d15;
		xg15_3_d15 = L_1_20*R15_13_d15 + L_3_20*R15_33_d15;

		xg16_1_d14 = L_1_21*R16_11_d14 + L_2_21*R16_21_d14 + L_3_21*R16_31_d14 + x16_1_d14;
		xg16_2_d14 = L_1_21*R16_12_d14 + L_2_21*R16_22_d14 + L_3_21*R16_32_d14 + x16_2_d14;
		xg16_3_d14 = L_1_21*R16_13_d14 + L_2_21*R16_23_d14 + L_3_21*R16_33_d14 + x16_3_d14;

		xg16_1_d15 = L_1_21*R16_11_d15 + L_2_21*R16_21_d15 + L_3_21*R16_31_d15 + x16_1_d15;
		xg16_2_d15 = L_1_21*R16_12_d15 + L_2_21*R16_22_d15 + L_3_21*R16_32_d15 + x16_2_d15;
		xg16_3_d15 = L_1_21*R16_13_d15 + L_2_21*R16_23_d15 + L_3_21*R16_33_d15 + x16_3_d15;

		xg16_1_d16 = L_1_21*R16_11_d16 + L_2_21*R16_21_d16;
		xg16_2_d16 = L_1_21*R16_12_d16 + L_2_21*R16_22_d16;
		xg16_3_d16 = L_1_21*R16_13_d16 + L_2_21*R16_23_d16;

		xg17_1_d14 = L_1_22*R17_11_d14 + L_2_22*R17_21_d14 + L_3_22*R17_31_d14 + x17_1_d14;
		xg17_2_d14 = L_1_22*R17_12_d14 + L_2_22*R17_22_d14 + L_3_22*R17_32_d14 + x17_2_d14;
		xg17_3_d14 = L_1_22*R17_13_d14 + L_2_22*R17_23_d14 + L_3_22*R17_33_d14 + x17_3_d14;

		xg17_1_d15 = L_1_22*R17_11_d15 + L_2_22*R17_21_d15 + L_3_22*R17_31_d15 + x17_1_d15;
		xg17_2_d15 = L_1_22*R17_12_d15 + L_2_22*R17_22_d15 + L_3_22*R17_32_d15 + x17_2_d15;
		xg17_3_d15 = L_1_22*R17_13_d15 + L_2_22*R17_23_d15 + L_3_22*R17_33_d15 + x17_3_d15;

		xg17_1_d16 = L_1_22*R17_11_d16 + L_2_22*R17_21_d16 + L_3_22*R17_31_d16 + x17_1_d16;
		xg17_2_d16 = L_1_22*R17_12_d16 + L_2_22*R17_22_d16 + L_3_22*R17_32_d16 + x17_2_d16;
		xg17_3_d16 = L_1_22*R17_13_d16 + L_2_22*R17_23_d16 + L_3_22*R17_33_d16 + x17_3_d16;

		xg17_1_d17 = L_1_22*R17_11_d17 + L_3_22*R17_31_d17;
		xg17_2_d17 = L_1_22*R17_12_d17 + L_3_22*R17_32_d17;
		xg17_3_d17 = L_1_22*R17_13_d17 + L_3_22*R17_33_d17;

		xg18_1_d14 = L_1_23*R18_11_d14 + L_2_23*R18_21_d14 + L_3_23*R18_31_d14 + x18_1_d14;
		xg18_2_d14 = L_1_23*R18_12_d14 + L_2_23*R18_22_d14 + L_3_23*R18_32_d14 + x18_2_d14;
		xg18_3_d14 = L_1_23*R18_13_d14 + L_2_23*R18_23_d14 + L_3_23*R18_33_d14 + x18_3_d14;

		xg18_1_d15 = L_1_23*R18_11_d15 + L_2_23*R18_21_d15 + L_3_23*R18_31_d15 + x18_1_d15;
		xg18_2_d15 = L_1_23*R18_12_d15 + L_2_23*R18_22_d15 + L_3_23*R18_32_d15 + x18_2_d15;
		xg18_3_d15 = L_1_23*R18_13_d15 + L_2_23*R18_23_d15 + L_3_23*R18_33_d15 + x18_3_d15;

		xg18_1_d16 = L_1_23*R18_11_d16 + L_2_23*R18_21_d16 + L_3_23*R18_31_d16 + x18_1_d16;
		xg18_2_d16 = L_1_23*R18_12_d16 + L_2_23*R18_22_d16 + L_3_23*R18_32_d16 + x18_2_d16;
		xg18_3_d16 = L_1_23*R18_13_d16 + L_2_23*R18_23_d16 + L_3_23*R18_33_d16 + x18_3_d16;

		xg18_1_d17 = L_1_23*R18_11_d17 + L_2_23*R18_21_d17 + L_3_23*R18_31_d17;
		xg18_2_d17 = L_1_23*R18_12_d17 + L_2_23*R18_22_d17 + L_3_23*R18_32_d17;
		xg18_3_d17 = L_1_23*R18_13_d17 + L_2_23*R18_23_d17 + L_3_23*R18_33_d17;

		xg18_1_d18 = L_2_23*R18_21_d18 + L_3_23*R18_31_d18;
		xg18_2_d18 = L_2_23*R18_22_d18 + L_3_23*R18_32_d18;
		xg18_3_d18 = L_2_23*R18_23_d18 + L_3_23*R18_33_d18;

		xg19_1_d14 = L_1_24*R19_11_d14 + L_2_24*R19_21_d14 + L_3_24*R19_31_d14 + x19_1_d14;
		xg19_2_d14 = L_1_24*R19_12_d14 + L_2_24*R19_22_d14 + L_3_24*R19_32_d14 + x19_2_d14;
		xg19_3_d14 = L_1_24*R19_13_d14 + L_2_24*R19_23_d14 + L_3_24*R19_33_d14 + x19_3_d14;

		xg19_1_d15 = L_1_24*R19_11_d15 + L_2_24*R19_21_d15 + L_3_24*R19_31_d15 + x19_1_d15;
		xg19_2_d15 = L_1_24*R19_12_d15 + L_2_24*R19_22_d15 + L_3_24*R19_32_d15 + x19_2_d15;
		xg19_3_d15 = L_1_24*R19_13_d15 + L_2_24*R19_23_d15 + L_3_24*R19_33_d15 + x19_3_d15;

		xg19_1_d16 = L_1_24*R19_11_d16 + L_2_24*R19_21_d16 + L_3_24*R19_31_d16 + x19_1_d16;
		xg19_2_d16 = L_1_24*R19_12_d16 + L_2_24*R19_22_d16 + L_3_24*R19_32_d16 + x19_2_d16;
		xg19_3_d16 = L_1_24*R19_13_d16 + L_2_24*R19_23_d16 + L_3_24*R19_33_d16 + x19_3_d16;

		xg19_1_d17 = L_1_24*R19_11_d17 + L_2_24*R19_21_d17 + L_3_24*R19_31_d17 + x19_1_d17;
		xg19_2_d17 = L_1_24*R19_12_d17 + L_2_24*R19_22_d17 + L_3_24*R19_32_d17 + x19_2_d17;
		xg19_3_d17 = L_1_24*R19_13_d17 + L_2_24*R19_23_d17 + L_3_24*R19_33_d17 + x19_3_d17;

		xg19_1_d18 = L_1_24*R19_11_d18 + L_2_24*R19_21_d18 + L_3_24*R19_31_d18 + x19_1_d18;
		xg19_2_d18 = L_1_24*R19_12_d18 + L_2_24*R19_22_d18 + L_3_24*R19_32_d18 + x19_2_d18;
		xg19_3_d18 = L_1_24*R19_13_d18 + L_2_24*R19_23_d18 + L_3_24*R19_33_d18 + x19_3_d18;

		xg19_1_d19 = L_1_24*R19_11_d19 + L_2_24*R19_21_d19;
		xg19_2_d19 = L_1_24*R19_12_d19 + L_2_24*R19_22_d19;
		xg19_3_d19 = L_1_24*R19_13_d19 + L_2_24*R19_23_d19;

		xg20_1_d14 = L_1_25*R20_11_d14 + L_2_25*R20_21_d14 + L_3_25*R20_31_d14 + x20_1_d14;
		xg20_2_d14 = L_1_25*R20_12_d14 + L_2_25*R20_22_d14 + L_3_25*R20_32_d14 + x20_2_d14;
		xg20_3_d14 = L_1_25*R20_13_d14 + L_2_25*R20_23_d14 + L_3_25*R20_33_d14 + x20_3_d14;

		xg20_1_d15 = L_1_25*R20_11_d15 + L_2_25*R20_21_d15 + L_3_25*R20_31_d15 + x20_1_d15;
		xg20_2_d15 = L_1_25*R20_12_d15 + L_2_25*R20_22_d15 + L_3_25*R20_32_d15 + x20_2_d15;
		xg20_3_d15 = L_1_25*R20_13_d15 + L_2_25*R20_23_d15 + L_3_25*R20_33_d15 + x20_3_d15;

		xg20_1_d16 = L_1_25*R20_11_d16 + L_2_25*R20_21_d16 + L_3_25*R20_31_d16 + x20_1_d16;
		xg20_2_d16 = L_1_25*R20_12_d16 + L_2_25*R20_22_d16 + L_3_25*R20_32_d16 + x20_2_d16;
		xg20_3_d16 = L_1_25*R20_13_d16 + L_2_25*R20_23_d16 + L_3_25*R20_33_d16 + x20_3_d16;

		xg20_1_d17 = L_1_25*R20_11_d17 + L_2_25*R20_21_d17 + L_3_25*R20_31_d17 + x20_1_d17;
		xg20_2_d17 = L_1_25*R20_12_d17 + L_2_25*R20_22_d17 + L_3_25*R20_32_d17 + x20_2_d17;
		xg20_3_d17 = L_1_25*R20_13_d17 + L_2_25*R20_23_d17 + L_3_25*R20_33_d17 + x20_3_d17;

		xg20_1_d18 = L_1_25*R20_11_d18 + L_2_25*R20_21_d18 + L_3_25*R20_31_d18 + x20_1_d18;
		xg20_2_d18 = L_1_25*R20_12_d18 + L_2_25*R20_22_d18 + L_3_25*R20_32_d18 + x20_2_d18;
		xg20_3_d18 = L_1_25*R20_13_d18 + L_2_25*R20_23_d18 + L_3_25*R20_33_d18 + x20_3_d18;

		xg20_1_d19 = L_1_25*R20_11_d19 + L_2_25*R20_21_d19 + L_3_25*R20_31_d19;
		xg20_2_d19 = L_1_25*R20_12_d19 + L_2_25*R20_22_d19 + L_3_25*R20_32_d19;
		xg20_3_d19 = L_1_25*R20_13_d19 + L_2_25*R20_23_d19 + L_3_25*R20_33_d19;

		xg20_1_d20 = L_1_25*R20_11_d20 + L_3_25*R20_31_d20;
		xg20_2_d20 = L_1_25*R20_12_d20 + L_3_25*R20_32_d20;
		xg20_3_d20 = L_1_25*R20_13_d20 + L_3_25*R20_33_d20;

		xg21_1_d14 = L_1_26*R21_11_d14 + L_2_26*R21_21_d14 + L_3_26*R21_31_d14 + x21_1_d14;
		xg21_2_d14 = L_1_26*R21_12_d14 + L_2_26*R21_22_d14 + L_3_26*R21_32_d14 + x21_2_d14;
		xg21_3_d14 = L_1_26*R21_13_d14 + L_2_26*R21_23_d14 + L_3_26*R21_33_d14 + x21_3_d14;

		xg21_1_d15 = L_1_26*R21_11_d15 + L_2_26*R21_21_d15 + L_3_26*R21_31_d15 + x21_1_d15;
		xg21_2_d15 = L_1_26*R21_12_d15 + L_2_26*R21_22_d15 + L_3_26*R21_32_d15 + x21_2_d15;
		xg21_3_d15 = L_1_26*R21_13_d15 + L_2_26*R21_23_d15 + L_3_26*R21_33_d15 + x21_3_d15;

		xg21_1_d16 = L_1_26*R21_11_d16 + L_2_26*R21_21_d16 + L_3_26*R21_31_d16 + x21_1_d16;
		xg21_2_d16 = L_1_26*R21_12_d16 + L_2_26*R21_22_d16 + L_3_26*R21_32_d16 + x21_2_d16;
		xg21_3_d16 = L_1_26*R21_13_d16 + L_2_26*R21_23_d16 + L_3_26*R21_33_d16 + x21_3_d16;

		xg21_1_d21 = L_1_26*R21_11_d21 + L_3_26*R21_31_d21;
		xg21_2_d21 = L_1_26*R21_12_d21 + L_3_26*R21_32_d21;
		xg21_3_d21 = L_1_26*R21_13_d21 + L_3_26*R21_33_d21;

		xg22_1_d14 = L_1_27*R22_11_d14 + L_2_27*R22_21_d14 + L_3_27*R22_31_d14 + x22_1_d14;
		xg22_2_d14 = L_1_27*R22_12_d14 + L_2_27*R22_22_d14 + L_3_27*R22_32_d14 + x22_2_d14;
		xg22_3_d14 = L_1_27*R22_13_d14 + L_2_27*R22_23_d14 + L_3_27*R22_33_d14 + x22_3_d14;

		xg22_1_d15 = L_1_27*R22_11_d15 + L_2_27*R22_21_d15 + L_3_27*R22_31_d15 + x22_1_d15;
		xg22_2_d15 = L_1_27*R22_12_d15 + L_2_27*R22_22_d15 + L_3_27*R22_32_d15 + x22_2_d15;
		xg22_3_d15 = L_1_27*R22_13_d15 + L_2_27*R22_23_d15 + L_3_27*R22_33_d15 + x22_3_d15;

		xg22_1_d16 = L_1_27*R22_11_d16 + L_2_27*R22_21_d16 + L_3_27*R22_31_d16 + x22_1_d16;
		xg22_2_d16 = L_1_27*R22_12_d16 + L_2_27*R22_22_d16 + L_3_27*R22_32_d16 + x22_2_d16;
		xg22_3_d16 = L_1_27*R22_13_d16 + L_2_27*R22_23_d16 + L_3_27*R22_33_d16 + x22_3_d16;

		xg22_1_d21 = L_1_27*R22_11_d21 + L_2_27*R22_21_d21 + L_3_27*R22_31_d21;
		xg22_2_d21 = L_1_27*R22_12_d21 + L_2_27*R22_22_d21 + L_3_27*R22_32_d21;
		xg22_3_d21 = L_1_27*R22_13_d21 + L_2_27*R22_23_d21 + L_3_27*R22_33_d21;

		xg22_1_d22 = L_2_27*R22_21_d22 + L_3_27*R22_31_d22;
		xg22_2_d22 = L_2_27*R22_22_d22 + L_3_27*R22_32_d22;
		xg22_3_d22 = L_2_27*R22_23_d22 + L_3_27*R22_33_d22;

		xg23_1_d14 = L_1_28*R23_11_d14 + L_2_28*R23_21_d14 + L_3_28*R23_31_d14 + x23_1_d14;
		xg23_2_d14 = L_1_28*R23_12_d14 + L_2_28*R23_22_d14 + L_3_28*R23_32_d14 + x23_2_d14;
		xg23_3_d14 = L_1_28*R23_13_d14 + L_2_28*R23_23_d14 + L_3_28*R23_33_d14 + x23_3_d14;

		xg23_1_d15 = L_1_28*R23_11_d15 + L_2_28*R23_21_d15 + L_3_28*R23_31_d15 + x23_1_d15;
		xg23_2_d15 = L_1_28*R23_12_d15 + L_2_28*R23_22_d15 + L_3_28*R23_32_d15 + x23_2_d15;
		xg23_3_d15 = L_1_28*R23_13_d15 + L_2_28*R23_23_d15 + L_3_28*R23_33_d15 + x23_3_d15;

		xg23_1_d16 = L_1_28*R23_11_d16 + L_2_28*R23_21_d16 + L_3_28*R23_31_d16 + x23_1_d16;
		xg23_2_d16 = L_1_28*R23_12_d16 + L_2_28*R23_22_d16 + L_3_28*R23_32_d16 + x23_2_d16;
		xg23_3_d16 = L_1_28*R23_13_d16 + L_2_28*R23_23_d16 + L_3_28*R23_33_d16 + x23_3_d16;

		xg23_1_d21 = L_1_28*R23_11_d21 + L_2_28*R23_21_d21 + L_3_28*R23_31_d21 + x23_1_d21;
		xg23_2_d21 = L_1_28*R23_12_d21 + L_2_28*R23_22_d21 + L_3_28*R23_32_d21 + x23_2_d21;
		xg23_3_d21 = L_1_28*R23_13_d21 + L_2_28*R23_23_d21 + L_3_28*R23_33_d21 + x23_3_d21;

		xg23_1_d22 = L_1_28*R23_11_d22 + L_2_28*R23_21_d22 + L_3_28*R23_31_d22 + x23_1_d22;
		xg23_2_d22 = L_1_28*R23_12_d22 + L_2_28*R23_22_d22 + L_3_28*R23_32_d22 + x23_2_d22;
		xg23_3_d22 = L_1_28*R23_13_d22 + L_2_28*R23_23_d22 + L_3_28*R23_33_d22 + x23_3_d22;

		xg23_1_d23 = L_1_28*R23_11_d23 + L_2_28*R23_21_d23;
		xg23_2_d23 = L_1_28*R23_12_d23 + L_2_28*R23_22_d23;
		xg23_3_d23 = L_1_28*R23_13_d23 + L_2_28*R23_23_d23;

		xg24_1_d14 = L_1_29*R24_11_d14 + L_2_29*R24_21_d14 + L_3_29*R24_31_d14 + x24_1_d14;
		xg24_2_d14 = L_1_29*R24_12_d14 + L_2_29*R24_22_d14 + L_3_29*R24_32_d14 + x24_2_d14;
		xg24_3_d14 = L_1_29*R24_13_d14 + L_2_29*R24_23_d14 + L_3_29*R24_33_d14 + x24_3_d14;

		xg24_1_d15 = L_1_29*R24_11_d15 + L_2_29*R24_21_d15 + L_3_29*R24_31_d15 + x24_1_d15;
		xg24_2_d15 = L_1_29*R24_12_d15 + L_2_29*R24_22_d15 + L_3_29*R24_32_d15 + x24_2_d15;
		xg24_3_d15 = L_1_29*R24_13_d15 + L_2_29*R24_23_d15 + L_3_29*R24_33_d15 + x24_3_d15;

		xg24_1_d16 = L_1_29*R24_11_d16 + L_2_29*R24_21_d16 + L_3_29*R24_31_d16 + x24_1_d16;
		xg24_2_d16 = L_1_29*R24_12_d16 + L_2_29*R24_22_d16 + L_3_29*R24_32_d16 + x24_2_d16;
		xg24_3_d16 = L_1_29*R24_13_d16 + L_2_29*R24_23_d16 + L_3_29*R24_33_d16 + x24_3_d16;

		xg24_1_d21 = L_1_29*R24_11_d21 + L_2_29*R24_21_d21 + L_3_29*R24_31_d21 + x24_1_d21;
		xg24_2_d21 = L_1_29*R24_12_d21 + L_2_29*R24_22_d21 + L_3_29*R24_32_d21 + x24_2_d21;
		xg24_3_d21 = L_1_29*R24_13_d21 + L_2_29*R24_23_d21 + L_3_29*R24_33_d21 + x24_3_d21;

		xg24_1_d22 = L_1_29*R24_11_d22 + L_2_29*R24_21_d22 + L_3_29*R24_31_d22 + x24_1_d22;
		xg24_2_d22 = L_1_29*R24_12_d22 + L_2_29*R24_22_d22 + L_3_29*R24_32_d22 + x24_2_d22;
		xg24_3_d22 = L_1_29*R24_13_d22 + L_2_29*R24_23_d22 + L_3_29*R24_33_d22 + x24_3_d22;

		xg24_1_d23 = L_1_29*R24_11_d23 + L_2_29*R24_21_d23 + L_3_29*R24_31_d23;
		xg24_2_d23 = L_1_29*R24_12_d23 + L_2_29*R24_22_d23 + L_3_29*R24_32_d23;
		xg24_3_d23 = L_1_29*R24_13_d23 + L_2_29*R24_23_d23 + L_3_29*R24_33_d23;

		xg24_1_d24 = L_1_29*R24_11_d24 + L_3_29*R24_31_d24;
		xg24_2_d24 = L_1_29*R24_12_d24 + L_3_29*R24_32_d24;
		xg24_3_d24 = L_1_29*R24_13_d24 + L_3_29*R24_33_d24;
	}

	// -- Collecting results -- //

	m_tot = M_6 + M_7 + M_8 + M_9 + M_10 + M_11 + M_12 + M_13 + M_14 + M_15 + M_16 + M_17 + M_18 + M_19 + M_20 + M_21 + M_22 + M_23 + M_24 + M_25 + M_26 + M_27 + M_28 + M_29;

	// global com absolute position
	in_out.r_COM[0] = (M_6*xg1_1 + M_7*xg2_1 + M_8*xg3_1 + M_9*xg4_1 + M_10*xg5_1 + M_11*xg6_1 + M_12*xg7_1 + M_13*xg8_1 + M_14*xg9_1 + M_15*xg10_1 + M_16*xg11_1 + M_17*xg12_1 + M_18*xg13_1 + M_19*xg14_1 + M_20*xg15_1 + M_21*xg16_1 + M_22*xg17_1 + M_23*xg18_1 + M_24*xg19_1 + M_25*xg20_1 + M_26*xg21_1 + M_27*xg22_1 + M_28*xg23_1 + M_29*xg24_1)/m_tot;
	in_out.r_COM[1] = (M_6*xg1_2 + M_7*xg2_2 + M_8*xg3_2 + M_9*xg4_2 + M_10*xg5_2 + M_11*xg6_2 + M_12*xg7_2 + M_13*xg8_2 + M_14*xg9_2 + M_15*xg10_2 + M_16*xg11_2 + M_17*xg12_2 + M_18*xg13_2 + M_19*xg14_2 + M_20*xg15_2 + M_21*xg16_2 + M_22*xg17_2 + M_23*xg18_2 + M_24*xg19_2 + M_25*xg20_2 + M_26*xg21_2 + M_27*xg22_2 + M_28*xg23_2 + M_29*xg24_2)/m_tot;
	in_out.r_COM[2] = (M_6*xg1_3 + M_7*xg2_3 + M_8*xg3_3 + M_9*xg4_3 + M_10*xg5_3 + M_11*xg6_3 + M_12*xg7_3 + M_13*xg8_3 + M_14*xg9_3 + M_15*xg10_3 + M_16*xg11_3 + M_17*xg12_3 + M_18*xg13_3 + M_19*xg14_3 + M_20*xg15_3 + M_21*xg16_3 + M_22*xg17_3 + M_23*xg18_3 + M_24*xg19_3 + M_25*xg20_3 + M_26*xg21_3 + M_27*xg22_3 + M_28*xg23_3 + M_29*xg24_3)/m_tot;

	// global com absolute velocity
	in_out.rp_COM[0] = (M_6*xgp1_1 + M_7*xgp2_1 + M_8*xgp3_1 + M_9*xgp4_1 + M_10*xgp5_1 + M_11*xgp6_1 + M_12*xgp7_1 + M_13*xgp8_1 + M_14*xgp9_1 + M_15*xgp10_1 + M_16*xgp11_1 + M_17*xgp12_1 + M_18*xgp13_1 + M_19*xgp14_1 + M_20*xgp15_1 + M_21*xgp16_1 + M_22*xgp17_1 + M_23*xgp18_1 + M_24*xgp19_1 + M_25*xgp20_1 + M_26*xgp21_1 + M_27*xgp22_1 + M_28*xgp23_1 + M_29*xgp24_1)/m_tot;
	in_out.rp_COM[1] = (M_6*xgp1_2 + M_7*xgp2_2 + M_8*xgp3_2 + M_9*xgp4_2 + M_10*xgp5_2 + M_11*xgp6_2 + M_12*xgp7_2 + M_13*xgp8_2 + M_14*xgp9_2 + M_15*xgp10_2 + M_16*xgp11_2 + M_17*xgp12_2 + M_18*xgp13_2 + M_19*xgp14_2 + M_20*xgp15_2 + M_21*xgp16_2 + M_22*xgp17_2 + M_23*xgp18_2 + M_24*xgp19_2 + M_25*xgp20_2 + M_26*xgp21_2 + M_27*xgp22_2 + M_28*xgp23_2 + M_29*xgp24_2)/m_tot;
	in_out.rp_COM[2] = (M_6*xgp1_3 + M_7*xgp2_3 + M_8*xgp3_3 + M_9*xgp4_3 + M_10*xgp5_3 + M_11*xgp6_3 + M_12*xgp7_3 + M_13*xgp8_3 + M_14*xgp9_3 + M_15*xgp10_3 + M_16*xgp11_3 + M_17*xgp12_3 + M_18*xgp13_3 + M_19*xgp14_3 + M_20*xgp15_3 + M_21*xgp16_3 + M_22*xgp17_3 + M_23*xgp18_3 + M_24*xgp19_3 + M_25*xgp20_3 + M_26*xgp21_3 + M_27*xgp22_3 + M_28*xgp23_3 + M_29*xgp24_3)/m_tot;

	// global com jacobian
	if (flag_jacob)
	{
		in_out.r_COM_der[RIGHT_HIP_PITCH][0] = (M_7*xg2_1_d2 + M_8*xg3_1_d2 + M_9*xg4_1_d2 + M_10*xg5_1_d2 + M_11*xg6_1_d2 + M_12*xg7_1_d2)/m_tot;
		in_out.r_COM_der[RIGHT_HIP_PITCH][1] = (M_7*xg2_2_d2 + M_8*xg3_2_d2 + M_9*xg4_2_d2 + M_10*xg5_2_d2 + M_11*xg6_2_d2 + M_12*xg7_2_d2)/m_tot;
		in_out.r_COM_der[RIGHT_HIP_PITCH][2] = (M_7*xg2_3_d2 + M_8*xg3_3_d2 + M_9*xg4_3_d2 + M_10*xg5_3_d2 + M_11*xg6_3_d2 + M_12*xg7_3_d2)/m_tot;

		in_out.r_COM_der[RIGHT_HIP_ROLL][0] = (M_8*xg3_1_d3 + M_9*xg4_1_d3 + M_10*xg5_1_d3 + M_11*xg6_1_d3 + M_12*xg7_1_d3)/m_tot;
		in_out.r_COM_der[RIGHT_HIP_ROLL][1] = (M_8*xg3_2_d3 + M_9*xg4_2_d3 + M_10*xg5_2_d3 + M_11*xg6_2_d3 + M_12*xg7_2_d3)/m_tot;
		in_out.r_COM_der[RIGHT_HIP_ROLL][2] = (M_8*xg3_3_d3 + M_9*xg4_3_d3 + M_10*xg5_3_d3 + M_11*xg6_3_d3 + M_12*xg7_3_d3)/m_tot;

		in_out.r_COM_der[RIGHT_HIP_YAW][0] = (M_9*xg4_1_d4 + M_10*xg5_1_d4 + M_11*xg6_1_d4 + M_12*xg7_1_d4)/m_tot;
		in_out.r_COM_der[RIGHT_HIP_YAW][1] = (M_9*xg4_2_d4 + M_10*xg5_2_d4 + M_11*xg6_2_d4 + M_12*xg7_2_d4)/m_tot;
		in_out.r_COM_der[RIGHT_HIP_YAW][2] = (M_9*xg4_3_d4 + M_10*xg5_3_d4 + M_11*xg6_3_d4 + M_12*xg7_3_d4)/m_tot;

		in_out.r_COM_der[RIGHT_KNEE_PITCH][0] = (M_10*xg5_1_d5 + M_11*xg6_1_d5 + M_12*xg7_1_d5)/m_tot;
		in_out.r_COM_der[RIGHT_KNEE_PITCH][1] = (M_10*xg5_2_d5 + M_11*xg6_2_d5 + M_12*xg7_2_d5)/m_tot;
		in_out.r_COM_der[RIGHT_KNEE_PITCH][2] = (M_10*xg5_3_d5 + M_11*xg6_3_d5 + M_12*xg7_3_d5)/m_tot;

		in_out.r_COM_der[RIGHT_FOOT_ROLL][0] = (M_11*xg6_1_d6 + M_12*xg7_1_d6)/m_tot;
		in_out.r_COM_der[RIGHT_FOOT_ROLL][1] = (M_11*xg6_2_d6 + M_12*xg7_2_d6)/m_tot;
		in_out.r_COM_der[RIGHT_FOOT_ROLL][2] = (M_11*xg6_3_d6 + M_12*xg7_3_d6)/m_tot;

		in_out.r_COM_der[RIGHT_FOOT_PITCH][0] = (M_12*xg7_1_d7)/m_tot;
		in_out.r_COM_der[RIGHT_FOOT_PITCH][1] = (M_12*xg7_2_d7)/m_tot;
		in_out.r_COM_der[RIGHT_FOOT_PITCH][2] = (M_12*xg7_3_d7)/m_tot;

		in_out.r_COM_der[LEFT_HIP_PITCH][0] = (M_13*xg8_1_d8 + M_14*xg9_1_d8 + M_15*xg10_1_d8 + M_16*xg11_1_d8 + M_17*xg12_1_d8 + M_18*xg13_1_d8)/m_tot;
		in_out.r_COM_der[LEFT_HIP_PITCH][1] = (M_13*xg8_2_d8 + M_14*xg9_2_d8 + M_15*xg10_2_d8 + M_16*xg11_2_d8 + M_17*xg12_2_d8 + M_18*xg13_2_d8)/m_tot;
		in_out.r_COM_der[LEFT_HIP_PITCH][2] = (M_13*xg8_3_d8 + M_14*xg9_3_d8 + M_15*xg10_3_d8 + M_16*xg11_3_d8 + M_17*xg12_3_d8 + M_18*xg13_3_d8)/m_tot;

		in_out.r_COM_der[LEFT_HIP_ROLL][0] = (M_14*xg9_1_d9 + M_15*xg10_1_d9 + M_16*xg11_1_d9 + M_17*xg12_1_d9 + M_18*xg13_1_d9)/m_tot;
		in_out.r_COM_der[LEFT_HIP_ROLL][1] = (M_14*xg9_2_d9 + M_15*xg10_2_d9 + M_16*xg11_2_d9 + M_17*xg12_2_d9 + M_18*xg13_2_d9)/m_tot;
		in_out.r_COM_der[LEFT_HIP_ROLL][2] = (M_14*xg9_3_d9 + M_15*xg10_3_d9 + M_16*xg11_3_d9 + M_17*xg12_3_d9 + M_18*xg13_3_d9)/m_tot;

		in_out.r_COM_der[LEFT_HIP_YAW][0] = (M_15*xg10_1_d10 + M_16*xg11_1_d10 + M_17*xg12_1_d10 + M_18*xg13_1_d10)/m_tot;
		in_out.r_COM_der[LEFT_HIP_YAW][1] = (M_15*xg10_2_d10 + M_16*xg11_2_d10 + M_17*xg12_2_d10 + M_18*xg13_2_d10)/m_tot;
		in_out.r_COM_der[LEFT_HIP_YAW][2] = (M_15*xg10_3_d10 + M_16*xg11_3_d10 + M_17*xg12_3_d10 + M_18*xg13_3_d10)/m_tot;

		in_out.r_COM_der[LEFT_KNEE_PITCH][0] = (M_16*xg11_1_d11 + M_17*xg12_1_d11 + M_18*xg13_1_d11)/m_tot;
		in_out.r_COM_der[LEFT_KNEE_PITCH][1] = (M_16*xg11_2_d11 + M_17*xg12_2_d11 + M_18*xg13_2_d11)/m_tot;
		in_out.r_COM_der[LEFT_KNEE_PITCH][2] = (M_16*xg11_3_d11 + M_17*xg12_3_d11 + M_18*xg13_3_d11)/m_tot;

		in_out.r_COM_der[LEFT_FOOT_ROLL][0] = (M_17*xg12_1_d12 + M_18*xg13_1_d12)/m_tot;
		in_out.r_COM_der[LEFT_FOOT_ROLL][1] = (M_17*xg12_2_d12 + M_18*xg13_2_d12)/m_tot;
		in_out.r_COM_der[LEFT_FOOT_ROLL][2] = (M_17*xg12_3_d12 + M_18*xg13_3_d12)/m_tot;

		in_out.r_COM_der[LEFT_FOOT_PITCH][0] = (M_18*xg13_1_d13)/m_tot;
		in_out.r_COM_der[LEFT_FOOT_PITCH][1] = (M_18*xg13_2_d13)/m_tot;
		in_out.r_COM_der[LEFT_FOOT_PITCH][2] = (M_18*xg13_3_d13)/m_tot;

		in_out.r_COM_der[TORSO_ROLL][0] = (M_19*xg14_1_d14 + M_20*xg15_1_d14 + M_21*xg16_1_d14 + M_22*xg17_1_d14 + M_23*xg18_1_d14 + M_24*xg19_1_d14 + M_25*xg20_1_d14 + M_26*xg21_1_d14 + M_27*xg22_1_d14 + M_28*xg23_1_d14 + M_29*xg24_1_d14)/m_tot;
		in_out.r_COM_der[TORSO_ROLL][1] = (M_19*xg14_2_d14 + M_20*xg15_2_d14 + M_21*xg16_2_d14 + M_22*xg17_2_d14 + M_23*xg18_2_d14 + M_24*xg19_2_d14 + M_25*xg20_2_d14 + M_26*xg21_2_d14 + M_27*xg22_2_d14 + M_28*xg23_2_d14 + M_29*xg24_2_d14)/m_tot;
		in_out.r_COM_der[TORSO_ROLL][2] = (M_19*xg14_3_d14 + M_20*xg15_3_d14 + M_21*xg16_3_d14 + M_22*xg17_3_d14 + M_23*xg18_3_d14 + M_24*xg19_3_d14 + M_25*xg20_3_d14 + M_26*xg21_3_d14 + M_27*xg22_3_d14 + M_28*xg23_3_d14 + M_29*xg24_3_d14)/m_tot;

		in_out.r_COM_der[TORSO_PITCH][0] = (M_20*xg15_1_d15 + M_21*xg16_1_d15 + M_22*xg17_1_d15 + M_23*xg18_1_d15 + M_24*xg19_1_d15 + M_25*xg20_1_d15 + M_26*xg21_1_d15 + M_27*xg22_1_d15 + M_28*xg23_1_d15 + M_29*xg24_1_d15)/m_tot;
		in_out.r_COM_der[TORSO_PITCH][1] = (M_20*xg15_2_d15 + M_21*xg16_2_d15 + M_22*xg17_2_d15 + M_23*xg18_2_d15 + M_24*xg19_2_d15 + M_25*xg20_2_d15 + M_26*xg21_2_d15 + M_27*xg22_2_d15 + M_28*xg23_2_d15 + M_29*xg24_2_d15)/m_tot;
		in_out.r_COM_der[TORSO_PITCH][2] = (M_20*xg15_3_d15 + M_21*xg16_3_d15 + M_22*xg17_3_d15 + M_23*xg18_3_d15 + M_24*xg19_3_d15 + M_25*xg20_3_d15 + M_26*xg21_3_d15 + M_27*xg22_3_d15 + M_28*xg23_3_d15 + M_29*xg24_3_d15)/m_tot;

		in_out.r_COM_der[TORSO_YAW][0] = (M_21*xg16_1_d16 + M_22*xg17_1_d16 + M_23*xg18_1_d16 + M_24*xg19_1_d16 + M_25*xg20_1_d16 + M_26*xg21_1_d16 + M_27*xg22_1_d16 + M_28*xg23_1_d16 + M_29*xg24_1_d16)/m_tot;
		in_out.r_COM_der[TORSO_YAW][1] = (M_21*xg16_2_d16 + M_22*xg17_2_d16 + M_23*xg18_2_d16 + M_24*xg19_2_d16 + M_25*xg20_2_d16 + M_26*xg21_2_d16 + M_27*xg22_2_d16 + M_28*xg23_2_d16 + M_29*xg24_2_d16)/m_tot;
		in_out.r_COM_der[TORSO_YAW][2] = (M_21*xg16_3_d16 + M_22*xg17_3_d16 + M_23*xg18_3_d16 + M_24*xg19_3_d16 + M_25*xg20_3_d16 + M_26*xg21_3_d16 + M_27*xg22_3_d16 + M_28*xg23_3_d16 + M_29*xg24_3_d16)/m_tot;

		in_out.r_COM_der[RIGHT_SH_PITCH][0] = (M_22*xg17_1_d17 + M_23*xg18_1_d17 + M_24*xg19_1_d17 + M_25*xg20_1_d17)/m_tot;
		in_out.r_COM_der[RIGHT_SH_PITCH][1] = (M_22*xg17_2_d17 + M_23*xg18_2_d17 + M_24*xg19_2_d17 + M_25*xg20_2_d17)/m_tot;
		in_out.r_COM_der[RIGHT_SH_PITCH][2] = (M_22*xg17_3_d17 + M_23*xg18_3_d17 + M_24*xg19_3_d17 + M_25*xg20_3_d17)/m_tot;

		in_out.r_COM_der[RIGHT_SH_ROLL][0] = (M_23*xg18_1_d18 + M_24*xg19_1_d18 + M_25*xg20_1_d18)/m_tot;
		in_out.r_COM_der[RIGHT_SH_ROLL][1] = (M_23*xg18_2_d18 + M_24*xg19_2_d18 + M_25*xg20_2_d18)/m_tot;
		in_out.r_COM_der[RIGHT_SH_ROLL][2] = (M_23*xg18_3_d18 + M_24*xg19_3_d18 + M_25*xg20_3_d18)/m_tot;

		in_out.r_COM_der[RIGHT_SH_YAW][0] = (M_24*xg19_1_d19 + M_25*xg20_1_d19)/m_tot;
		in_out.r_COM_der[RIGHT_SH_YAW][1] = (M_24*xg19_2_d19 + M_25*xg20_2_d19)/m_tot;
		in_out.r_COM_der[RIGHT_SH_YAW][2] = (M_24*xg19_3_d19 + M_25*xg20_3_d19)/m_tot;

		in_out.r_COM_der[RIGHT_ELB_PITCH][0] = (M_25*xg20_1_d20)/m_tot;
		in_out.r_COM_der[RIGHT_ELB_PITCH][1] = (M_25*xg20_2_d20)/m_tot;
		in_out.r_COM_der[RIGHT_ELB_PITCH][2] = (M_25*xg20_3_d20)/m_tot;

		in_out.r_COM_der[LEFT_SH_PITCH][0] = (M_26*xg21_1_d21 + M_27*xg22_1_d21 + M_28*xg23_1_d21 + M_29*xg24_1_d21)/m_tot;
		in_out.r_COM_der[LEFT_SH_PITCH][1] = (M_26*xg21_2_d21 + M_27*xg22_2_d21 + M_28*xg23_2_d21 + M_29*xg24_2_d21)/m_tot;
		in_out.r_COM_der[LEFT_SH_PITCH][2] = (M_26*xg21_3_d21 + M_27*xg22_3_d21 + M_28*xg23_3_d21 + M_29*xg24_3_d21)/m_tot;

		in_out.r_COM_der[LEFT_SH_ROLL][0] = (M_27*xg22_1_d22 + M_28*xg23_1_d22 + M_29*xg24_1_d22)/m_tot;
		in_out.r_COM_der[LEFT_SH_ROLL][1] = (M_27*xg22_2_d22 + M_28*xg23_2_d22 + M_29*xg24_2_d22)/m_tot;
		in_out.r_COM_der[LEFT_SH_ROLL][2] = (M_27*xg22_3_d22 + M_28*xg23_3_d22 + M_29*xg24_3_d22)/m_tot;

		in_out.r_COM_der[LEFT_SH_YAW][0] = (M_28*xg23_1_d23 + M_29*xg24_1_d23)/m_tot;
		in_out.r_COM_der[LEFT_SH_YAW][1] = (M_28*xg23_2_d23 + M_29*xg24_2_d23)/m_tot;
		in_out.r_COM_der[LEFT_SH_YAW][2] = (M_28*xg23_3_d23 + M_29*xg24_3_d23)/m_tot;

		in_out.r_COM_der[LEFT_ELB_PITCH][0] = (M_29*xg24_1_d24)/m_tot;
		in_out.r_COM_der[LEFT_ELB_PITCH][1] = (M_29*xg24_2_d24)/m_tot;
		in_out.r_COM_der[LEFT_ELB_PITCH][2] = (M_29*xg24_3_d24)/m_tot;
	}

	// right foot absolute position
	in_out.r_Rfoot[0] = DPT_3_16*R7_31 + x7_1;
	in_out.r_Rfoot[1] = DPT_3_16*R7_32 + x7_2;
	in_out.r_Rfoot[2] = DPT_3_16*R7_33 + x7_3;

	// right foot absolute velocity
	in_out.rp_Rfoot[0] = -DPT_3_16*R7_32*om7_3 + DPT_3_16*R7_33*om7_2 + xp7_1;
	in_out.rp_Rfoot[1] = DPT_3_16*R7_31*om7_3 - DPT_3_16*R7_33*om7_1 + xp7_2;
	in_out.rp_Rfoot[2] = -DPT_3_16*R7_31*om7_2 + DPT_3_16*R7_32*om7_1 + xp7_3;

	// right foot jacobian
	if (flag_jacob)
	{
		in_out.r_Rfoot_der[RIGHT_HIP_PITCH][0] = DPT_3_16*R7_31_d2 + x7_1_d2;
		in_out.r_Rfoot_der[RIGHT_HIP_PITCH][1] = DPT_3_16*R7_32_d2 + x7_2_d2;
		in_out.r_Rfoot_der[RIGHT_HIP_PITCH][2] = DPT_3_16*R7_33_d2 + x7_3_d2;

		in_out.r_Rfoot_der[RIGHT_HIP_ROLL][0] = DPT_3_16*R7_31_d3 + x7_1_d3;
		in_out.r_Rfoot_der[RIGHT_HIP_ROLL][1] = DPT_3_16*R7_32_d3 + x7_2_d3;
		in_out.r_Rfoot_der[RIGHT_HIP_ROLL][2] = DPT_3_16*R7_33_d3 + x7_3_d3;

		in_out.r_Rfoot_der[RIGHT_HIP_YAW][0] = DPT_3_16*R7_31_d4 + x7_1_d4;
		in_out.r_Rfoot_der[RIGHT_HIP_YAW][1] = DPT_3_16*R7_32_d4 + x7_2_d4;
		in_out.r_Rfoot_der[RIGHT_HIP_YAW][2] = DPT_3_16*R7_33_d4 + x7_3_d4;

		in_out.r_Rfoot_der[RIGHT_KNEE_PITCH][0] = DPT_3_16*R7_31_d5 + x7_1_d5;
		in_out.r_Rfoot_der[RIGHT_KNEE_PITCH][1] = DPT_3_16*R7_32_d5 + x7_2_d5;
		in_out.r_Rfoot_der[RIGHT_KNEE_PITCH][2] = DPT_3_16*R7_33_d5 + x7_3_d5;

		in_out.r_Rfoot_der[RIGHT_FOOT_ROLL][0] = DPT_3_16*R7_31_d6;
		in_out.r_Rfoot_der[RIGHT_FOOT_ROLL][1] = DPT_3_16*R7_32_d6;
		in_out.r_Rfoot_der[RIGHT_FOOT_ROLL][2] = DPT_3_16*R7_33_d6;

		in_out.r_Rfoot_der[RIGHT_FOOT_PITCH][0] = DPT_3_16*R7_31_d7;
		in_out.r_Rfoot_der[RIGHT_FOOT_PITCH][1] = DPT_3_16*R7_32_d7;
		in_out.r_Rfoot_der[RIGHT_FOOT_PITCH][2] = DPT_3_16*R7_33_d7;
	}

	// left foot absolute position
	in_out.r_Lfoot[0] = DPT_3_29*R13_31 + x13_1;
	in_out.r_Lfoot[1] = DPT_3_29*R13_32 + x13_2;
	in_out.r_Lfoot[2] = DPT_3_29*R13_33 + x13_3;

	// left foot absolute velocity
	in_out.rp_Lfoot[0] = -DPT_3_29*R13_32*om13_3 + DPT_3_29*R13_33*om13_2 + xp13_1;
	in_out.rp_Lfoot[1] = DPT_3_29*R13_31*om13_3 - DPT_3_29*R13_33*om13_1 + xp13_2;
	in_out.rp_Lfoot[2] = -DPT_3_29*R13_31*om13_2 + DPT_3_29*R13_32*om13_1 + xp13_3;

	// left foot jacobian
	if (flag_jacob)
	{
		in_out.r_Lfoot_der[LEFT_HIP_PITCH][0] = DPT_3_29*R13_31_d8 + x13_1_d8;
		in_out.r_Lfoot_der[LEFT_HIP_PITCH][1] = DPT_3_29*R13_32_d8 + x13_2_d8;
		in_out.r_Lfoot_der[LEFT_HIP_PITCH][2] = DPT_3_29*R13_33_d8 + x13_3_d8;

		in_out.r_Lfoot_der[LEFT_HIP_ROLL][0] = DPT_3_29*R13_31_d9 + x13_1_d9;
		in_out.r_Lfoot_der[LEFT_HIP_ROLL][1] = DPT_3_29*R13_32_d9 + x13_2_d9;
		in_out.r_Lfoot_der[LEFT_HIP_ROLL][2] = DPT_3_29*R13_33_d9 + x13_3_d9;

		in_out.r_Lfoot_der[LEFT_HIP_YAW][0] = DPT_3_29*R13_31_d10 + x13_1_d10;
		in_out.r_Lfoot_der[LEFT_HIP_YAW][1] = DPT_3_29*R13_32_d10 + x13_2_d10;
		in_out.r_Lfoot_der[LEFT_HIP_YAW][2] = DPT_3_29*R13_33_d10 + x13_3_d10;

		in_out.r_Lfoot_der[LEFT_KNEE_PITCH][0] = DPT_3_29*R13_31_d11 + x13_1_d11;
		in_out.r_Lfoot_der[LEFT_KNEE_PITCH][1] = DPT_3_29*R13_32_d11 + x13_2_d11;
		in_out.r_Lfoot_der[LEFT_KNEE_PITCH][2] = DPT_3_29*R13_33_d11 + x13_3_d11;

		in_out.r_Lfoot_der[LEFT_FOOT_ROLL][0] = DPT_3_29*R13_31_d12;
		in_out.r_Lfoot_der[LEFT_FOOT_ROLL][1] = DPT_3_29*R13_32_d12;
		in_out.r_Lfoot_der[LEFT_FOOT_ROLL][2] = DPT_3_29*R13_33_d12;

		in_out.r_Lfoot_der[LEFT_FOOT_PITCH][0] = DPT_3_29*R13_31_d13;
		in_out.r_Lfoot_der[LEFT_FOOT_PITCH][1] = DPT_3_29*R13_32_d13;
		in_out.r_Lfoot_der[LEFT_FOOT_PITCH][2] = DPT_3_29*R13_33_d13;
	}

	// right foot contact points absolute position
	in_out.r_Rfoot_cont[0][0] = DPT_3_16*R7_31 - 0.06*R7_11 - 0.045*R7_21 + x7_1;
	in_out.r_Rfoot_cont[0][1] = DPT_3_16*R7_32 - 0.06*R7_12 - 0.045*R7_22 + x7_2;
	in_out.r_Rfoot_cont[0][2] = DPT_3_16*R7_33 - 0.06*R7_13 - 0.045*R7_23 + x7_3;

	in_out.r_Rfoot_cont[1][0] = DPT_3_16*R7_31 - 0.06*R7_11 + 0.045*R7_21 + x7_1;
	in_out.r_Rfoot_cont[1][1] = DPT_3_16*R7_32 - 0.06*R7_12 + 0.045*R7_22 + x7_2;
	in_out.r_Rfoot_cont[1][2] = DPT_3_16*R7_33 - 0.06*R7_13 + 0.045*R7_23 + x7_3;

	in_out.r_Rfoot_cont[2][0] = DPT_3_16*R7_31 + 0.08*R7_11 - 0.045*R7_21 + x7_1;
	in_out.r_Rfoot_cont[2][1] = DPT_3_16*R7_32 + 0.08*R7_12 - 0.045*R7_22 + x7_2;
	in_out.r_Rfoot_cont[2][2] = DPT_3_16*R7_33 + 0.08*R7_13 - 0.045*R7_23 + x7_3;

	in_out.r_Rfoot_cont[3][0] = DPT_3_16*R7_31 + 0.08*R7_11 + 0.045*R7_21 + x7_1;
	in_out.r_Rfoot_cont[3][1] = DPT_3_16*R7_32 + 0.08*R7_12 + 0.045*R7_22 + x7_2;
	in_out.r_Rfoot_cont[3][2] = DPT_3_16*R7_33 + 0.08*R7_13 + 0.045*R7_23 + x7_3;

	// right foot contact points jacobian
	if (flag_jacob)
	{
		in_out.r_Rfoot_cont_der[0][RIGHT_HIP_PITCH][0] = -0.045*R7_21_d2 - 0.06*R7_11_d2 + DPT_3_16*R7_31_d2 + x7_1_d2;
		in_out.r_Rfoot_cont_der[0][RIGHT_HIP_PITCH][1] = -0.045*R7_22_d2 - 0.06*R7_12_d2 + DPT_3_16*R7_32_d2 + x7_2_d2;
		in_out.r_Rfoot_cont_der[0][RIGHT_HIP_PITCH][2] = -0.045*R7_23_d2 - 0.06*R7_13_d2 + DPT_3_16*R7_33_d2 + x7_3_d2;

		in_out.r_Rfoot_cont_der[0][RIGHT_HIP_ROLL][0] = -0.045*R7_21_d3 - 0.06*R7_11_d3 + DPT_3_16*R7_31_d3 + x7_1_d3;
		in_out.r_Rfoot_cont_der[0][RIGHT_HIP_ROLL][1] = -0.045*R7_22_d3 - 0.06*R7_12_d3 + DPT_3_16*R7_32_d3 + x7_2_d3;
		in_out.r_Rfoot_cont_der[0][RIGHT_HIP_ROLL][2] = -0.045*R7_23_d3 - 0.06*R7_13_d3 + DPT_3_16*R7_33_d3 + x7_3_d3;

		in_out.r_Rfoot_cont_der[0][RIGHT_HIP_YAW][0] = -0.045*R7_21_d4 - 0.06*R7_11_d4 + DPT_3_16*R7_31_d4 + x7_1_d4;
		in_out.r_Rfoot_cont_der[0][RIGHT_HIP_YAW][1] = -0.045*R7_22_d4 - 0.06*R7_12_d4 + DPT_3_16*R7_32_d4 + x7_2_d4;
		in_out.r_Rfoot_cont_der[0][RIGHT_HIP_YAW][2] = -0.045*R7_23_d4 - 0.06*R7_13_d4 + DPT_3_16*R7_33_d4 + x7_3_d4;

		in_out.r_Rfoot_cont_der[0][RIGHT_KNEE_PITCH][0] = -0.045*R7_21_d5 - 0.06*R7_11_d5 + DPT_3_16*R7_31_d5 + x7_1_d5;
		in_out.r_Rfoot_cont_der[0][RIGHT_KNEE_PITCH][1] = -0.045*R7_22_d5 - 0.06*R7_12_d5 + DPT_3_16*R7_32_d5 + x7_2_d5;
		in_out.r_Rfoot_cont_der[0][RIGHT_KNEE_PITCH][2] = -0.045*R7_23_d5 - 0.06*R7_13_d5 + DPT_3_16*R7_33_d5 + x7_3_d5;

		in_out.r_Rfoot_cont_der[0][RIGHT_FOOT_ROLL][0] = -0.045*R7_21_d6 - 0.06*R7_11_d6 + DPT_3_16*R7_31_d6;
		in_out.r_Rfoot_cont_der[0][RIGHT_FOOT_ROLL][1] = -0.045*R7_22_d6 - 0.06*R7_12_d6 + DPT_3_16*R7_32_d6;
		in_out.r_Rfoot_cont_der[0][RIGHT_FOOT_ROLL][2] = -0.045*R7_23_d6 - 0.06*R7_13_d6 + DPT_3_16*R7_33_d6;

		in_out.r_Rfoot_cont_der[0][RIGHT_FOOT_PITCH][0] = -0.06*R7_11_d7 + DPT_3_16*R7_31_d7;
		in_out.r_Rfoot_cont_der[0][RIGHT_FOOT_PITCH][1] = -0.06*R7_12_d7 + DPT_3_16*R7_32_d7;
		in_out.r_Rfoot_cont_der[0][RIGHT_FOOT_PITCH][2] = -0.06*R7_13_d7 + DPT_3_16*R7_33_d7;

		in_out.r_Rfoot_cont_der[1][RIGHT_HIP_PITCH][0] = -0.06*R7_11_d2 + 0.045*R7_21_d2 + DPT_3_16*R7_31_d2 + x7_1_d2;
		in_out.r_Rfoot_cont_der[1][RIGHT_HIP_PITCH][1] = -0.06*R7_12_d2 + 0.045*R7_22_d2 + DPT_3_16*R7_32_d2 + x7_2_d2;
		in_out.r_Rfoot_cont_der[1][RIGHT_HIP_PITCH][2] = -0.06*R7_13_d2 + 0.045*R7_23_d2 + DPT_3_16*R7_33_d2 + x7_3_d2;

		in_out.r_Rfoot_cont_der[1][RIGHT_HIP_ROLL][0] = -0.06*R7_11_d3 + 0.045*R7_21_d3 + DPT_3_16*R7_31_d3 + x7_1_d3;
		in_out.r_Rfoot_cont_der[1][RIGHT_HIP_ROLL][1] = -0.06*R7_12_d3 + 0.045*R7_22_d3 + DPT_3_16*R7_32_d3 + x7_2_d3;
		in_out.r_Rfoot_cont_der[1][RIGHT_HIP_ROLL][2] = -0.06*R7_13_d3 + 0.045*R7_23_d3 + DPT_3_16*R7_33_d3 + x7_3_d3;

		in_out.r_Rfoot_cont_der[1][RIGHT_HIP_YAW][0] = -0.06*R7_11_d4 + 0.045*R7_21_d4 + DPT_3_16*R7_31_d4 + x7_1_d4;
		in_out.r_Rfoot_cont_der[1][RIGHT_HIP_YAW][1] = -0.06*R7_12_d4 + 0.045*R7_22_d4 + DPT_3_16*R7_32_d4 + x7_2_d4;
		in_out.r_Rfoot_cont_der[1][RIGHT_HIP_YAW][2] = -0.06*R7_13_d4 + 0.045*R7_23_d4 + DPT_3_16*R7_33_d4 + x7_3_d4;

		in_out.r_Rfoot_cont_der[1][RIGHT_KNEE_PITCH][0] = -0.06*R7_11_d5 + 0.045*R7_21_d5 + DPT_3_16*R7_31_d5 + x7_1_d5;
		in_out.r_Rfoot_cont_der[1][RIGHT_KNEE_PITCH][1] = -0.06*R7_12_d5 + 0.045*R7_22_d5 + DPT_3_16*R7_32_d5 + x7_2_d5;
		in_out.r_Rfoot_cont_der[1][RIGHT_KNEE_PITCH][2] = -0.06*R7_13_d5 + 0.045*R7_23_d5 + DPT_3_16*R7_33_d5 + x7_3_d5;

		in_out.r_Rfoot_cont_der[1][RIGHT_FOOT_ROLL][0] = -0.06*R7_11_d6 + 0.045*R7_21_d6 + DPT_3_16*R7_31_d6;
		in_out.r_Rfoot_cont_der[1][RIGHT_FOOT_ROLL][1] = -0.06*R7_12_d6 + 0.045*R7_22_d6 + DPT_3_16*R7_32_d6;
		in_out.r_Rfoot_cont_der[1][RIGHT_FOOT_ROLL][2] = -0.06*R7_13_d6 + 0.045*R7_23_d6 + DPT_3_16*R7_33_d6;

		in_out.r_Rfoot_cont_der[1][RIGHT_FOOT_PITCH][0] = -0.06*R7_11_d7 + DPT_3_16*R7_31_d7;
		in_out.r_Rfoot_cont_der[1][RIGHT_FOOT_PITCH][1] = -0.06*R7_12_d7 + DPT_3_16*R7_32_d7;
		in_out.r_Rfoot_cont_der[1][RIGHT_FOOT_PITCH][2] = -0.06*R7_13_d7 + DPT_3_16*R7_33_d7;

		in_out.r_Rfoot_cont_der[2][RIGHT_HIP_PITCH][0] = -0.045*R7_21_d2 + 0.08*R7_11_d2 + DPT_3_16*R7_31_d2 + x7_1_d2;
		in_out.r_Rfoot_cont_der[2][RIGHT_HIP_PITCH][1] = -0.045*R7_22_d2 + 0.08*R7_12_d2 + DPT_3_16*R7_32_d2 + x7_2_d2;
		in_out.r_Rfoot_cont_der[2][RIGHT_HIP_PITCH][2] = -0.045*R7_23_d2 + 0.08*R7_13_d2 + DPT_3_16*R7_33_d2 + x7_3_d2;

		in_out.r_Rfoot_cont_der[2][RIGHT_HIP_ROLL][0] = -0.045*R7_21_d3 + 0.08*R7_11_d3 + DPT_3_16*R7_31_d3 + x7_1_d3;
		in_out.r_Rfoot_cont_der[2][RIGHT_HIP_ROLL][1] = -0.045*R7_22_d3 + 0.08*R7_12_d3 + DPT_3_16*R7_32_d3 + x7_2_d3;
		in_out.r_Rfoot_cont_der[2][RIGHT_HIP_ROLL][2] = -0.045*R7_23_d3 + 0.08*R7_13_d3 + DPT_3_16*R7_33_d3 + x7_3_d3;

		in_out.r_Rfoot_cont_der[2][RIGHT_HIP_YAW][0] = -0.045*R7_21_d4 + 0.08*R7_11_d4 + DPT_3_16*R7_31_d4 + x7_1_d4;
		in_out.r_Rfoot_cont_der[2][RIGHT_HIP_YAW][1] = -0.045*R7_22_d4 + 0.08*R7_12_d4 + DPT_3_16*R7_32_d4 + x7_2_d4;
		in_out.r_Rfoot_cont_der[2][RIGHT_HIP_YAW][2] = -0.045*R7_23_d4 + 0.08*R7_13_d4 + DPT_3_16*R7_33_d4 + x7_3_d4;

		in_out.r_Rfoot_cont_der[2][RIGHT_KNEE_PITCH][0] = -0.045*R7_21_d5 + 0.08*R7_11_d5 + DPT_3_16*R7_31_d5 + x7_1_d5;
		in_out.r_Rfoot_cont_der[2][RIGHT_KNEE_PITCH][1] = -0.045*R7_22_d5 + 0.08*R7_12_d5 + DPT_3_16*R7_32_d5 + x7_2_d5;
		in_out.r_Rfoot_cont_der[2][RIGHT_KNEE_PITCH][2] = -0.045*R7_23_d5 + 0.08*R7_13_d5 + DPT_3_16*R7_33_d5 + x7_3_d5;

		in_out.r_Rfoot_cont_der[2][RIGHT_FOOT_ROLL][0] = -0.045*R7_21_d6 + 0.08*R7_11_d6 + DPT_3_16*R7_31_d6;
		in_out.r_Rfoot_cont_der[2][RIGHT_FOOT_ROLL][1] = -0.045*R7_22_d6 + 0.08*R7_12_d6 + DPT_3_16*R7_32_d6;
		in_out.r_Rfoot_cont_der[2][RIGHT_FOOT_ROLL][2] = -0.045*R7_23_d6 + 0.08*R7_13_d6 + DPT_3_16*R7_33_d6;

		in_out.r_Rfoot_cont_der[2][RIGHT_FOOT_PITCH][0] = 0.08*R7_11_d7 + DPT_3_16*R7_31_d7;
		in_out.r_Rfoot_cont_der[2][RIGHT_FOOT_PITCH][1] = 0.08*R7_12_d7 + DPT_3_16*R7_32_d7;
		in_out.r_Rfoot_cont_der[2][RIGHT_FOOT_PITCH][2] = 0.08*R7_13_d7 + DPT_3_16*R7_33_d7;

		in_out.r_Rfoot_cont_der[3][RIGHT_HIP_PITCH][0] = 0.045*R7_21_d2 + 0.08*R7_11_d2 + DPT_3_16*R7_31_d2 + x7_1_d2;
		in_out.r_Rfoot_cont_der[3][RIGHT_HIP_PITCH][1] = 0.045*R7_22_d2 + 0.08*R7_12_d2 + DPT_3_16*R7_32_d2 + x7_2_d2;
		in_out.r_Rfoot_cont_der[3][RIGHT_HIP_PITCH][2] = 0.045*R7_23_d2 + 0.08*R7_13_d2 + DPT_3_16*R7_33_d2 + x7_3_d2;

		in_out.r_Rfoot_cont_der[3][RIGHT_HIP_ROLL][0] = 0.045*R7_21_d3 + 0.08*R7_11_d3 + DPT_3_16*R7_31_d3 + x7_1_d3;
		in_out.r_Rfoot_cont_der[3][RIGHT_HIP_ROLL][1] = 0.045*R7_22_d3 + 0.08*R7_12_d3 + DPT_3_16*R7_32_d3 + x7_2_d3;
		in_out.r_Rfoot_cont_der[3][RIGHT_HIP_ROLL][2] = 0.045*R7_23_d3 + 0.08*R7_13_d3 + DPT_3_16*R7_33_d3 + x7_3_d3;

		in_out.r_Rfoot_cont_der[3][RIGHT_HIP_YAW][0] = 0.045*R7_21_d4 + 0.08*R7_11_d4 + DPT_3_16*R7_31_d4 + x7_1_d4;
		in_out.r_Rfoot_cont_der[3][RIGHT_HIP_YAW][1] = 0.045*R7_22_d4 + 0.08*R7_12_d4 + DPT_3_16*R7_32_d4 + x7_2_d4;
		in_out.r_Rfoot_cont_der[3][RIGHT_HIP_YAW][2] = 0.045*R7_23_d4 + 0.08*R7_13_d4 + DPT_3_16*R7_33_d4 + x7_3_d4;

		in_out.r_Rfoot_cont_der[3][RIGHT_KNEE_PITCH][0] = 0.045*R7_21_d5 + 0.08*R7_11_d5 + DPT_3_16*R7_31_d5 + x7_1_d5;
		in_out.r_Rfoot_cont_der[3][RIGHT_KNEE_PITCH][1] = 0.045*R7_22_d5 + 0.08*R7_12_d5 + DPT_3_16*R7_32_d5 + x7_2_d5;
		in_out.r_Rfoot_cont_der[3][RIGHT_KNEE_PITCH][2] = 0.045*R7_23_d5 + 0.08*R7_13_d5 + DPT_3_16*R7_33_d5 + x7_3_d5;

		in_out.r_Rfoot_cont_der[3][RIGHT_FOOT_ROLL][0] = 0.045*R7_21_d6 + 0.08*R7_11_d6 + DPT_3_16*R7_31_d6;
		in_out.r_Rfoot_cont_der[3][RIGHT_FOOT_ROLL][1] = 0.045*R7_22_d6 + 0.08*R7_12_d6 + DPT_3_16*R7_32_d6;
		in_out.r_Rfoot_cont_der[3][RIGHT_FOOT_ROLL][2] = 0.045*R7_23_d6 + 0.08*R7_13_d6 + DPT_3_16*R7_33_d6;

		in_out.r_Rfoot_cont_der[3][RIGHT_FOOT_PITCH][0] = 0.08*R7_11_d7 + DPT_3_16*R7_31_d7;
		in_out.r_Rfoot_cont_der[3][RIGHT_FOOT_PITCH][1] = 0.08*R7_12_d7 + DPT_3_16*R7_32_d7;
		in_out.r_Rfoot_cont_der[3][RIGHT_FOOT_PITCH][2] = 0.08*R7_13_d7 + DPT_3_16*R7_33_d7;
	}

	// left foot contact points absolute position
	in_out.r_Lfoot_cont[0][0] = DPT_3_16*R13_31 - 0.06*R13_11 - 0.045*R13_21 + x13_1;
	in_out.r_Lfoot_cont[0][1] = DPT_3_16*R13_32 - 0.06*R13_12 - 0.045*R13_22 + x13_2;
	in_out.r_Lfoot_cont[0][2] = DPT_3_16*R13_33 - 0.06*R13_13 - 0.045*R13_23 + x13_3;

	in_out.r_Lfoot_cont[1][0] = DPT_3_16*R13_31 - 0.06*R13_11 + 0.045*R13_21 + x13_1;
	in_out.r_Lfoot_cont[1][1] = DPT_3_16*R13_32 - 0.06*R13_12 + 0.045*R13_22 + x13_2;
	in_out.r_Lfoot_cont[1][2] = DPT_3_16*R13_33 - 0.06*R13_13 + 0.045*R13_23 + x13_3;

	in_out.r_Lfoot_cont[2][0] = DPT_3_16*R13_31 + 0.08*R13_11 - 0.045*R13_21 + x13_1;
	in_out.r_Lfoot_cont[2][1] = DPT_3_16*R13_32 + 0.08*R13_12 - 0.045*R13_22 + x13_2;
	in_out.r_Lfoot_cont[2][2] = DPT_3_16*R13_33 + 0.08*R13_13 - 0.045*R13_23 + x13_3;

	in_out.r_Lfoot_cont[3][0] = DPT_3_16*R13_31 + 0.08*R13_11 + 0.045*R13_21 + x13_1;
	in_out.r_Lfoot_cont[3][1] = DPT_3_16*R13_32 + 0.08*R13_12 + 0.045*R13_22 + x13_2;
	in_out.r_Lfoot_cont[3][2] = DPT_3_16*R13_33 + 0.08*R13_13 + 0.045*R13_23 + x13_3;

	// left foot contact points jacobian
	if (flag_jacob)
	{
		in_out.r_Lfoot_cont_der[0][LEFT_HIP_PITCH][0] = -0.045*R13_21_d8 - 0.06*R13_11_d8 + DPT_3_16*R13_31_d8 + x13_1_d8;
		in_out.r_Lfoot_cont_der[0][LEFT_HIP_PITCH][1] = -0.045*R13_22_d8 - 0.06*R13_12_d8 + DPT_3_16*R13_32_d8 + x13_2_d8;
		in_out.r_Lfoot_cont_der[0][LEFT_HIP_PITCH][2] = -0.045*R13_23_d8 - 0.06*R13_13_d8 + DPT_3_16*R13_33_d8 + x13_3_d8;

		in_out.r_Lfoot_cont_der[0][LEFT_HIP_ROLL][0] = -0.045*R13_21_d9 - 0.06*R13_11_d9 + DPT_3_16*R13_31_d9 + x13_1_d9;
		in_out.r_Lfoot_cont_der[0][LEFT_HIP_ROLL][1] = -0.045*R13_22_d9 - 0.06*R13_12_d9 + DPT_3_16*R13_32_d9 + x13_2_d9;
		in_out.r_Lfoot_cont_der[0][LEFT_HIP_ROLL][2] = -0.045*R13_23_d9 - 0.06*R13_13_d9 + DPT_3_16*R13_33_d9 + x13_3_d9;

		in_out.r_Lfoot_cont_der[0][LEFT_HIP_YAW][0] = -0.045*R13_21_d10 - 0.06*R13_11_d10 + DPT_3_16*R13_31_d10 + x13_1_d10;
		in_out.r_Lfoot_cont_der[0][LEFT_HIP_YAW][1] = -0.045*R13_22_d10 - 0.06*R13_12_d10 + DPT_3_16*R13_32_d10 + x13_2_d10;
		in_out.r_Lfoot_cont_der[0][LEFT_HIP_YAW][2] = -0.045*R13_23_d10 - 0.06*R13_13_d10 + DPT_3_16*R13_33_d10 + x13_3_d10;

		in_out.r_Lfoot_cont_der[0][LEFT_KNEE_PITCH][0] = -0.045*R13_21_d11 - 0.06*R13_11_d11 + DPT_3_16*R13_31_d11 + x13_1_d11;
		in_out.r_Lfoot_cont_der[0][LEFT_KNEE_PITCH][1] = -0.045*R13_22_d11 - 0.06*R13_12_d11 + DPT_3_16*R13_32_d11 + x13_2_d11;
		in_out.r_Lfoot_cont_der[0][LEFT_KNEE_PITCH][2] = -0.045*R13_23_d11 - 0.06*R13_13_d11 + DPT_3_16*R13_33_d11 + x13_3_d11;

		in_out.r_Lfoot_cont_der[0][LEFT_FOOT_ROLL][0] = -0.045*R13_21_d12 - 0.06*R13_11_d12 + DPT_3_16*R13_31_d12;
		in_out.r_Lfoot_cont_der[0][LEFT_FOOT_ROLL][1] = -0.045*R13_22_d12 - 0.06*R13_12_d12 + DPT_3_16*R13_32_d12;
		in_out.r_Lfoot_cont_der[0][LEFT_FOOT_ROLL][2] = -0.045*R13_23_d12 - 0.06*R13_13_d12 + DPT_3_16*R13_33_d12;

		in_out.r_Lfoot_cont_der[0][LEFT_FOOT_PITCH][0] = -0.06*R13_11_d13 + DPT_3_16*R13_31_d13;
		in_out.r_Lfoot_cont_der[0][LEFT_FOOT_PITCH][1] = -0.06*R13_12_d13 + DPT_3_16*R13_32_d13;
		in_out.r_Lfoot_cont_der[0][LEFT_FOOT_PITCH][2] = -0.06*R13_13_d13 + DPT_3_16*R13_33_d13;

		in_out.r_Lfoot_cont_der[1][LEFT_HIP_PITCH][0] = -0.06*R13_11_d8 + 0.045*R13_21_d8 + DPT_3_16*R13_31_d8 + x13_1_d8;
		in_out.r_Lfoot_cont_der[1][LEFT_HIP_PITCH][1] = -0.06*R13_12_d8 + 0.045*R13_22_d8 + DPT_3_16*R13_32_d8 + x13_2_d8;
		in_out.r_Lfoot_cont_der[1][LEFT_HIP_PITCH][2] = -0.06*R13_13_d8 + 0.045*R13_23_d8 + DPT_3_16*R13_33_d8 + x13_3_d8;

		in_out.r_Lfoot_cont_der[1][LEFT_HIP_ROLL][0] = -0.06*R13_11_d9 + 0.045*R13_21_d9 + DPT_3_16*R13_31_d9 + x13_1_d9;
		in_out.r_Lfoot_cont_der[1][LEFT_HIP_ROLL][1] = -0.06*R13_12_d9 + 0.045*R13_22_d9 + DPT_3_16*R13_32_d9 + x13_2_d9;
		in_out.r_Lfoot_cont_der[1][LEFT_HIP_ROLL][2] = -0.06*R13_13_d9 + 0.045*R13_23_d9 + DPT_3_16*R13_33_d9 + x13_3_d9;

		in_out.r_Lfoot_cont_der[1][LEFT_HIP_YAW][0] = -0.06*R13_11_d10 + 0.045*R13_21_d10 + DPT_3_16*R13_31_d10 + x13_1_d10;
		in_out.r_Lfoot_cont_der[1][LEFT_HIP_YAW][1] = -0.06*R13_12_d10 + 0.045*R13_22_d10 + DPT_3_16*R13_32_d10 + x13_2_d10;
		in_out.r_Lfoot_cont_der[1][LEFT_HIP_YAW][2] = -0.06*R13_13_d10 + 0.045*R13_23_d10 + DPT_3_16*R13_33_d10 + x13_3_d10;

		in_out.r_Lfoot_cont_der[1][LEFT_KNEE_PITCH][0] = -0.06*R13_11_d11 + 0.045*R13_21_d11 + DPT_3_16*R13_31_d11 + x13_1_d11;
		in_out.r_Lfoot_cont_der[1][LEFT_KNEE_PITCH][1] = -0.06*R13_12_d11 + 0.045*R13_22_d11 + DPT_3_16*R13_32_d11 + x13_2_d11;
		in_out.r_Lfoot_cont_der[1][LEFT_KNEE_PITCH][2] = -0.06*R13_13_d11 + 0.045*R13_23_d11 + DPT_3_16*R13_33_d11 + x13_3_d11;

		in_out.r_Lfoot_cont_der[1][LEFT_FOOT_ROLL][0] = -0.06*R13_11_d12 + 0.045*R13_21_d12 + DPT_3_16*R13_31_d12;
		in_out.r_Lfoot_cont_der[1][LEFT_FOOT_ROLL][1] = -0.06*R13_12_d12 + 0.045*R13_22_d12 + DPT_3_16*R13_32_d12;
		in_out.r_Lfoot_cont_der[1][LEFT_FOOT_ROLL][2] = -0.06*R13_13_d12 + 0.045*R13_23_d12 + DPT_3_16*R13_33_d12;

		in_out.r_Lfoot_cont_der[1][LEFT_FOOT_PITCH][0] = -0.06*R13_11_d13 + DPT_3_16*R13_31_d13;
		in_out.r_Lfoot_cont_der[1][LEFT_FOOT_PITCH][1] = -0.06*R13_12_d13 + DPT_3_16*R13_32_d13;
		in_out.r_Lfoot_cont_der[1][LEFT_FOOT_PITCH][2] = -0.06*R13_13_d13 + DPT_3_16*R13_33_d13;

		in_out.r_Lfoot_cont_der[2][LEFT_HIP_PITCH][0] = -0.045*R13_21_d8 + 0.08*R13_11_d8 + DPT_3_16*R13_31_d8 + x13_1_d8;
		in_out.r_Lfoot_cont_der[2][LEFT_HIP_PITCH][1] = -0.045*R13_22_d8 + 0.08*R13_12_d8 + DPT_3_16*R13_32_d8 + x13_2_d8;
		in_out.r_Lfoot_cont_der[2][LEFT_HIP_PITCH][2] = -0.045*R13_23_d8 + 0.08*R13_13_d8 + DPT_3_16*R13_33_d8 + x13_3_d8;

		in_out.r_Lfoot_cont_der[2][LEFT_HIP_ROLL][0] = -0.045*R13_21_d9 + 0.08*R13_11_d9 + DPT_3_16*R13_31_d9 + x13_1_d9;
		in_out.r_Lfoot_cont_der[2][LEFT_HIP_ROLL][1] = -0.045*R13_22_d9 + 0.08*R13_12_d9 + DPT_3_16*R13_32_d9 + x13_2_d9;
		in_out.r_Lfoot_cont_der[2][LEFT_HIP_ROLL][2] = -0.045*R13_23_d9 + 0.08*R13_13_d9 + DPT_3_16*R13_33_d9 + x13_3_d9;

		in_out.r_Lfoot_cont_der[2][LEFT_HIP_YAW][0] = -0.045*R13_21_d10 + 0.08*R13_11_d10 + DPT_3_16*R13_31_d10 + x13_1_d10;
		in_out.r_Lfoot_cont_der[2][LEFT_HIP_YAW][1] = -0.045*R13_22_d10 + 0.08*R13_12_d10 + DPT_3_16*R13_32_d10 + x13_2_d10;
		in_out.r_Lfoot_cont_der[2][LEFT_HIP_YAW][2] = -0.045*R13_23_d10 + 0.08*R13_13_d10 + DPT_3_16*R13_33_d10 + x13_3_d10;

		in_out.r_Lfoot_cont_der[2][LEFT_KNEE_PITCH][0] = -0.045*R13_21_d11 + 0.08*R13_11_d11 + DPT_3_16*R13_31_d11 + x13_1_d11;
		in_out.r_Lfoot_cont_der[2][LEFT_KNEE_PITCH][1] = -0.045*R13_22_d11 + 0.08*R13_12_d11 + DPT_3_16*R13_32_d11 + x13_2_d11;
		in_out.r_Lfoot_cont_der[2][LEFT_KNEE_PITCH][2] = -0.045*R13_23_d11 + 0.08*R13_13_d11 + DPT_3_16*R13_33_d11 + x13_3_d11;

		in_out.r_Lfoot_cont_der[2][LEFT_FOOT_ROLL][0] = -0.045*R13_21_d12 + 0.08*R13_11_d12 + DPT_3_16*R13_31_d12;
		in_out.r_Lfoot_cont_der[2][LEFT_FOOT_ROLL][1] = -0.045*R13_22_d12 + 0.08*R13_12_d12 + DPT_3_16*R13_32_d12;
		in_out.r_Lfoot_cont_der[2][LEFT_FOOT_ROLL][2] = -0.045*R13_23_d12 + 0.08*R13_13_d12 + DPT_3_16*R13_33_d12;

		in_out.r_Lfoot_cont_der[2][LEFT_FOOT_PITCH][0] = 0.08*R13_11_d13 + DPT_3_16*R13_31_d13;
		in_out.r_Lfoot_cont_der[2][LEFT_FOOT_PITCH][1] = 0.08*R13_12_d13 + DPT_3_16*R13_32_d13;
		in_out.r_Lfoot_cont_der[2][LEFT_FOOT_PITCH][2] = 0.08*R13_13_d13 + DPT_3_16*R13_33_d13;

		in_out.r_Lfoot_cont_der[3][LEFT_HIP_PITCH][0] = 0.045*R13_21_d8 + 0.08*R13_11_d8 + DPT_3_16*R13_31_d8 + x13_1_d8;
		in_out.r_Lfoot_cont_der[3][LEFT_HIP_PITCH][1] = 0.045*R13_22_d8 + 0.08*R13_12_d8 + DPT_3_16*R13_32_d8 + x13_2_d8;
		in_out.r_Lfoot_cont_der[3][LEFT_HIP_PITCH][2] = 0.045*R13_23_d8 + 0.08*R13_13_d8 + DPT_3_16*R13_33_d8 + x13_3_d8;

		in_out.r_Lfoot_cont_der[3][LEFT_HIP_ROLL][0] = 0.045*R13_21_d9 + 0.08*R13_11_d9 + DPT_3_16*R13_31_d9 + x13_1_d9;
		in_out.r_Lfoot_cont_der[3][LEFT_HIP_ROLL][1] = 0.045*R13_22_d9 + 0.08*R13_12_d9 + DPT_3_16*R13_32_d9 + x13_2_d9;
		in_out.r_Lfoot_cont_der[3][LEFT_HIP_ROLL][2] = 0.045*R13_23_d9 + 0.08*R13_13_d9 + DPT_3_16*R13_33_d9 + x13_3_d9;

		in_out.r_Lfoot_cont_der[3][LEFT_HIP_YAW][0] = 0.045*R13_21_d10 + 0.08*R13_11_d10 + DPT_3_16*R13_31_d10 + x13_1_d10;
		in_out.r_Lfoot_cont_der[3][LEFT_HIP_YAW][1] = 0.045*R13_22_d10 + 0.08*R13_12_d10 + DPT_3_16*R13_32_d10 + x13_2_d10;
		in_out.r_Lfoot_cont_der[3][LEFT_HIP_YAW][2] = 0.045*R13_23_d10 + 0.08*R13_13_d10 + DPT_3_16*R13_33_d10 + x13_3_d10;

		in_out.r_Lfoot_cont_der[3][LEFT_KNEE_PITCH][0] = 0.045*R13_21_d11 + 0.08*R13_11_d11 + DPT_3_16*R13_31_d11 + x13_1_d11;
		in_out.r_Lfoot_cont_der[3][LEFT_KNEE_PITCH][1] = 0.045*R13_22_d11 + 0.08*R13_12_d11 + DPT_3_16*R13_32_d11 + x13_2_d11;
		in_out.r_Lfoot_cont_der[3][LEFT_KNEE_PITCH][2] = 0.045*R13_23_d11 + 0.08*R13_13_d11 + DPT_3_16*R13_33_d11 + x13_3_d11;

		in_out.r_Lfoot_cont_der[3][LEFT_FOOT_ROLL][0] = 0.045*R13_21_d12 + 0.08*R13_11_d12 + DPT_3_16*R13_31_d12;
		in_out.r_Lfoot_cont_der[3][LEFT_FOOT_ROLL][1] = 0.045*R13_22_d12 + 0.08*R13_12_d12 + DPT_3_16*R13_32_d12;
		in_out.r_Lfoot_cont_der[3][LEFT_FOOT_ROLL][2] = 0.045*R13_23_d12 + 0.08*R13_13_d12 + DPT_3_16*R13_33_d12;

		in_out.r_Lfoot_cont_der[3][LEFT_FOOT_PITCH][0] = 0.08*R13_11_d13 + DPT_3_16*R13_31_d13;
		in_out.r_Lfoot_cont_der[3][LEFT_FOOT_PITCH][1] = 0.08*R13_12_d13 + DPT_3_16*R13_32_d13;
		in_out.r_Lfoot_cont_der[3][LEFT_FOOT_PITCH][2] = 0.08*R13_13_d13 + DPT_3_16*R13_33_d13;
	}

	// feet absolute orientation
	in_out.Rfoot_or[0] = R7_11;
	in_out.Rfoot_or[1] = R7_12;
	in_out.Rfoot_or[2] = R7_13;
	in_out.Rfoot_or[3] = R7_21;
	in_out.Rfoot_or[4] = R7_22;
	in_out.Rfoot_or[5] = R7_23;
	in_out.Rfoot_or[6] = R7_31;
	in_out.Rfoot_or[7] = R7_32;
	in_out.Rfoot_or[8] = R7_33;

	in_out.Lfoot_or[0] = R13_11;
	in_out.Lfoot_or[1] = R13_12;
	in_out.Lfoot_or[2] = R13_13;
	in_out.Lfoot_or[3] = R13_21;
	in_out.Lfoot_or[4] = R13_22;
	in_out.Lfoot_or[5] = R13_23;
	in_out.Lfoot_or[6] = R13_31;
	in_out.Lfoot_or[7] = R13_32;
	in_out.Lfoot_or[8] = R13_33;

	// right foot absolute orientation jacobian
	if (flag_jacob)
	{
		in_out.Rfoot_or_der[RIGHT_HIP_PITCH][0] = R7_11_d2;
		in_out.Rfoot_or_der[RIGHT_HIP_PITCH][1] = R7_12_d2;
		in_out.Rfoot_or_der[RIGHT_HIP_PITCH][2] = R7_13_d2;
		in_out.Rfoot_or_der[RIGHT_HIP_PITCH][3] = R7_21_d2;
		in_out.Rfoot_or_der[RIGHT_HIP_PITCH][4] = R7_22_d2;
		in_out.Rfoot_or_der[RIGHT_HIP_PITCH][5] = R7_23_d2;
		in_out.Rfoot_or_der[RIGHT_HIP_PITCH][6] = R7_31_d2;
		in_out.Rfoot_or_der[RIGHT_HIP_PITCH][7] = R7_32_d2;
		in_out.Rfoot_or_der[RIGHT_HIP_PITCH][8] = R7_33_d2;

		in_out.Rfoot_or_der[RIGHT_HIP_ROLL][0] = R7_11_d3;
		in_out.Rfoot_or_der[RIGHT_HIP_ROLL][1] = R7_12_d3;
		in_out.Rfoot_or_der[RIGHT_HIP_ROLL][2] = R7_13_d3;
		in_out.Rfoot_or_der[RIGHT_HIP_ROLL][3] = R7_21_d3;
		in_out.Rfoot_or_der[RIGHT_HIP_ROLL][4] = R7_22_d3;
		in_out.Rfoot_or_der[RIGHT_HIP_ROLL][5] = R7_23_d3;
		in_out.Rfoot_or_der[RIGHT_HIP_ROLL][6] = R7_31_d3;
		in_out.Rfoot_or_der[RIGHT_HIP_ROLL][7] = R7_32_d3;
		in_out.Rfoot_or_der[RIGHT_HIP_ROLL][8] = R7_33_d3;

		in_out.Rfoot_or_der[RIGHT_HIP_YAW][0] = R7_11_d4;
		in_out.Rfoot_or_der[RIGHT_HIP_YAW][1] = R7_12_d4;
		in_out.Rfoot_or_der[RIGHT_HIP_YAW][2] = R7_13_d4;
		in_out.Rfoot_or_der[RIGHT_HIP_YAW][3] = R7_21_d4;
		in_out.Rfoot_or_der[RIGHT_HIP_YAW][4] = R7_22_d4;
		in_out.Rfoot_or_der[RIGHT_HIP_YAW][5] = R7_23_d4;
		in_out.Rfoot_or_der[RIGHT_HIP_YAW][6] = R7_31_d4;
		in_out.Rfoot_or_der[RIGHT_HIP_YAW][7] = R7_32_d4;
		in_out.Rfoot_or_der[RIGHT_HIP_YAW][8] = R7_33_d4;

		in_out.Rfoot_or_der[RIGHT_KNEE_PITCH][0] = R7_11_d5;
		in_out.Rfoot_or_der[RIGHT_KNEE_PITCH][1] = R7_12_d5;
		in_out.Rfoot_or_der[RIGHT_KNEE_PITCH][2] = R7_13_d5;
		in_out.Rfoot_or_der[RIGHT_KNEE_PITCH][3] = R7_21_d5;
		in_out.Rfoot_or_der[RIGHT_KNEE_PITCH][4] = R7_22_d5;
		in_out.Rfoot_or_der[RIGHT_KNEE_PITCH][5] = R7_23_d5;
		in_out.Rfoot_or_der[RIGHT_KNEE_PITCH][6] = R7_31_d5;
		in_out.Rfoot_or_der[RIGHT_KNEE_PITCH][7] = R7_32_d5;
		in_out.Rfoot_or_der[RIGHT_KNEE_PITCH][8] = R7_33_d5;

		in_out.Rfoot_or_der[RIGHT_FOOT_ROLL][0] = R7_11_d6;
		in_out.Rfoot_or_der[RIGHT_FOOT_ROLL][1] = R7_12_d6;
		in_out.Rfoot_or_der[RIGHT_FOOT_ROLL][2] = R7_13_d6;
		in_out.Rfoot_or_der[RIGHT_FOOT_ROLL][3] = R7_21_d6;
		in_out.Rfoot_or_der[RIGHT_FOOT_ROLL][4] = R7_22_d6;
		in_out.Rfoot_or_der[RIGHT_FOOT_ROLL][5] = R7_23_d6;
		in_out.Rfoot_or_der[RIGHT_FOOT_ROLL][6] = R7_31_d6;
		in_out.Rfoot_or_der[RIGHT_FOOT_ROLL][7] = R7_32_d6;
		in_out.Rfoot_or_der[RIGHT_FOOT_ROLL][8] = R7_33_d6;

		in_out.Rfoot_or_der[RIGHT_FOOT_PITCH][0] = R7_11_d7;
		in_out.Rfoot_or_der[RIGHT_FOOT_PITCH][1] = R7_12_d7;
		in_out.Rfoot_or_der[RIGHT_FOOT_PITCH][2] = R7_13_d7;
		in_out.Rfoot_or_der[RIGHT_FOOT_PITCH][6] = R7_31_d7;
		in_out.Rfoot_or_der[RIGHT_FOOT_PITCH][7] = R7_32_d7;
		in_out.Rfoot_or_der[RIGHT_FOOT_PITCH][8] = R7_33_d7;
	}

	// left foot absolute orientation jacobian
	if (flag_jacob)
	{
		in_out.Lfoot_or_der[LEFT_HIP_PITCH][0] = R13_11_d8;
		in_out.Lfoot_or_der[LEFT_HIP_PITCH][1] = R13_12_d8;
		in_out.Lfoot_or_der[LEFT_HIP_PITCH][2] = R13_13_d8;
		in_out.Lfoot_or_der[LEFT_HIP_PITCH][3] = R13_21_d8;
		in_out.Lfoot_or_der[LEFT_HIP_PITCH][4] = R13_22_d8;
		in_out.Lfoot_or_der[LEFT_HIP_PITCH][5] = R13_23_d8;
		in_out.Lfoot_or_der[LEFT_HIP_PITCH][6] = R13_31_d8;
		in_out.Lfoot_or_der[LEFT_HIP_PITCH][7] = R13_32_d8;
		in_out.Lfoot_or_der[LEFT_HIP_PITCH][8] = R13_33_d8;

		in_out.Lfoot_or_der[LEFT_HIP_ROLL][0] = R13_11_d9;
		in_out.Lfoot_or_der[LEFT_HIP_ROLL][1] = R13_12_d9;
		in_out.Lfoot_or_der[LEFT_HIP_ROLL][2] = R13_13_d9;
		in_out.Lfoot_or_der[LEFT_HIP_ROLL][3] = R13_21_d9;
		in_out.Lfoot_or_der[LEFT_HIP_ROLL][4] = R13_22_d9;
		in_out.Lfoot_or_der[LEFT_HIP_ROLL][5] = R13_23_d9;
		in_out.Lfoot_or_der[LEFT_HIP_ROLL][6] = R13_31_d9;
		in_out.Lfoot_or_der[LEFT_HIP_ROLL][7] = R13_32_d9;
		in_out.Lfoot_or_der[LEFT_HIP_ROLL][8] = R13_33_d9;

		in_out.Lfoot_or_der[LEFT_HIP_YAW][0] = R13_11_d10;
		in_out.Lfoot_or_der[LEFT_HIP_YAW][1] = R13_12_d10;
		in_out.Lfoot_or_der[LEFT_HIP_YAW][2] = R13_13_d10;
		in_out.Lfoot_or_der[LEFT_HIP_YAW][3] = R13_21_d10;
		in_out.Lfoot_or_der[LEFT_HIP_YAW][4] = R13_22_d10;
		in_out.Lfoot_or_der[LEFT_HIP_YAW][5] = R13_23_d10;
		in_out.Lfoot_or_der[LEFT_HIP_YAW][6] = R13_31_d10;
		in_out.Lfoot_or_der[LEFT_HIP_YAW][7] = R13_32_d10;
		in_out.Lfoot_or_der[LEFT_HIP_YAW][8] = R13_33_d10;

		in_out.Lfoot_or_der[LEFT_KNEE_PITCH][0] = R13_11_d11;
		in_out.Lfoot_or_der[LEFT_KNEE_PITCH][1] = R13_12_d11;
		in_out.Lfoot_or_der[LEFT_KNEE_PITCH][2] = R13_13_d11;
		in_out.Lfoot_or_der[LEFT_KNEE_PITCH][3] = R13_21_d11;
		in_out.Lfoot_or_der[LEFT_KNEE_PITCH][4] = R13_22_d11;
		in_out.Lfoot_or_der[LEFT_KNEE_PITCH][5] = R13_23_d11;
		in_out.Lfoot_or_der[LEFT_KNEE_PITCH][6] = R13_31_d11;
		in_out.Lfoot_or_der[LEFT_KNEE_PITCH][7] = R13_32_d11;
		in_out.Lfoot_or_der[LEFT_KNEE_PITCH][8] = R13_33_d11;

		in_out.Lfoot_or_der[LEFT_FOOT_ROLL][0] = R13_11_d12;
		in_out.Lfoot_or_der[LEFT_FOOT_ROLL][1] = R13_12_d12;
		in_out.Lfoot_or_der[LEFT_FOOT_ROLL][2] = R13_13_d12;
		in_out.Lfoot_or_der[LEFT_FOOT_ROLL][3] = R13_21_d12;
		in_out.Lfoot_or_der[LEFT_FOOT_ROLL][4] = R13_22_d12;
		in_out.Lfoot_or_der[LEFT_FOOT_ROLL][5] = R13_23_d12;
		in_out.Lfoot_or_der[LEFT_FOOT_ROLL][6] = R13_31_d12;
		in_out.Lfoot_or_der[LEFT_FOOT_ROLL][7] = R13_32_d12;
		in_out.Lfoot_or_der[LEFT_FOOT_ROLL][8] = R13_33_d12;

		in_out.Lfoot_or_der[LEFT_FOOT_PITCH][0] = R13_11_d13;
		in_out.Lfoot_or_der[LEFT_FOOT_PITCH][1] = R13_12_d13;
		in_out.Lfoot_or_der[LEFT_FOOT_PITCH][2] = R13_13_d13;
		in_out.Lfoot_or_der[LEFT_FOOT_PITCH][6] = R13_31_d13;
		in_out.Lfoot_or_der[LEFT_FOOT_PITCH][7] = R13_32_d13;
		in_out.Lfoot_or_der[LEFT_FOOT_PITCH][8] = R13_33_d13;
	}

	// right foot orientation matrix as angles [rad]
	in_out.theta_Rfoot[0] = atan2(R7_23, R7_33);
	in_out.theta_Rfoot[1] = atan2(-R7_13, sqrt(R7_11*R7_11 + R7_12*R7_12));
	in_out.theta_Rfoot[2] = atan2(R7_12, R7_11);

	// left foot orientation matrix as angles [rad]
	in_out.theta_Lfoot[0] = atan2(R13_23, R13_33);
	in_out.theta_Lfoot[1] = atan2(-R13_13, sqrt(R13_11*R13_11 + R13_12*R13_12));
	in_out.theta_Lfoot[2] = atan2(R13_12, R13_11);

	c_y_Rfoot = cos(in_out.theta_Rfoot[1]);
	c_y_Lfoot = cos(in_out.theta_Lfoot[1]);
	c_z_Rfoot = cos(in_out.theta_Rfoot[2]);
	c_z_Lfoot = cos(in_out.theta_Lfoot[2]);

	s_y_Rfoot = sin(in_out.theta_Rfoot[1]);
	s_y_Lfoot = sin(in_out.theta_Lfoot[1]);
	s_z_Rfoot = sin(in_out.theta_Rfoot[2]);
	s_z_Lfoot = sin(in_out.theta_Lfoot[2]);

	if ((!c_y_Rfoot) || (!c_y_Lfoot))
	{
		return;
	}

	inv_c_y_Rfoot = 1.0 / c_y_Rfoot;
	inv_c_y_Lfoot = 1.0 / c_y_Lfoot;

	// right foot orientation angle derivatives [rad/s]
	in_out.omega_Rfoot[0] = inv_c_y_Rfoot * (c_z_Rfoot*om7_1 + s_z_Rfoot*om7_2);
	in_out.omega_Rfoot[1] = c_z_Rfoot*om7_2 - s_z_Rfoot*om7_1;
	in_out.omega_Rfoot[2] = inv_c_y_Rfoot * s_y_Rfoot * (s_z_Rfoot*om7_2 + c_z_Rfoot*om7_1) + om7_3;

	// left foot orientation angle derivatives [rad/s]
	in_out.omega_Lfoot[0] = inv_c_y_Lfoot * (c_z_Lfoot*om13_1 + s_z_Lfoot*om13_2);
	in_out.omega_Lfoot[1] = c_z_Lfoot*om13_2 - s_z_Lfoot*om13_1;
	in_out.omega_Lfoot[2] = inv_c_y_Lfoot * s_y_Lfoot * (s_z_Lfoot*om13_2 + c_z_Lfoot*om13_1) + om13_3;

	// right wrist absolute position
	in_out.r_Rwrist[0] = -0.02*R20_11 - 0.005*R20_21 - 0.225*R20_31 + x20_1;
	in_out.r_Rwrist[1] = -0.02*R20_12 - 0.005*R20_22 - 0.225*R20_32 + x20_2;
	in_out.r_Rwrist[2] = -0.02*R20_13 - 0.005*R20_23 - 0.225*R20_33 + x20_3;

	// right wrist absolute velocity
	in_out.rp_Rwrist[0] = om20_2*(-0.02*R20_13 - 0.005*R20_23 - 0.225*R20_33) - om20_3*(-0.02*R20_12 - 0.005*R20_22 - 0.225*R20_32) + xp20_1;
	in_out.rp_Rwrist[1] = -om20_1*(-0.02*R20_13 - 0.005*R20_23 - 0.225*R20_33) + om20_3*(-0.02*R20_11 - 0.005*R20_21 - 0.225*R20_31) + xp20_2;
	in_out.rp_Rwrist[2] = om20_1*(-0.02*R20_12 - 0.005*R20_22 - 0.225*R20_32) - om20_2*(-0.02*R20_11 - 0.005*R20_21 - 0.225*R20_31) + xp20_3;

	// right wrist jacobian
	if (flag_jacob)
	{
		in_out.r_Rwrist_der[TORSO_ROLL][0] = -0.005*R20_21_d14 - 0.02*R20_11_d14 - 0.225*R20_31_d14 + x20_1_d14;
		in_out.r_Rwrist_der[TORSO_ROLL][1] = -0.005*R20_22_d14 - 0.02*R20_12_d14 - 0.225*R20_32_d14 + x20_2_d14;
		in_out.r_Rwrist_der[TORSO_ROLL][2] = -0.005*R20_23_d14 - 0.02*R20_13_d14 - 0.225*R20_33_d14 + x20_3_d14;

		in_out.r_Rwrist_der[TORSO_PITCH][0] = -0.005*R20_21_d15 - 0.02*R20_11_d15 - 0.225*R20_31_d15 + x20_1_d15;
		in_out.r_Rwrist_der[TORSO_PITCH][1] = -0.005*R20_22_d15 - 0.02*R20_12_d15 - 0.225*R20_32_d15 + x20_2_d15;
		in_out.r_Rwrist_der[TORSO_PITCH][2] = -0.005*R20_23_d15 - 0.02*R20_13_d15 - 0.225*R20_33_d15 + x20_3_d15;

		in_out.r_Rwrist_der[TORSO_YAW][0] = -0.005*R20_21_d16 - 0.02*R20_11_d16 - 0.225*R20_31_d16 + x20_1_d16;
		in_out.r_Rwrist_der[TORSO_YAW][1] = -0.005*R20_22_d16 - 0.02*R20_12_d16 - 0.225*R20_32_d16 + x20_2_d16;
		in_out.r_Rwrist_der[TORSO_YAW][2] = -0.005*R20_23_d16 - 0.02*R20_13_d16 - 0.225*R20_33_d16 + x20_3_d16;

		in_out.r_Rwrist_der[RIGHT_SH_PITCH][0] = -0.005*R20_21_d17 - 0.02*R20_11_d17 - 0.225*R20_31_d17 + x20_1_d17;
		in_out.r_Rwrist_der[RIGHT_SH_PITCH][1] = -0.005*R20_22_d17 - 0.02*R20_12_d17 - 0.225*R20_32_d17 + x20_2_d17;
		in_out.r_Rwrist_der[RIGHT_SH_PITCH][2] = -0.005*R20_23_d17 - 0.02*R20_13_d17 - 0.225*R20_33_d17 + x20_3_d17;

		in_out.r_Rwrist_der[RIGHT_SH_ROLL][0] = -0.005*R20_21_d18 - 0.02*R20_11_d18 - 0.225*R20_31_d18 + x20_1_d18;
		in_out.r_Rwrist_der[RIGHT_SH_ROLL][1] = -0.005*R20_22_d18 - 0.02*R20_12_d18 - 0.225*R20_32_d18 + x20_2_d18;
		in_out.r_Rwrist_der[RIGHT_SH_ROLL][2] = -0.005*R20_23_d18 - 0.02*R20_13_d18 - 0.225*R20_33_d18 + x20_3_d18;

		in_out.r_Rwrist_der[RIGHT_SH_YAW][0] = -0.005*R20_21_d19 - 0.02*R20_11_d19 - 0.225*R20_31_d19;
		in_out.r_Rwrist_der[RIGHT_SH_YAW][1] = -0.005*R20_22_d19 - 0.02*R20_12_d19 - 0.225*R20_32_d19;
		in_out.r_Rwrist_der[RIGHT_SH_YAW][2] = -0.005*R20_23_d19 - 0.02*R20_13_d19 - 0.225*R20_33_d19;

		in_out.r_Rwrist_der[RIGHT_ELB_PITCH][0] = -0.02*R20_11_d20 - 0.225*R20_31_d20;
		in_out.r_Rwrist_der[RIGHT_ELB_PITCH][1] = -0.02*R20_12_d20 - 0.225*R20_32_d20;
		in_out.r_Rwrist_der[RIGHT_ELB_PITCH][2] = -0.02*R20_13_d20 - 0.225*R20_33_d20;
	}

	// left wrist absolute position
	in_out.r_Lwrist[0] = -0.02*R24_11 + 0.005*R24_21 - 0.225*R24_31 + x24_1;
	in_out.r_Lwrist[1] = -0.02*R24_12 + 0.005*R24_22 - 0.225*R24_32 + x24_2;
	in_out.r_Lwrist[2] = -0.02*R24_13 + 0.005*R24_23 - 0.225*R24_33 + x24_3;

	// left wrist absolute velocity
	in_out.rp_Lwrist[0] = om24_2*(-0.02*R24_13 + 0.005*R24_23 - 0.225*R24_33) - om24_3*(-0.02*R24_12 + 0.005*R24_22 - 0.225*R24_32) + xp24_1;
	in_out.rp_Lwrist[1] = -om24_1*(-0.02*R24_13 + 0.005*R24_23 - 0.225*R24_33) + om24_3*(-0.02*R24_11 + 0.005*R24_21 - 0.225*R24_31) + xp24_2;
	in_out.rp_Lwrist[2] = om24_1*(-0.02*R24_12 + 0.005*R24_22 - 0.225*R24_32) - om24_2*(-0.02*R24_11 + 0.005*R24_21 - 0.225*R24_31) + xp24_3;

	// left wrist jacobian
	if (flag_jacob)
	{
		in_out.r_Lwrist_der[TORSO_ROLL][0] = -0.02*R24_11_d14 - 0.225*R24_31_d14 + 0.005*R24_21_d14 + x24_1_d14;
		in_out.r_Lwrist_der[TORSO_ROLL][1] = -0.02*R24_12_d14 - 0.225*R24_32_d14 + 0.005*R24_22_d14 + x24_2_d14;
		in_out.r_Lwrist_der[TORSO_ROLL][2] = -0.02*R24_13_d14 - 0.225*R24_33_d14 + 0.005*R24_23_d14 + x24_3_d14;

		in_out.r_Lwrist_der[TORSO_PITCH][0] = -0.02*R24_11_d15 - 0.225*R24_31_d15 + 0.005*R24_21_d15 + x24_1_d15;
		in_out.r_Lwrist_der[TORSO_PITCH][1] = -0.02*R24_12_d15 - 0.225*R24_32_d15 + 0.005*R24_22_d15 + x24_2_d15;
		in_out.r_Lwrist_der[TORSO_PITCH][2] = -0.02*R24_13_d15 - 0.225*R24_33_d15 + 0.005*R24_23_d15 + x24_3_d15;

		in_out.r_Lwrist_der[TORSO_YAW][0] = -0.02*R24_11_d16 - 0.225*R24_31_d16 + 0.005*R24_21_d16 + x24_1_d16;
		in_out.r_Lwrist_der[TORSO_YAW][1] = -0.02*R24_12_d16 - 0.225*R24_32_d16 + 0.005*R24_22_d16 + x24_2_d16;
		in_out.r_Lwrist_der[TORSO_YAW][2] = -0.02*R24_13_d16 - 0.225*R24_33_d16 + 0.005*R24_23_d16 + x24_3_d16;

		in_out.r_Lwrist_der[LEFT_SH_PITCH][0] = -0.02*R24_11_d21 - 0.225*R24_31_d21 + 0.005*R24_21_d21 + x24_1_d21;
		in_out.r_Lwrist_der[LEFT_SH_PITCH][1] = -0.02*R24_12_d21 - 0.225*R24_32_d21 + 0.005*R24_22_d21 + x24_2_d21;
		in_out.r_Lwrist_der[LEFT_SH_PITCH][2] = -0.02*R24_13_d21 - 0.225*R24_33_d21 + 0.005*R24_23_d21 + x24_3_d21;

		in_out.r_Lwrist_der[LEFT_SH_ROLL][0] = -0.02*R24_11_d22 - 0.225*R24_31_d22 + 0.005*R24_21_d22 + x24_1_d22;
		in_out.r_Lwrist_der[LEFT_SH_ROLL][1] = -0.02*R24_12_d22 - 0.225*R24_32_d22 + 0.005*R24_22_d22 + x24_2_d22;
		in_out.r_Lwrist_der[LEFT_SH_ROLL][2] = -0.02*R24_13_d22 - 0.225*R24_33_d22 + 0.005*R24_23_d22 + x24_3_d22;

		in_out.r_Lwrist_der[LEFT_SH_YAW][0] = -0.02*R24_11_d23 - 0.225*R24_31_d23 + 0.005*R24_21_d23;
		in_out.r_Lwrist_der[LEFT_SH_YAW][1] = -0.02*R24_12_d23 - 0.225*R24_32_d23 + 0.005*R24_22_d23;
		in_out.r_Lwrist_der[LEFT_SH_YAW][2] = -0.02*R24_13_d23 - 0.225*R24_33_d23 + 0.005*R24_23_d23;

		in_out.r_Lwrist_der[LEFT_ELB_PITCH][0] = -0.02*R24_11_d24 - 0.225*R24_31_d24;
		in_out.r_Lwrist_der[LEFT_ELB_PITCH][1] = -0.02*R24_12_d24 - 0.225*R24_32_d24;
		in_out.r_Lwrist_der[LEFT_ELB_PITCH][2] = -0.02*R24_13_d24 - 0.225*R24_33_d24;
	}

	// wrists absolute orientation
	in_out.Rwrist_or[0] = R20_11;
	in_out.Rwrist_or[1] = R20_12;
	in_out.Rwrist_or[2] = R20_13;
	in_out.Rwrist_or[3] = R20_21;
	in_out.Rwrist_or[4] = R20_22;
	in_out.Rwrist_or[5] = R20_23;
	in_out.Rwrist_or[6] = R20_31;
	in_out.Rwrist_or[7] = R20_32;
	in_out.Rwrist_or[8] = R20_33;

	in_out.Lwrist_or[0] = R24_11;
	in_out.Lwrist_or[1] = R24_12;
	in_out.Lwrist_or[2] = R24_13;
	in_out.Lwrist_or[3] = R24_21;
	in_out.Lwrist_or[4] = R24_22;
	in_out.Lwrist_or[5] = R24_23;
	in_out.Lwrist_or[6] = R24_31;
	in_out.Lwrist_or[7] = R24_32;
	in_out.Lwrist_or[8] = R24_33;

	// right wrist absolute orientation jacobian
	if (flag_jacob)
	{
		in_out.Rwrist_or_der[TORSO_ROLL][0] = R20_11_d14;
		in_out.Rwrist_or_der[TORSO_ROLL][1] = R20_12_d14;
		in_out.Rwrist_or_der[TORSO_ROLL][2] = R20_13_d14;
		in_out.Rwrist_or_der[TORSO_ROLL][3] = R20_21_d14;
		in_out.Rwrist_or_der[TORSO_ROLL][4] = R20_22_d14;
		in_out.Rwrist_or_der[TORSO_ROLL][5] = R20_23_d14;
		in_out.Rwrist_or_der[TORSO_ROLL][6] = R20_31_d14;
		in_out.Rwrist_or_der[TORSO_ROLL][7] = R20_32_d14;
		in_out.Rwrist_or_der[TORSO_ROLL][8] = R20_33_d14;

		in_out.Rwrist_or_der[TORSO_PITCH][0] = R20_11_d15;
		in_out.Rwrist_or_der[TORSO_PITCH][1] = R20_12_d15;
		in_out.Rwrist_or_der[TORSO_PITCH][2] = R20_13_d15;
		in_out.Rwrist_or_der[TORSO_PITCH][3] = R20_21_d15;
		in_out.Rwrist_or_der[TORSO_PITCH][4] = R20_22_d15;
		in_out.Rwrist_or_der[TORSO_PITCH][5] = R20_23_d15;
		in_out.Rwrist_or_der[TORSO_PITCH][6] = R20_31_d15;
		in_out.Rwrist_or_der[TORSO_PITCH][7] = R20_32_d15;
		in_out.Rwrist_or_der[TORSO_PITCH][8] = R20_33_d15;

		in_out.Rwrist_or_der[TORSO_YAW][0] = R20_11_d16;
		in_out.Rwrist_or_der[TORSO_YAW][1] = R20_12_d16;
		in_out.Rwrist_or_der[TORSO_YAW][2] = R20_13_d16;
		in_out.Rwrist_or_der[TORSO_YAW][3] = R20_21_d16;
		in_out.Rwrist_or_der[TORSO_YAW][4] = R20_22_d16;
		in_out.Rwrist_or_der[TORSO_YAW][5] = R20_23_d16;
		in_out.Rwrist_or_der[TORSO_YAW][6] = R20_31_d16;
		in_out.Rwrist_or_der[TORSO_YAW][7] = R20_32_d16;
		in_out.Rwrist_or_der[TORSO_YAW][8] = R20_33_d16;

		in_out.Rwrist_or_der[RIGHT_SH_PITCH][0] = R20_11_d17;
		in_out.Rwrist_or_der[RIGHT_SH_PITCH][1] = R20_12_d17;
		in_out.Rwrist_or_der[RIGHT_SH_PITCH][2] = R20_13_d17;
		in_out.Rwrist_or_der[RIGHT_SH_PITCH][3] = R20_21_d17;
		in_out.Rwrist_or_der[RIGHT_SH_PITCH][4] = R20_22_d17;
		in_out.Rwrist_or_der[RIGHT_SH_PITCH][5] = R20_23_d17;
		in_out.Rwrist_or_der[RIGHT_SH_PITCH][6] = R20_31_d17;
		in_out.Rwrist_or_der[RIGHT_SH_PITCH][7] = R20_32_d17;
		in_out.Rwrist_or_der[RIGHT_SH_PITCH][8] = R20_33_d17;

		in_out.Rwrist_or_der[RIGHT_SH_ROLL][0] = R20_11_d18;
		in_out.Rwrist_or_der[RIGHT_SH_ROLL][1] = R20_12_d18;
		in_out.Rwrist_or_der[RIGHT_SH_ROLL][2] = R20_13_d18;
		in_out.Rwrist_or_der[RIGHT_SH_ROLL][3] = R20_21_d18;
		in_out.Rwrist_or_der[RIGHT_SH_ROLL][4] = R20_22_d18;
		in_out.Rwrist_or_der[RIGHT_SH_ROLL][5] = R20_23_d18;
		in_out.Rwrist_or_der[RIGHT_SH_ROLL][6] = R20_31_d18;
		in_out.Rwrist_or_der[RIGHT_SH_ROLL][7] = R20_32_d18;
		in_out.Rwrist_or_der[RIGHT_SH_ROLL][8] = R20_33_d18;

		in_out.Rwrist_or_der[RIGHT_SH_YAW][0] = R20_11_d19;
		in_out.Rwrist_or_der[RIGHT_SH_YAW][1] = R20_12_d19;
		in_out.Rwrist_or_der[RIGHT_SH_YAW][2] = R20_13_d19;
		in_out.Rwrist_or_der[RIGHT_SH_YAW][3] = R20_21_d19;
		in_out.Rwrist_or_der[RIGHT_SH_YAW][4] = R20_22_d19;
		in_out.Rwrist_or_der[RIGHT_SH_YAW][5] = R20_23_d19;
		in_out.Rwrist_or_der[RIGHT_SH_YAW][6] = R20_31_d19;
		in_out.Rwrist_or_der[RIGHT_SH_YAW][7] = R20_32_d19;
		in_out.Rwrist_or_der[RIGHT_SH_YAW][8] = R20_33_d19;

		in_out.Rwrist_or_der[RIGHT_ELB_PITCH][0] = R20_11_d20;
		in_out.Rwrist_or_der[RIGHT_ELB_PITCH][1] = R20_12_d20;
		in_out.Rwrist_or_der[RIGHT_ELB_PITCH][2] = R20_13_d20;
		in_out.Rwrist_or_der[RIGHT_ELB_PITCH][6] = R20_31_d20;
		in_out.Rwrist_or_der[RIGHT_ELB_PITCH][7] = R20_32_d20;
		in_out.Rwrist_or_der[RIGHT_ELB_PITCH][8] = R20_33_d20;
	}

	// left wrist absolute orientation jacobian
	if (flag_jacob)
	{
		in_out.Lwrist_or_der[TORSO_ROLL][0] = R24_11_d14;
		in_out.Lwrist_or_der[TORSO_ROLL][1] = R24_12_d14;
		in_out.Lwrist_or_der[TORSO_ROLL][2] = R24_13_d14;
		in_out.Lwrist_or_der[TORSO_ROLL][3] = R24_21_d14;
		in_out.Lwrist_or_der[TORSO_ROLL][4] = R24_22_d14;
		in_out.Lwrist_or_der[TORSO_ROLL][5] = R24_23_d14;
		in_out.Lwrist_or_der[TORSO_ROLL][6] = R24_31_d14;
		in_out.Lwrist_or_der[TORSO_ROLL][7] = R24_32_d14;
		in_out.Lwrist_or_der[TORSO_ROLL][8] = R24_33_d14;

		in_out.Lwrist_or_der[TORSO_PITCH][0] = R24_11_d15;
		in_out.Lwrist_or_der[TORSO_PITCH][1] = R24_12_d15;
		in_out.Lwrist_or_der[TORSO_PITCH][2] = R24_13_d15;
		in_out.Lwrist_or_der[TORSO_PITCH][3] = R24_21_d15;
		in_out.Lwrist_or_der[TORSO_PITCH][4] = R24_22_d15;
		in_out.Lwrist_or_der[TORSO_PITCH][5] = R24_23_d15;
		in_out.Lwrist_or_der[TORSO_PITCH][6] = R24_31_d15;
		in_out.Lwrist_or_der[TORSO_PITCH][7] = R24_32_d15;
		in_out.Lwrist_or_der[TORSO_PITCH][8] = R24_33_d15;

		in_out.Lwrist_or_der[TORSO_YAW][0] = R24_11_d16;
		in_out.Lwrist_or_der[TORSO_YAW][1] = R24_12_d16;
		in_out.Lwrist_or_der[TORSO_YAW][2] = R24_13_d16;
		in_out.Lwrist_or_der[TORSO_YAW][3] = R24_21_d16;
		in_out.Lwrist_or_der[TORSO_YAW][4] = R24_22_d16;
		in_out.Lwrist_or_der[TORSO_YAW][5] = R24_23_d16;
		in_out.Lwrist_or_der[TORSO_YAW][6] = R24_31_d16;
		in_out.Lwrist_or_der[TORSO_YAW][7] = R24_32_d16;
		in_out.Lwrist_or_der[TORSO_YAW][8] = R24_33_d16;

		in_out.Lwrist_or_der[LEFT_SH_PITCH][0] = R24_11_d21;
		in_out.Lwrist_or_der[LEFT_SH_PITCH][1] = R24_12_d21;
		in_out.Lwrist_or_der[LEFT_SH_PITCH][2] = R24_13_d21;
		in_out.Lwrist_or_der[LEFT_SH_PITCH][3] = R24_21_d21;
		in_out.Lwrist_or_der[LEFT_SH_PITCH][4] = R24_22_d21;
		in_out.Lwrist_or_der[LEFT_SH_PITCH][5] = R24_23_d21;
		in_out.Lwrist_or_der[LEFT_SH_PITCH][6] = R24_31_d21;
		in_out.Lwrist_or_der[LEFT_SH_PITCH][7] = R24_32_d21;
		in_out.Lwrist_or_der[LEFT_SH_PITCH][8] = R24_33_d21;

		in_out.Lwrist_or_der[LEFT_SH_ROLL][0] = R24_11_d22;
		in_out.Lwrist_or_der[LEFT_SH_ROLL][1] = R24_12_d22;
		in_out.Lwrist_or_der[LEFT_SH_ROLL][2] = R24_13_d22;
		in_out.Lwrist_or_der[LEFT_SH_ROLL][3] = R24_21_d22;
		in_out.Lwrist_or_der[LEFT_SH_ROLL][4] = R24_22_d22;
		in_out.Lwrist_or_der[LEFT_SH_ROLL][5] = R24_23_d22;
		in_out.Lwrist_or_der[LEFT_SH_ROLL][6] = R24_31_d22;
		in_out.Lwrist_or_der[LEFT_SH_ROLL][7] = R24_32_d22;
		in_out.Lwrist_or_der[LEFT_SH_ROLL][8] = R24_33_d22;

		in_out.Lwrist_or_der[LEFT_SH_YAW][0] = R24_11_d23;
		in_out.Lwrist_or_der[LEFT_SH_YAW][1] = R24_12_d23;
		in_out.Lwrist_or_der[LEFT_SH_YAW][2] = R24_13_d23;
		in_out.Lwrist_or_der[LEFT_SH_YAW][3] = R24_21_d23;
		in_out.Lwrist_or_der[LEFT_SH_YAW][4] = R24_22_d23;
		in_out.Lwrist_or_der[LEFT_SH_YAW][5] = R24_23_d23;
		in_out.Lwrist_or_der[LEFT_SH_YAW][6] = R24_31_d23;
		in_out.Lwrist_or_der[LEFT_SH_YAW][7] = R24_32_d23;
		in_out.Lwrist_or_der[LEFT_SH_YAW][8] = R24_33_d23;

		in_out.Lwrist_or_der[LEFT_ELB_PITCH][0] = R24_11_d24;
		in_out.Lwrist_or_der[LEFT_ELB_PITCH][1] = R24_12_d24;
		in_out.Lwrist_or_der[LEFT_ELB_PITCH][2] = R24_13_d24;
		in_out.Lwrist_or_der[LEFT_ELB_PITCH][6] = R24_31_d24;
		in_out.Lwrist_or_der[LEFT_ELB_PITCH][7] = R24_32_d24;
		in_out.Lwrist_or_der[LEFT_ELB_PITCH][8] = R24_33_d24;
	}

	// waist orientation matrix as angles [rad]
	in_out.theta_waist[0] = atan2(R1_23, R1_33);
	in_out.theta_waist[1] = atan2(-R1_13, sqrt(R1_11*R1_11 + R1_12*R1_12));
	in_out.theta_waist[2] = atan2(R1_12, R1_11);

	// torso orientation matrix as angles [rad]
	in_out.theta_torso[0] = atan2(R16_23, R16_33);
	in_out.theta_torso[1] = atan2(-R16_13, sqrt(R16_11*R16_11 + R16_12*R16_12));
	in_out.theta_torso[2] = atan2(R16_12, R16_11);

	c_y_waist = cos(in_out.theta_waist[1]);
	c_y_torso = cos(in_out.theta_torso[1]);
	c_z_waist = cos(in_out.theta_waist[2]);
	c_z_torso = cos(in_out.theta_torso[2]);

	s_y_waist = sin(in_out.theta_waist[1]);
	s_y_torso = sin(in_out.theta_torso[1]);
	s_z_waist = sin(in_out.theta_waist[2]);
	s_z_torso = sin(in_out.theta_torso[2]);

	if ((!c_y_waist) || (!c_y_torso))
	{
		return;
	}

	inv_c_y_waist = 1.0 / c_y_waist;
	inv_c_y_torso = 1.0 / c_y_torso;

	// waist orientation angle derivatives [rad/s]
	in_out.omega_waist[0] = inv_c_y_waist * (c_z_waist*om1_1 + s_z_waist*om1_2);
	in_out.omega_waist[1] = c_z_waist*om1_2 - s_z_waist*om1_1;
	in_out.omega_waist[2] = inv_c_y_waist * s_y_waist * (s_z_waist*om1_2 + c_z_waist*om1_1) + om1_3;

	// torso orientation angle derivatives [rad/s]
	in_out.omega_torso[0] = inv_c_y_torso * (c_z_torso*om16_1 + s_z_torso*om16_2);
	in_out.omega_torso[1] = c_z_torso*om16_2 - s_z_torso*om16_1;
	in_out.omega_torso[2] = inv_c_y_torso * s_y_torso * (s_z_torso*om16_2 + c_z_torso*om16_1) + om16_3;
}
