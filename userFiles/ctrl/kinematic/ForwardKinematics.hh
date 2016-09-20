/*! 
 * \author Nicolas Van der Noot
 * \file ForwardKinematics.hh
 * \brief ForwardKinematics class
 */

#ifndef _FORWARD_KINEMATICS_HH_
#define _FORWARD_KINEMATICS_HH_

#include "Computation.hh"
#include "CtrlInputs.hh"
#include "coman_properties.hh"
#include "WalkStates.hh"
#include "MainState.hh"

#define NB_CONT_IMP 4 ///< number of contact per foot for impedance-based controller

// forward declaration
class InverseKin;
class InverseKinLeg;

/*! \brief inputs and outputs of the main kinematics function
 */
class KinematicsInOut
{
	public:
		KinematicsInOut(int nb_mot);
		~KinematicsInOut();

	public:
		// -- inputs -- //

		// IMU
		double IMU_Orientation[9];  ///< IMU rotation matrix [-] 
		double IMU_Angular_Rate[3]; ///< IMU angular rate [rad/s]

		// motor joints
		int nb_mot; ///< number of motors

		std::vector<double> q_mot;  ///< position (motor) [rad]
		std::vector<double> qd_mot; ///< velocity (motor) [rad/s]


		// -- outputs -- //

		// center of mass
		double r_COM[3]; ///< global com absolute position [m]
		double rp_COM[3]; ///< global com absolute velocity [m/s]

		// feet kinematics (contact point with the ground)
		double r_Rfoot[3]; ///< right foot absolute position [m]
		double r_Lfoot[3]; ///< left foot absolute position [m]

		double rp_Rfoot[3]; ///< right foot absolute velocity [m/s]
		double rp_Lfoot[3]; ///< left foot absolute velocity [m/s]

		// feet orientation matrix
		double Rfoot_or[9]; ///< right foot orientation [-]
		double Lfoot_or[9]; ///< left foot orientation [-]

		// wrists kinematics (end of ball)
		double r_Rwrist[3]; ///< right wrist absolute position [m]
		double r_Lwrist[3]; ///< left wrist absolute position [m]

		double rp_Rwrist[3]; ///< right wrist absolute velocity [m/s]
		double rp_Lwrist[3]; ///< left wrist absolute velocity [m/s]

		// wrists orientation matrix
		double Rwrist_or[9]; ///< right wrist orientation [-]
		double Lwrist_or[9]; ///< left wrist orientation [-]

		// rotation matrices as angles
		double theta_waist[3]; ///< waist orientation angles [rad]
		double theta_torso[3]; ///< torso orientation angles [rad]
		double theta_Rfoot[3]; ///< right foot orientation angles [rad]
		double theta_Lfoot[3]; ///< left foot orientation angles [rad]

		double omega_waist[3]; ///< waist orientation angle derivatives [rad/s]
		double omega_torso[3]; ///< torso orientation angle derivatives [rad/s]
		double omega_Rfoot[3]; ///< right foot orientation angle derivatives [rad/s]
		double omega_Lfoot[3]; ///< left foot orientation angle derivatives [rad/s]

		// jacobian related
		double r_COM_der[NB_MOTORS][3]; ///< jacobian of 'r_COM' [m/rad]

		double r_Rfoot_der[NB_MOTORS][3]; ///< jacobian of 'r_Rfoot' [m/rad]
		double r_Lfoot_der[NB_MOTORS][3]; ///< jacobian of 'r_Lfoot' [m/rad]

		double r_Rwrist_der[NB_MOTORS][3]; ///< jacobian of 'r_Rwrist' [m/rad]
		double r_Lwrist_der[NB_MOTORS][3]; ///< jacobian of 'r_Lwrist' [m/rad]

		double r_Rfoot_cont[NB_CONT_IMP][3]; ///< right foot four contacts for impendance controller [m]
		double r_Lfoot_cont[NB_CONT_IMP][3]; ///< left foot four contacts for impendance controller [m]
		double r_Rfoot_cont_der[NB_CONT_IMP][NB_MOTORS][3]; ///< jacobian of 'r_Rfoot_cont' [m/rad]
		double r_Lfoot_cont_der[NB_CONT_IMP][NB_MOTORS][3]; ///< jacobian of 'r_Lfoot_cont' [m/rad]

		double Rfoot_or_der[NB_MOTORS][9]; ///< jacobian of 'Rfoot_or' [-]
		double Lfoot_or_der[NB_MOTORS][9]; ///< jacobian of 'Lfoot_or' [-]

		double Rwrist_or_der[NB_MOTORS][9]; ///< jacobian of 'Rwrist_or' [-]
		double Lwrist_or_der[NB_MOTORS][9]; ///< jacobian of 'Lwrist_or' [-]

		/// get the number of contacts per foot
		int get_nb_cont_foot() const { return NB_CONT_IMP; }

		/// get the number of motors
		int get_nb_mot() const { return nb_mot; }
};

/*! \brief forward kinematics computation
 */
class ForwardKinematics: public Computation
{
	public:
		ForwardKinematics(CtrlInputs *inputs, CtrlOptions *options, MotorCtrlIndex *ctrl_index, WalkStates *ws);
		virtual ~ForwardKinematics();

		virtual void compute();

		double get_r_COM_Rfoot_yaw(int i)  const { return r_COM_Rfoot_yaw[i]; }
		double get_r_COM_Lfoot_yaw(int i)  const { return r_COM_Lfoot_yaw[i]; }
		double get_r_COM_feet_yaw(int i)   const { return r_COM_feet_yaw[i];  }

		double get_rp_COM_Rfoot_yaw(int i) const { return rp_COM_Rfoot_yaw[i]; }
		double get_rp_COM_Lfoot_yaw(int i) const { return rp_COM_Lfoot_yaw[i]; }
		double get_rp_COM_feet_yaw(int i)  const { return rp_COM_feet_yaw[i];  }

		double get_r_COM_Rfoot(int i)  const { return r_COM_Rfoot[i]; }
		double get_r_COM_Lfoot(int i)  const { return r_COM_Lfoot[i]; }

		double get_rp_COM(int i)  const { return rp_COM[i];  }

		void fill_gen_inputs(KinematicsInOut &in_out);
		void main_kinematics(KinematicsInOut &in_out);

		void set_flag_jacob(int value) { flag_jacob = value; }

		int get_fwd_kin_index(int ctrl_index);
		int get_ctrl_index(int fwd_kin_index);

		InverseKinLeg* get_r_leg_inverse() { return r_leg_inverse; }
		InverseKinLeg* get_l_leg_inverse() { return l_leg_inverse; }

		/// get gen_in_out
		KinematicsInOut& get_io() { return gen_in_out; }
		KinematicsInOut* get_io_ptr() { return &gen_in_out; }

	private:
		KinematicsInOut gen_in_out; ///< generic inputs and outputs

		WalkStates *ws; ///< walk states
		MainState *ms;  ///< main COMAN state

		// i/o vector IDs
		int RightHipPitch_id;  ///< right hip pitch joint
		int RightHipRoll_id;   ///< right hip roll joint
		int RightHipYaw_id;    ///< right hip yaw joint
		int RightKneePitch_id; ///< right knee pitch joint
		int RightFootRoll_id;  ///< right foot roll joint
		int RightFootPitch_id; ///< right foot pitch joint

		int LeftHipPitch_id;  ///< left hip pitch joint
		int LeftHipRoll_id;   ///< left hip roll joint
		int LeftHipYaw_id;    ///< left hip yaw joint
		int LeftKneePitch_id; ///< left knee pitch joint
		int LeftFootRoll_id;  ///< left foot roll joint
		int LeftFootPitch_id; ///< left foot pitch joint

		int TorsoRoll_id;  ///< torso roll joint
		int TorsoPitch_id; ///< torso pitch joint
		int TorsoYaw_id;   ///< torso yaw joint

		int RightShPitch_id;  ///< right shoulder pitch joint
		int RightShRoll_id;   ///< right shoulder roll joint
		int RightShYaw_id;    ///< right shoulder yaw joint
		int RightElbPitch_id; ///< right elbow pitch joint

		int LeftShPitch_id;  ///< left shoulder pitch joint
		int LeftShRoll_id;   ///< left shoulder roll joint
		int LeftShYaw_id;    ///< left shoulder yaw joint
		int LeftElbPitch_id; ///< left elbow pitch joint

		// flag
		int flag_jacob; ///< 1 to compute the jacobian quantities, 0 otherwise

		double r_COM_Rfoot[3]; ///< distance from the right foot to the COM [m]
		double r_COM_Lfoot[3]; ///< distance from the left foot to the COM [m]
		double r_COM_feet[3];  ///< distance from the middle of the feet to the COM [m]

		double rp_COM_Rfoot[3]; ///< velocity from the right foot to the COM [m/s]
		double rp_COM_Lfoot[3]; ///< velocity from the left foot to the COM [m/s]
		double rp_COM_feet[3];  ///< velocity from the middle of the feet to the COM [m/s]

		double r_COM_Rfoot_yaw[3]; ///< r_COM_Rfoot in the robot forward frame (yaw angle is zero) [m]
		double r_COM_Lfoot_yaw[3]; ///< r_COM_Lfoot in the robot forward frame (yaw angle is zero) [m]
		double r_COM_feet_yaw[3];  ///< r_COM_feet in the robot forward frame (yaw angle is zero) [m]

		double rp_COM_Rfoot_yaw[3]; ///< time derivative of r_COM_Rfoot_yaw [m/s]
		double rp_COM_Lfoot_yaw[3]; ///< time derivative of r_COM_Lfoot_yaw [m/s]
		double rp_COM_feet_yaw[3];  ///< time derivative of r_COM_feet_yaw [m/s]

		double rp_COM[3]; ///< global com absolute velocity [m/s]

		std::vector<InverseKin*> inverse_kin; ///< inverse kinematics module

		InverseKinLeg *r_leg_inverse; ///< right leg inverse kinematics computation
		InverseKinLeg *l_leg_inverse; ///< left leg inverse kinematics computation

		// functions prototypes
		void state_check();
		void foot_abs_or();
		void force_torque_inertial_frame();
		void torso_orientation();
		void com_feet();
};

#endif
