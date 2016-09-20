/*! 
 * \author Francois Heremans & Nicolas Van der Noot
 * \file ImpedanceCtrl.hh
 * \brief ImpedanceCtrl class
 */

#ifndef _IMPEDANCE_CTRL_HH_
#define _IMPEDANCE_CTRL_HH_

#include "Computation.hh"
#include "Matrix2D.hh"
#include "ForwardKinematics.hh"
#include "CtrlOutputs.hh"

/*! \brief impedance based controller
 */
class ImpedanceCtrl: public Computation
{
	public:
		ImpedanceCtrl(CtrlInputs *inputs, CtrlOptions *options, MotorCtrlIndex *ctrl_index, CtrlOutputs *outputs, ForwardKinematics *fwd_kin, int nb_cont, int nb_mot);
		~ImpedanceCtrl();

		virtual void compute();

		void set_r_COM_ref(int i, double value) { r_COM_ref[i] = value; }

		void set_r_COM_ref(double x, double y, double z);

		void apply_Qq_ref();
		void set_input_Qq_ref();

	protected:
		int nb_cont; ///< number of contacts
		int nb_mot;  ///< number of motors

		Matrix2D Ax; ///< Ax matrix for force distribution
		Matrix2D Ay; ///< Ay matrix for force distribution
		Matrix2D Az; ///< Az matrix for force distribution

		Matrix2D Ax_trans; ///< transpose matrix of 'Ax'
		Matrix2D Ay_trans; ///< transpose matrix of 'Ay'
		Matrix2D Az_trans; ///< transpose matrix of 'Az'

		Matrix2D Ax_mult; ///< multiplication matrix: 'Ax*Ax_trans'
		Matrix2D Ay_mult; ///< multiplication matrix: 'Ay*Ay_trans'
		Matrix2D Az_mult; ///< multiplication matrix: 'Az*Az_trans'

		Matrix2D Ax_mult_inv; ///< inverse matrix of 'Ax_mult'
		Matrix2D Ay_mult_inv; ///< inverse matrix of 'Ay_mult'
		Matrix2D Az_mult_inv; ///< inverse matrix of 'Az_mult'

		Matrix2D Ax_ps_inv; ///< pseudo inverse matrix of 'Ax'
		Matrix2D Ay_ps_inv; ///< pseudo inverse matrix of 'Ay'
		Matrix2D Az_ps_inv; ///< pseudo inverse matrix of 'Az'

		Matrix2D jacob_x; ///< x components of the jacobian
		Matrix2D jacob_y; ///< y components of the jacobian
		Matrix2D jacob_z; ///< z components of the jacobian

		Matrix2D fs; ///< individual contact forces

		ForwardKinematics *fwd_kin; ///< forward kinematics
		KinematicsInOut *io;        ///< I/O of the kinematics
		CtrlOutputs *outputs;       ///< outputs of the controller

		double r_COM_ref[3]; ///< COM position reference [m]

		double r_base[3];  ///< base position [m]
		double rp_base[3]; ///< base position derivative [m/s]

		double delta_com[3];     ///< com position from base [m]
		double delta_com_dot[3]; ///< com position derivative from base [m/s]
		double delta_com_int[3]; ///< integral of 'delta_com' [ms]
		double last_t_delta_com; ///< last time 'delta_com_int' was updated [s]

		double fp[3]; ///< contact force [N]

		double rp[2]; ///< planar position of the desired COP [m]

		double kp_x;  ///< P constant for PD control of the COM along the x axis
		double kp_y;  ///< P constant for PD control of the COM along the y axis
		double kp_z;  ///< P constant for PD control of the COM along the z axis

		double kd_x;  ///< D constant for PD control of the COM along the x axis
		double kd_y;  ///< D constant for PD control of the COM along the y axis
		double kd_z;  ///< D constant for PD control of the COM along the z axis

		double ki_x;  ///< I constant for PD control of the COM along the x axis
		double ki_y;  ///< I constant for PD control of the COM along the y axis
		double ki_z;  ///< I constant for PD control of the COM along the z axis

		double kp_trq; ///< P constant for the PD control of the basic torque output
		double kd_trq; ///< D constant for the PD control of the basic torque output

		double max_tor; ///< maximal torque [Nm]
		double thres_knee; ///< threshold position to prevent knee over-extension [rad]
		double thres_knee_gain; ///< gain to prevent knee over-extension [Nm/rad]

		int RightKneePitch_ctrl_id; ///< right knee pitch controller joint
		int LeftKneePitch_ctrl_id;  ///< left knee pitch controller joint

		int RightKneePitch_fwd_kin_id; ///< right knee pitch forward kinematics joint
		int LeftKneePitch_fwd_kin_id;  ///< left knee pitch forward kinematics joint

		// vectors with indexes coherent with 'fwd_kin_index'
		std::vector<double> trq_ref; ///< torque references [Nm]
		std::vector<double> pos_ref; ///< position references [rad]
		std::vector<int> ctrl_id;    ///< corresponding IDs for the controller
		std::vector<int> q_ctrl_id;  ///< IDs of the q_mot vector of the controller

		// function prototypes
		void inverse_matrices();
		void compute_GAF();
		void compute_force_distrib();
		void compute_tor();

		virtual void update_matrices() = 0;
		virtual void compute_base() = 0;
		virtual void compute_jacob() = 0;
};

#endif
