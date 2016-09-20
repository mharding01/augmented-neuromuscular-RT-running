/*! 
 * \author Nicolas Van der Noot
 * \file CtrlInputs.hh
 * \brief CtrlInputs class
 */

#ifndef _CTRL_INPUTS_HH_
#define _CTRL_INPUTS_HH_

#include "OptiInputs.hh"
#include "UserCtrl.hh"
#include "MotorCtrlIndex.hh"
#include "coman_properties.hh"
#include "CtrlOptions.hh"

/*! \brief Inputs of the controller
 */
class CtrlInputs
{
	public:
		CtrlInputs(UserCtrl *user_ctrl, MotorCtrlIndex *ctrl_index, CtrlOptions *options);
		~CtrlInputs();


		// -- get inputs -- //

		double get_t() { return t; }

		int get_nb_mot() const { return nb_mot; }

		double get_q(int mot_id)      const { return q[mot_id];      }
		double get_qd(int mot_id)     const { return qd[mot_id];     }
		double get_Qq(int mot_id)     const { return Qq[mot_id];     }
		double get_q_mot(int mot_id)  const { return q_mot[mot_id];  }
		double get_qd_mot(int mot_id) const { return qd_mot[mot_id]; }

		double get_q_ref(int mot_id)  const { return q_ref[mot_id];  }
		double get_Qq_ref(int mot_id) const { return Qq_ref[mot_id]; }

		// get motor values for controller indexes
		double get_ctrl_q(int ctrl_id)      const { return q[ctrl_index->get_inv_index(ctrl_id)];      }
		double get_ctrl_qd(int ctrl_id)     const { return qd[ctrl_index->get_inv_index(ctrl_id)];     }
		double get_ctrl_Qq(int ctrl_id)     const { return Qq[ctrl_index->get_inv_index(ctrl_id)];     }
		double get_ctrl_q_mot(int ctrl_id)  const { return q_mot[ctrl_index->get_inv_index(ctrl_id)];  }
		double get_ctrl_qd_mot(int ctrl_id) const { return qd_mot[ctrl_index->get_inv_index(ctrl_id)]; }

		double get_F_Rfoot(int axis) const { return F_Rfoot[axis]; }
		double get_F_Lfoot(int axis) const { return F_Lfoot[axis]; }
		double get_T_Rfoot(int axis) const { return T_Rfoot[axis]; }
		double get_T_Lfoot(int axis) const { return T_Lfoot[axis]; }

		double get_F_Rfoot_IF(int axis) const { return F_Rfoot_IF[axis]; }
		double get_F_Lfoot_IF(int axis) const { return F_Lfoot_IF[axis]; }
		double get_F_Rtoe_IF(int axis) const { return F_Rtoe_IF[axis]; }
		double get_F_Ltoe_IF(int axis) const { return F_Ltoe_IF[axis]; }
		double get_T_Rfoot_IF(int axis) const { return T_Rfoot_IF[axis]; }
		double get_T_Lfoot_IF(int axis) const { return T_Lfoot_IF[axis]; }

		double get_IMU_Orientation(int comp_id) const { return IMU_Orientation[comp_id]; }
		double get_IMU_Angular_Rate(int axis)   const { return IMU_Angular_Rate[axis];   }
		double get_IMU_Acceleration(int axis)   const { return IMU_Acceleration[axis];   }

		double get_Fz_feet(int foot_id)    const { return Fz_feet[foot_id];    }
		double get_mean_Fz_feet(int index) const { return mean_Fz_feet[index]; }
		double get_Fz_toe(int foot_id)    const { return Fz_toe[foot_id];    }
		double get_mean_Fz_toe(int index) const { return mean_Fz_toe[index]; }

		double get_theta_waist(int axis) const { return theta_waist[axis]; }		
		double get_theta_torso(int axis) const { return theta_torso[axis]; }
		double get_theta_Rfoot(int axis) const { return theta_Rfoot[axis]; }		
		double get_theta_Lfoot(int axis) const { return theta_Lfoot[axis]; }

		double get_omega_waist(int axis) const { return omega_waist[axis]; }
		double get_omega_torso(int axis) const { return omega_torso[axis]; }
		double get_omega_Rfoot(int axis) const { return omega_Rfoot[axis]; }
		double get_omega_Lfoot(int axis) const { return omega_Lfoot[axis]; }

		OptiInputs* get_opti_inputs() const { return opti_inputs; }
		UserCtrl* get_user_ctrl()     const { return user_ctrl;   }
		MotorCtrlIndex* get_indexes() const { return ctrl_index;     }
		CtrlOptions* get_options()    const { return options;     }


		// -- set inputs -- //

		void set_t(double value) { t = value; }

		void set_q(int mot_id, double value)      { q[mot_id]      = value; }
		void set_qd(int mot_id, double value)     { qd[mot_id]     = value; }
		void set_Qq(int mot_id, double value)     { Qq[mot_id]     = value; }
		void set_q_mot(int mot_id, double value)  { q_mot[mot_id]  = value; }
		void set_qd_mot(int mot_id, double value) { qd_mot[mot_id] = value; }

		void set_q_ref(int mot_id, double value)  { q_ref[mot_id]  = value; }
		void set_Qq_ref(int mot_id, double value) { Qq_ref[mot_id] = value; }

		// set motor values for controller indexes
		void set_ctrl_q(int ctrl_id, double value)      { q[ctrl_index->get_inv_index(ctrl_id)]      = value; }
		void set_ctrl_qd(int ctrl_id, double value)     { qd[ctrl_index->get_inv_index(ctrl_id)]     = value; }
		void set_ctrl_Qq(int ctrl_id, double value)     { Qq[ctrl_index->get_inv_index(ctrl_id)]     = value; }
		void set_ctrl_q_mot(int ctrl_id, double value)  { q_mot[ctrl_index->get_inv_index(ctrl_id)]  = value; }
		void set_ctrl_qd_mot(int ctrl_id, double value) { qd_mot[ctrl_index->get_inv_index(ctrl_id)] = value; }

		void set_F_Rfoot(int axis, double value) { F_Rfoot[axis] = value; }
		void set_F_Lfoot(int axis, double value) { F_Lfoot[axis] = value; }
		void set_T_Rfoot(int axis, double value) { T_Rfoot[axis] = value; }
		void set_T_Lfoot(int axis, double value) { T_Lfoot[axis] = value; }

		void set_F_Rfoot_IF(int axis, double value) { F_Rfoot_IF[axis] = value; }
		void set_F_Lfoot_IF(int axis, double value) { F_Lfoot_IF[axis] = value; }
		void set_F_Rtoe_IF(int axis, double value) { F_Rtoe_IF[axis] = value; }
		void set_F_Ltoe_IF(int axis, double value) { F_Ltoe_IF[axis] = value; }
		void set_T_Rfoot_IF(int axis, double value) { T_Rfoot_IF[axis] = value; }
		void set_T_Lfoot_IF(int axis, double value) { T_Lfoot_IF[axis] = value; }

		void set_IMU_Orientation(int comp_id, double value) { IMU_Orientation[comp_id] = value; }
		void set_IMU_Angular_Rate(int axis, double value)   { IMU_Angular_Rate[axis]   = value; }
		void set_IMU_Acceleration(int axis, double value)   { IMU_Acceleration[axis]   = value; }

		void set_Fz_feet(int foot_id, double value)    { Fz_feet[foot_id]    = value; }
		void set_mean_Fz_feet(int index, double value) { mean_Fz_feet[index] = value; }
		void set_Fz_toe(int foot_id, double value)    { Fz_toe[foot_id]    = value; }
		void set_mean_Fz_toe(int index, double value) { mean_Fz_toe[index] = value; }

		void set_theta_waist(int axis, double value) { theta_waist[axis] = value; }		
		void set_theta_torso(int axis, double value) { theta_torso[axis] = value; }
		void set_theta_Rfoot(int axis, double value) { theta_Rfoot[axis] = value; }		
		void set_theta_Lfoot(int axis, double value) { theta_Lfoot[axis] = value; }

		void set_omega_waist(int axis, double value) { omega_waist[axis] = value; }
		void set_omega_torso(int axis, double value) { omega_torso[axis] = value; }
		void set_omega_Rfoot(int axis, double value) { omega_Rfoot[axis] = value; }
		void set_omega_Lfoot(int axis, double value) { omega_Lfoot[axis] = value; }

		
	private:
		int nb_mot; ///< number of motors

		// interface 
		double t;                   ///< time [s]
		
		std::vector<double> q;      ///< position [rad]
		std::vector<double> qd;     ///< volocity [rad/s]
		std::vector<double> Qq;     ///< torque [Nm]
		std::vector<double> q_mot;  ///< position (motor) [rad]
		std::vector<double> qd_mot; ///< velocity (motor) [rad/s]

		std::vector<double> q_ref;  ///< position references (not directly applied) [rad]
		std::vector<double> Qq_ref; ///< torque references (not directly applied) [Nm]

		double F_Rfoot[3];          ///< right foot force [N]
		double F_Lfoot[3];          ///< left  foot force [N]
		double T_Rfoot[3];          ///< right foot torque [Nm]
		double T_Lfoot[3];          ///< left  foot torque [Nm]
		double IMU_Orientation[9];  ///< IMU rotation matrix [-] 
		double IMU_Angular_Rate[3]; ///< IMU angular rate [rad/s]
		double IMU_Acceleration[3]; ///< IMU acceleration [rad/s^2]

		// additional
		double Fz_feet[NB_LEGS];      ///< Ground reaction force under the feet [N]
		double mean_Fz_feet[NB_LEGS]; ///< mean feet forces [N]
		double Fz_toe[NB_LEGS];      ///< Ground reaction force under the toes [N]
		double mean_Fz_toe[NB_LEGS]; ///< mean toes forces [N]

		double theta_waist[3]; ///< waist angle [rad]
		double theta_torso[3]; ///< torso angle [rad]
		double theta_Rfoot[3]; ///< right foot angle [rad]
		double theta_Lfoot[3]; ///< left foot angle [rad]

		double omega_waist[3]; ///< waist angle derivatve [rad/s]
		double omega_torso[3]; ///< torso angle derivatve [rad/s]
		double omega_Rfoot[3]; ///< right foot angle derivatve [rad/s]
		double omega_Lfoot[3]; ///< left foot angle derivatve [rad/s]

		double F_Rfoot_IF[3]; ///< right foot force (in inertial frame) [N]
		double F_Lfoot_IF[3]; ///< left  foot force (in inertial frame) [N]
		double F_Rtoe_IF[3]; ///< right toe force (in inertial frame) [N]
		double F_Ltoe_IF[3]; ///< left  toe force (in inertial frame) [N]
		double T_Rfoot_IF[3]; ///< right foot torque (in inertial frame) [Nm]
		double T_Lfoot_IF[3]; ///< left  foot torque (in inertial frame) [Nm]

		// classes
		OptiInputs *opti_inputs; ///< inputs of the optimization
		UserCtrl *user_ctrl;     ///< user control structure
		MotorCtrlIndex *ctrl_index; ///< indexes for implemented motors controller
		CtrlOptions *options;    ///< controller options
};

#endif
