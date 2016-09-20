/*! 
 * \author Nicolas Van der Noot
 * \file InverseKinLeg.hh
 * \brief InverseKineLeg class
 */

#ifndef _INVERSE_KIN_LEG_HH_
#define _INVERSE_KIN_LEG_HH_

#include "InverseKin.hh"
#include <vector>

/*! \brief inverse kinematics for the legs
 */
class InverseKinLeg: public InverseKin
{
	public:
		InverseKinLeg(ForwardKinematics *fwd_kin, MotorCtrlIndex *ctrl_index, CtrlInputs *inputs, int nb_const, int leg_id);
		virtual ~InverseKinLeg();

		virtual void fill_pos();
		virtual void fill_init_pos();
		virtual void fill_constraints();
		virtual void fill_jacobian();

		/// set the foot position reference
		void set_foot_ref(int i, double value) { foot_ref[i] = value; }

		/// set the foot orientation reference
		void set_R_foot_ref(int i, double value) { R_foot_ref[i] = value; }

		void set_pos_ref(double x, double y, double z);
		void set_theta_ref(double theta_1, double theta_2, double theta_3);

	private:
		int leg_id; ///< ID of the leg

		// i/o vector IDs
		int HipPitch_ctrl_id;  ///< hip pitch joint: controller ID
		int HipRoll_ctrl_id;   ///< hip roll joint: controller ID
		int HipYaw_ctrl_id;    ///< hip yaw joint: controller ID
		int KneePitch_ctrl_id; ///< knee pitch joint: controller ID
		int FootRoll_ctrl_id;  ///< foot roll joint: controller ID
		int FootPitch_ctrl_id; ///< foot pitch joint: controller ID

		int HipPitch_fwd_kin_id;  ///< hip pitch joint: forward kinematics ID
		int HipRoll_fwd_kin_id;   ///< hip roll joint: forward kinematics ID
		int HipYaw_fwd_kin_id;    ///< hip yaw joint: forward kinematics ID
		int KneePitch_fwd_kin_id; ///< knee pitch joint: forward kinematics ID
		int FootRoll_fwd_kin_id;  ///< foot roll joint: forward kinematics ID
		int FootPitch_fwd_kin_id; ///< foot pitch joint: forward kinematics ID

		std::vector<int> ctrl_vec;    ///< vector with the controller indexes
		std::vector<int> fwd_kin_vec; ///< vector with the forward kinematics indexes

		// targets
		double foot_ref[3]; ///< reference position of the foot relative to the waist [m]
		double R_foot_ref[3]; ///< reference rotation matrix of the foot (realtive to the waist) [-], elements -[2][1], [2][0] and -[1][0]
};

#endif
