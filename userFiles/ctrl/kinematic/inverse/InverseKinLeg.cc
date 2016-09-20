#include "InverseKinLeg.hh"
#include "coman_properties.hh"
#include "CtrlIndex.hh"

/*! \brief constructor
 *
 * \param[in] fwd_kin forward kinematics module
 * \param[in] nb_const number of constraints to solve
 */
InverseKinLeg::InverseKinLeg(ForwardKinematics *fwd_kin, MotorCtrlIndex *ctrl_index, CtrlInputs *inputs, int nb_const, int leg_id):
	InverseKin(fwd_kin, inputs, nb_const)
{
	this->leg_id = leg_id;

	switch (leg_id)
	{
		case R_ID:
			HipPitch_ctrl_id  = ctrl_index->get_inv_index(CtrlIndex::RightHipPitch);
			HipRoll_ctrl_id   = ctrl_index->get_inv_index(CtrlIndex::RightHipRoll);
			HipYaw_ctrl_id    = ctrl_index->get_inv_index(CtrlIndex::RightHipYaw);
			KneePitch_ctrl_id = ctrl_index->get_inv_index(CtrlIndex::RightKneePitch);
			FootRoll_ctrl_id  = ctrl_index->get_inv_index(CtrlIndex::RightFootRoll);
			FootPitch_ctrl_id = ctrl_index->get_inv_index(CtrlIndex::RightFootPitch);

			HipPitch_fwd_kin_id  = fwd_kin->get_fwd_kin_index(CtrlIndex::RightHipPitch);
			HipRoll_fwd_kin_id   = fwd_kin->get_fwd_kin_index(CtrlIndex::RightHipRoll);
			HipYaw_fwd_kin_id    = fwd_kin->get_fwd_kin_index(CtrlIndex::RightHipYaw);
			KneePitch_fwd_kin_id = fwd_kin->get_fwd_kin_index(CtrlIndex::RightKneePitch);
			FootRoll_fwd_kin_id  = fwd_kin->get_fwd_kin_index(CtrlIndex::RightFootRoll);
			FootPitch_fwd_kin_id = fwd_kin->get_fwd_kin_index(CtrlIndex::RightFootPitch);
			break;

		case L_ID:
			HipPitch_ctrl_id  = ctrl_index->get_inv_index(CtrlIndex::LeftHipPitch);
			HipRoll_ctrl_id   = ctrl_index->get_inv_index(CtrlIndex::LeftHipRoll);
			HipYaw_ctrl_id    = ctrl_index->get_inv_index(CtrlIndex::LeftHipYaw);
			KneePitch_ctrl_id = ctrl_index->get_inv_index(CtrlIndex::LeftKneePitch);
			FootRoll_ctrl_id  = ctrl_index->get_inv_index(CtrlIndex::LeftFootRoll);
			FootPitch_ctrl_id = ctrl_index->get_inv_index(CtrlIndex::LeftFootPitch);

			HipPitch_fwd_kin_id  = fwd_kin->get_fwd_kin_index(CtrlIndex::LeftHipPitch);
			HipRoll_fwd_kin_id   = fwd_kin->get_fwd_kin_index(CtrlIndex::LeftHipRoll);
			HipYaw_fwd_kin_id    = fwd_kin->get_fwd_kin_index(CtrlIndex::LeftHipYaw);
			KneePitch_fwd_kin_id = fwd_kin->get_fwd_kin_index(CtrlIndex::LeftKneePitch);
			FootRoll_fwd_kin_id  = fwd_kin->get_fwd_kin_index(CtrlIndex::LeftFootRoll);
			FootPitch_fwd_kin_id = fwd_kin->get_fwd_kin_index(CtrlIndex::LeftFootPitch);
			break;

		default:
			std::cout << "Error: unknown leg index: " << leg_id << " !" << std::endl;
			exit(EXIT_FAILURE);
			break;
	}

	ctrl_vec.push_back(HipPitch_ctrl_id);
	ctrl_vec.push_back(HipRoll_ctrl_id);
	ctrl_vec.push_back(HipYaw_ctrl_id);
	ctrl_vec.push_back(KneePitch_ctrl_id);
	ctrl_vec.push_back(FootRoll_ctrl_id);
	ctrl_vec.push_back(FootPitch_ctrl_id);

	fwd_kin_vec.push_back(HipPitch_fwd_kin_id);
	fwd_kin_vec.push_back(HipRoll_fwd_kin_id);
	fwd_kin_vec.push_back(HipYaw_fwd_kin_id);
	fwd_kin_vec.push_back(KneePitch_fwd_kin_id);
	fwd_kin_vec.push_back(FootRoll_fwd_kin_id);
	fwd_kin_vec.push_back(FootPitch_fwd_kin_id);

	// safety
	if (ctrl_vec.size() != nb_const || fwd_kin_vec.size() != nb_const)
	{
		std::cout << "Error: " << nb_const << " constraints, but size of 'ctrl_vec' or 'fwd_kin_vec' not matching !" << std::endl;
		exit(EXIT_FAILURE);
	}

	for(int i=0; i<3; i++)
	{
		foot_ref[i]   = 0.0;
		R_foot_ref[i] = 0.0;
	}
}

/*! \brief destructor
 */
InverseKinLeg::~InverseKinLeg()
{

}

/*! \brief fill the position vector for the main kinematics for the first iteration
 */
void InverseKinLeg::fill_init_pos()
{
	double cur_val;

	// identity matrix for the orientation
	io.IMU_Orientation[0] = 1.0;
	io.IMU_Orientation[1] = 0.0;
	io.IMU_Orientation[2] = 0.0;
	io.IMU_Orientation[3] = 0.0;
	io.IMU_Orientation[4] = 1.0;
	io.IMU_Orientation[5] = 0.0;
	io.IMU_Orientation[6] = 0.0;
	io.IMU_Orientation[7] = 0.0;
	io.IMU_Orientation[8] = 1.0;

	// start value: current motor joint positions
	for(int i=0; i<nb_const; i++)
	{
		cur_val = inputs->get_q_mot(ctrl_vec[i]);

		io.q_mot[ctrl_vec[i]] = cur_val;
		pos.set(i, 0, cur_val);
	}
}

/*! \brief fill the position vector for the main kinematics
 */
void InverseKinLeg::fill_pos()
{
	for(int i=0; i<nb_const; i++)
	{
		io.q_mot[ctrl_vec[i]] = pos.get(i, 0);
	}
}

/*! \brief fill the constraints matrix
 */
void InverseKinLeg::fill_constraints()
{
	for(int i=0; i<3; i++)
	{
		// foot position
		if (leg_id == R_ID)
		{
			h.set(i, 0, foot_ref[i] - io.r_Rfoot[i]);
		}
		else
		{
			h.set(i, 0, foot_ref[i] - io.r_Lfoot[i]);
		}
	}

	// foot orientation
	if (leg_id == R_ID)
	{
		h.set(3, 0, R_foot_ref[0] + io.Rfoot_or[7]);
		h.set(4, 0, R_foot_ref[1] - io.Rfoot_or[6]);
		h.set(5, 0, R_foot_ref[2] + io.Rfoot_or[3]);
	}
	else
	{
		h.set(3, 0, R_foot_ref[0] + io.Lfoot_or[7]);
		h.set(4, 0, R_foot_ref[1] - io.Lfoot_or[6]);
		h.set(5, 0, R_foot_ref[2] + io.Lfoot_or[3]);
	}
}

/*! \brief fill the jacobian matrix
 */
void InverseKinLeg::fill_jacobian()
{
	int der_id;

	for(int i=0; i<3; i++)
	{
		for(int j=0; j<nb_const; j++)
		{
			der_id = fwd_kin_vec[j];

			// foot position
			if (leg_id == R_ID)
			{
				jac.set(i, j, -io.r_Rfoot_der[der_id][i]);
			}
			else
			{
				jac.set(i, j, -io.r_Lfoot_der[der_id][i]);
			}
		}
	}

	for(int j=0; j<nb_const; j++)
	{
		der_id = fwd_kin_vec[j];

		// foot orientation
		if (leg_id == R_ID)
		{
			jac.set(3, j,  io.Rfoot_or_der[der_id][7]);
			jac.set(4, j, -io.Rfoot_or_der[der_id][6]);
			jac.set(5, j,  io.Rfoot_or_der[der_id][3]);
		}
		else
		{
			jac.set(3, j,  io.Lfoot_or_der[der_id][7]);
			jac.set(4, j, -io.Lfoot_or_der[der_id][6]);
			jac.set(5, j,  io.Lfoot_or_der[der_id][3]);
		}
	}
}

/*! \brief set the foot position reference
 * 
 * \param[in] x x position of the foot (relative to the waist) [m]
 * \param[in] y y position of the foot (relative to the waist) [m]
 * \param[in] z z position of the foot (relative to the waist) [m]
 */
void InverseKinLeg::set_pos_ref(double x, double y, double z)
{
	foot_ref[0] = x;
	foot_ref[1] = y;
	foot_ref[2] = z;
}

/*! \brief set the rotation matrix reference based on a succession of R1-R2-R3
 * 
 * \param[in] theta_1 first angle: R1 [rad]
 * \param[in] theta_2 second angle: R2 [rad]
 * \param[in] theta_3 third angle: R3 [rad]
 */
void InverseKinLeg::set_theta_ref(double theta_1, double theta_2, double theta_3)
{
	double c2 = cos(theta_2);

	// -[2][1]: -R_32
	R_foot_ref[0] = c2*sin(theta_1);

	// [2][0]: R_31
	R_foot_ref[1] = sin(theta_2);

	// -[1][0]: -R_21
	R_foot_ref[2] = c2*sin(theta_3);
}
