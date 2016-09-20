/*! 
 * \author Nicolas Van der Noot
 * \file controller_inputs.cc
 * \brief Generic file to get the inputs from the controller
 */

#include "controller_io.hh"
#include "SensorsInfo.hh"
#include "WholeFeet.hh"
#include "CppInterface.hh"
#include "user_model.h"
#include "CoManIndex.hh"
#include "SimuCtrl.hh"
#include "ModelSimuIndex.hh"
#include "MotorCoManIndex.hh"
#include "RobotActuators.hh"
#include "get_simu_id_interface.hh"
#include "user_IO.h"

// function prototype
void correct_unknow_z_IMU(Inputs_ctrl *ivs);

/*! \brief assigns the inputs of the controller
 * 
 * \param[in] mbs_data Robotran structure
 * \param[out] ivs inputs structure
 * \param[in] sens_info info from sensors
 */
void controller_inputs(MbsData *mbs_data, MotorCoManIndex *coman_index, Inputs_ctrl *ivs, SensorsInfo *sens_info)
{
	int i_mot, robotran_id, motor_id;
	double FR[3], FL[3], TR[3], TL[3];
	double RR[9], RL[9];
	double FRD[3], FLD[3];

	WholeFeet *whole_feet;
	CppInterface *cppInterface;
	SimuCtrl *simuCtrl;
	ModelSimuIndex *simu_index;
	RobotActuators *actuators;
	SimuOptions *options;
	ContactGeom::RigidShape *rfoot_shape;
	ContactGeom::RigidShape *lfoot_shape;

	cppInterface = static_cast<CppInterface*>(mbs_data->user_model->cppInterface);

	options = mbs_data->user_IO->options;

	simuCtrl = cppInterface->get_simu_ctrl();

	simu_index = simuCtrl->get_simu_index();

	actuators = simuCtrl->get_actuators();

	switch (options->gcm_model)
	{
		case MESH_GCM_MODEL:
			whole_feet = cppInterface->get_simu_ctrl()->get_gcm_mesh()->get_whole_feet();
			break;

		case PRIM_GCM_MODEL:
			rfoot_shape = simuCtrl->get_rfoot_shape();
			lfoot_shape = simuCtrl->get_lfoot_shape();
			break;
	
		default:
			break;
	}


	// -- Time -- //

	ivs->t = mbs_data->tsim; // time [s]


	// ---- Forces under the feet ---- //

	// Vertical forces [N]

	for (int i=0; i<3; i++)
	{
		switch (options->gcm_model)
		{
			case NO_GCM_MODEL:
				FR[i] = 0.0;
				FL[i] = 0.0;
				FRD[i] = 0.0;
				FLD[i] = 0.0;

				TR[i] = 0.0;
				TL[i] = 0.0;
				break;

			case MESH_GCM_MODEL:
				FR[i] = whole_feet->get_leg_feet_forces(RIGHT_ID, i);
				FL[i] = whole_feet->get_leg_feet_forces(LEFT_ID, i);
				FRD[i] = whole_feet->get_leg_feet_dist_forces(RIGHT_ID, i);
				FLD[i] = whole_feet->get_leg_feet_dist_forces(LEFT_ID, i);

				TR[i] = whole_feet->get_leg_feet_torques(RIGHT_ID, i);
				TL[i] = whole_feet->get_leg_feet_torques(LEFT_ID, i);
				break;

			case PRIM_GCM_MODEL:
				if (rfoot_shape != NULL && lfoot_shape != NULL )
				{
					FR[i] = rfoot_shape->get_F_tot_comp(i);
					FL[i] = lfoot_shape->get_F_tot_comp(i);
					FRD[i] = rfoot_shape->get_F_tot_comp(i);
					FLD[i] = lfoot_shape->get_F_tot_comp(i);

					TR[i] = rfoot_shape->get_T_tot_comp(i);
					TL[i] = lfoot_shape->get_T_tot_comp(i);
				}
				else
				{
					FR[i] = 0.0;
					FL[i] = 0.0;
					FRD[i] = 0.0;
					FLD[i] = 0.0;

					TR[i] = 0.0;
					TL[i] = 0.0;
				}
				break;
		
			default:
				std::cout << "Errot: unknown GCM !" << std::endl;
				exit(EXIT_FAILURE);
		}
	}

	for (int i=0; i<9; i++)
	{
		RR[i] = sens_info->get_S_RFoots_R(i);
		RL[i] = sens_info->get_S_LFoots_R(i);
	}

	// feet forces [N]
	ivs->F_Rfoot[0] = RR[0]*FR[0] + RR[3]*FR[1] + RR[6]*FR[2];
	ivs->F_Rfoot[1] = RR[1]*FR[0] + RR[4]*FR[1] + RR[7]*FR[2];
	ivs->F_Rfoot[2] = RR[2]*FR[0] + RR[5]*FR[1] + RR[8]*FR[2];

	ivs->F_Lfoot[0] = RL[0]*FL[0] + RL[3]*FL[1] + RL[6]*FL[2];
	ivs->F_Lfoot[1] = RL[1]*FL[0] + RL[4]*FL[1] + RL[7]*FL[2];
	ivs->F_Lfoot[2] = RL[2]*FL[0] + RL[5]*FL[1] + RL[8]*FL[2];

	// toes forces [N]
	ivs->F_RToe_IF[0] = FRD[0];
	ivs->F_RToe_IF[1] = FRD[1]; 
	ivs->F_RToe_IF[2] = FRD[2];

	ivs->F_LToe_IF[0] = FLD[0]; 
	ivs->F_LToe_IF[1] = FLD[1]; 
	ivs->F_LToe_IF[2] = FLD[2]; 

	// feet torques [Nm]
	ivs->T_Rfoot[0] = RR[0]*TR[0] + RR[3]*TR[1] + RR[6]*TR[2];
	ivs->T_Rfoot[1] = RR[1]*TR[0] + RR[4]*TR[1] + RR[7]*TR[2];
	ivs->T_Rfoot[2] = RR[2]*TR[0] + RR[5]*TR[1] + RR[8]*TR[2];

	ivs->T_Lfoot[0] = RL[0]*TL[0] + RL[3]*TL[1] + RL[6]*TL[2];
	ivs->T_Lfoot[1] = RL[1]*TL[0] + RL[4]*TL[1] + RL[7]*TL[2];
	ivs->T_Lfoot[2] = RL[2]*TL[0] + RL[5]*TL[1] + RL[8]*TL[2];


	// -- Joint positions, velocities and torques -- //

	for(int i=0; i<coman_index->get_nb_mot(); i++)
	{
		motor_id = coman_index->get_index(i);

		robotran_id = simu_index->get_mbs_jt(get_simu_id_interface(motor_id));

		i_mot = actuators->get_inv_actuators_index(get_simu_id_interface(motor_id)) + 1;

		// motors (position - velocity) -> before the springs
		ivs->q_mot[motor_id]  = mbs_data->ux[i_mot];  // position [rad]
		ivs->qd_mot[motor_id] = mbs_data->uxd[i_mot]; // velocity [rad/s]

		// absolute joints (position - velocity - torques) -> after the springs
		ivs->q[motor_id]  = mbs_data->q[robotran_id];   // position [rad]
		ivs->qd[motor_id] = mbs_data->qd[robotran_id];  // velocity [rad/s]
		ivs->Qq[motor_id] = mbs_data->Qq[robotran_id];  // torque   [Nm]
	}

	// -- IMU -- //

	for (int i=0; i<9; i++)
	{
		ivs->IMU_Orientation[i] = sens_info->get_S_MidWaist_R(i);
	}

	// IMU absolute velocity and acceleration
	for (int i=0; i<3; i++)
	{
		ivs->IMU_Angular_Rate[i] = sens_info->get_S_MidWaist_OM(i);  // angulare rate -> velocity [rad/s]
		ivs->IMU_Acceleration[i] = sens_info->get_S_MidWaist_OMP(i); // acceleration [rad/s^2]
	}

	// info not accurately available on the real COMAN
	//correct_unknow_z_IMU(ivs);
}

/*! \brief correct the IMU information for unknown z axis
 * 
 * \param[in,out] ivs inputs structure
 */
void correct_unknow_z_IMU(Inputs_ctrl *ivs)
{
	double IMU11, IMU12, IMU13, IMU23, IMU33;
	double theta_1, theta_2, theta_3;
	double theta_p_1, theta_p_2, theta_p_3;
	double omega_1, omega_2, omega_3;
	double inv_c2;
	double c1, c2, c3;
	double s1, s2, s3;

	// rotation matrix transformed for unknown z axis
	IMU11 = ivs->IMU_Orientation[0];
	IMU12 = ivs->IMU_Orientation[1];
	IMU13 = ivs->IMU_Orientation[2];
	IMU23 = ivs->IMU_Orientation[5];
	IMU33 = ivs->IMU_Orientation[8];

	theta_1 = atan2(IMU23, IMU33);
	theta_2 = atan2(-IMU13, sqrt(IMU11*IMU11 + IMU12*IMU12));
	theta_3 = 0.0; // atan2(IMU12, IMU11) to get correct angle

	c1 = cos(theta_1);
	c2 = cos(theta_2);
	c3 = cos(theta_3);

	s1 = sin(theta_1);
	s2 = sin(theta_2);
	s3 = sin(theta_3);

	ivs->IMU_Orientation[0] = c2*c3;
	ivs->IMU_Orientation[1] = c2*s3;
	ivs->IMU_Orientation[2] = -s2;
	ivs->IMU_Orientation[3] = -c1*s3 + s1*s2*c3;
	ivs->IMU_Orientation[4] = c1*c3 + s1*s2*s3;
	ivs->IMU_Orientation[5] = s1*c2;
	ivs->IMU_Orientation[6] = s1*s3 + c1*s2*c3;
	ivs->IMU_Orientation[7] = -s1*c3 + c1*s2*s3;
	ivs->IMU_Orientation[8] = c1*c2;

	if (!c2)
	{
		return;
	}

	omega_1 = ivs->IMU_Angular_Rate[0];
	omega_2 = ivs->IMU_Angular_Rate[1];
	omega_3 = ivs->IMU_Angular_Rate[2];

	inv_c2 = 1.0 / c2;

	theta_p_1 = inv_c2 * (c3*omega_1 + s3*omega_2);
	theta_p_2 = c3*omega_2 - s3*omega_1;
	theta_p_3 = 0.0; // inv_c2 * s2 * (s3*omega_2 + c3*omega_1) + omega_3 to get correct angle derivative

	ivs->IMU_Angular_Rate[0] = c2*c3*theta_p_1 - s3*theta_p_2;
	ivs->IMU_Angular_Rate[1] = c2*s3*theta_p_1 + c3*theta_p_2;
	ivs->IMU_Angular_Rate[2] = -s2*theta_p_1 + theta_p_3;

	// ivs->IMU_Acceleration not corrected
}
