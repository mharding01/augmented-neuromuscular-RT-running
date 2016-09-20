#include "RobotActuators.hh"
#include "SimuIndex.hh"
#include "user_IO.h"

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] simu_index indexes of the simulation
 */
RobotActuators::RobotActuators(MbsData *mbs_data, ModelSimuIndex *simu_index)
{
	int coman_model;
	int cur_index;
	double J, D, V_T;

	options = mbs_data->user_IO->options;

	this->mbs_data   = mbs_data;
	this->simu_index = simu_index;

	coman_model = options->coman_model;

	// common characteristics for all motors
	J   = 0.1387;
	D   = 25.52;
	V_T = 6.175;

	// Actuator arguments: mbs_data, mbs_index, K_s, D_s, J, D, V_T, Aco, Bco

	// right leg
	actuators_list.push_back(new Actuator(mbs_data, simu_index, SimuJointIndex::RightHipPitch,  395.46, 0.198, J, D, V_T, 2.682984, 0.0));
	actuators_list.push_back(new Actuator(mbs_data, simu_index, SimuJointIndex::RightHipRoll,   8400.0, 4.2,   J, D, V_T, 1.751820, 0.0));
	actuators_list.push_back(new Actuator(mbs_data, simu_index, SimuJointIndex::RightHipYaw,    8400.0, 4.2,   J, D, V_T, 0.872477, 0.0));
	actuators_list.push_back(new Actuator(mbs_data, simu_index, SimuJointIndex::RightKneePitch, 395.46, 0.198, J, D, V_T, 2.226734, 0.0));
	actuators_list.push_back(new Actuator(mbs_data, simu_index, SimuJointIndex::RightFootRoll,  8400.0, 4.2,   J, D, V_T, 1.300777, 0.0));
	actuators_list.push_back(new Actuator(mbs_data, simu_index, SimuJointIndex::RightFootPitch, 395.46, 0.198, J, D, V_T, 2.521183, 0.0));

	// left leg
	actuators_list.push_back(new Actuator(mbs_data, simu_index, SimuJointIndex::LeftHipPitch,  395.46, 0.198, J, D, V_T, 1.767830, 0.0));
	actuators_list.push_back(new Actuator(mbs_data, simu_index, SimuJointIndex::LeftHipRoll,   8400.0, 4.2,   J, D, V_T, 1.957402, 0.0));
	actuators_list.push_back(new Actuator(mbs_data, simu_index, SimuJointIndex::LeftHipYaw,    8400.0, 4.2,   J, D, V_T, 1.165841, 0.0));
	actuators_list.push_back(new Actuator(mbs_data, simu_index, SimuJointIndex::LeftKneePitch, 395.46, 0.198, J, D, V_T, 2.715265, 0.0));
	actuators_list.push_back(new Actuator(mbs_data, simu_index, SimuJointIndex::LeftFootRoll,  8400.0, 4.2,   J, D, V_T, 0.688135, 0.0));
	actuators_list.push_back(new Actuator(mbs_data, simu_index, SimuJointIndex::LeftFootPitch, 395.46, 0.198, J, D, V_T, 1.588422, 0.0));

	// torso
	actuators_list.push_back(new Actuator(mbs_data, simu_index, SimuJointIndex::TorsoRoll,  8400.0, 4.2,   J, D, V_T, 1.277784, 0.0));
	actuators_list.push_back(new Actuator(mbs_data, simu_index, SimuJointIndex::TorsoPitch, 395.46, 0.198, J, D, V_T, 1.820269, 0.0));
	actuators_list.push_back(new Actuator(mbs_data, simu_index, SimuJointIndex::TorsoYaw,   395.46, 0.198, J, D, V_T, 2.013845, 0.0));

	// right arm
	actuators_list.push_back(new Actuator(mbs_data, simu_index, SimuJointIndex::RightShPitch,  120.0,  0.198, J, D, V_T, 1.372548, 0.0));
	actuators_list.push_back(new Actuator(mbs_data, simu_index, SimuJointIndex::RightShRoll,   120.0,  0.198, J, D, V_T, 1.790012, 0.0));
	actuators_list.push_back(new Actuator(mbs_data, simu_index, SimuJointIndex::RightShYaw,    8400.0, 4.2,   J, D, V_T, 1.384795, 0.0));
	actuators_list.push_back(new Actuator(mbs_data, simu_index, SimuJointIndex::RightElbPitch, 120.0,  0.198, J, D, V_T, 0.803593, 0.0));

	// left arm
	actuators_list.push_back(new Actuator(mbs_data, simu_index, SimuJointIndex::LeftShPitch,  120.0,  0.198, J, D, V_T, 1.909666, 0.0));
	actuators_list.push_back(new Actuator(mbs_data, simu_index, SimuJointIndex::LeftShRoll,   120.0,  0.198, J, D, V_T, 2.076732, 0.0));
	actuators_list.push_back(new Actuator(mbs_data, simu_index, SimuJointIndex::LeftShYaw,    8400.0, 4.2,   J, D, V_T, 1.671781, 0.0));
	actuators_list.push_back(new Actuator(mbs_data, simu_index, SimuJointIndex::LeftElbPitch, 120.0,  0.198, J, D, V_T, 0.803593, 0.0));

	nb_mot = actuators_list.size();

	// get maximal index
	index_max = -1;

	for(unsigned int i=0; i<actuators_list.size(); i++)
	{
		cur_index = actuators_list[i]->get_simu_index();

		if (cur_index > index_max)
		{
			index_max = cur_index;
		}
	}

	for(int i=0; i<=index_max; i++)
	{
		inv_actuators_index.push_back(-1);
	}

	for(unsigned int i=0; i<actuators_list.size(); i++)
	{
		cur_index = actuators_list[i]->get_simu_index();
		inv_actuators_index[cur_index] = i;
	}
}

/*! \brief destructor
 */
RobotActuators::~RobotActuators()
{
	for(unsigned int i=0; i<actuators_list.size(); i++)
	{
		delete actuators_list[i];
	}
}

/*! \brief compute uxd (user derivative with MbsData)
 * 
 * \param[in] ux vector integrated by Robotran
 * \param[out] uxd vector to integrate with Robotran
 *
 * ux  : [q_m , qd_m ]
 * uxd : [qd_m, qdd_m]
 */
void RobotActuators::motor_compute_der(double *ux, double *uxd)
{
	int i_mot;
	double q_m, qd_m;
	Actuator *cur_act;

	// qd_m is qdd_m integrated
	for(int i=1; i<=nb_mot; i++)
	{
		uxd[i] = ux[nb_mot+i];
	}

	for(int i=0; i<nb_mot; i++)
	{
		i_mot = i + 1;

		q_m  = ux[i_mot];
		qd_m = uxd[i_mot];

		cur_act = actuators_list[i];

		cur_act->set_pos_vel(q_m, qd_m);
		cur_act->compute_acceleration();

		// get qdd_m
		uxd[nb_mot+i_mot] = cur_act->get_qdd_m();
	}
}

/*! \brief compute the torque due to the motors
 * 
 * \param[in] ux vector integrated by Robotran
 * \param[in] uxd vector to integrate with Robotran
 * \param[out] Qq torque vector (with the MBS file indexes)
 *
 * ux  : [q_m , qd_m ]
 * uxd : [qd_m, qdd_m]
 */
void RobotActuators::motor_compute_tor(double *ux, double *uxd, double *Qq)
{
	int i_mot;
	double q_m, qd_m;
	Actuator *cur_act;

	for(int i=0; i<nb_mot; i++)
	{
		i_mot = i + 1;
		
		q_m  = ux[i_mot];
		qd_m = uxd[i_mot];

		cur_act = actuators_list[i];

		cur_act->set_pos_vel(q_m, qd_m);
		cur_act->compute_torque();

		Qq[cur_act->get_mbs_index()] = cur_act->get_Qq();
	}
}
