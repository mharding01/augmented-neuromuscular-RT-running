
#include "TorqueFitness.hh"
#include "NicoCtrl.hh"
#include "CtrlIndex.hh"

#include "user_all_id.h"
#include "cmake_config.h"
#include <fstream>

#define T_START_SECOND_CYCLE 1.114125 // [s] = mns_data->tf/2.0 (1.06975 for 4m/s)

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] sens_info info from the sensor
 */
TorqueFitness::TorqueFitness(MbsData *mbs_data, Ctrl *ctrl): FitnessStage(mbs_data)
{
	NicoCtrl *nico_ctrl = static_cast<NicoCtrl*>(ctrl);
	outputs = static_cast<CtrlOutputs*>(nico_ctrl->get_outputs());
	MotorCtrlIndex *motor_index = static_cast<MotorCtrlIndex*>(nico_ctrl->get_motor_index());

	dt = mbs_data->dt0;

	RightHipPitch_id  = motor_index->get_inv_index(CtrlIndex::RightHipPitch);
	RightKneePitch_id = motor_index->get_inv_index(CtrlIndex::RightKneePitch);
    RightFootPitch_id = motor_index->get_inv_index(CtrlIndex::RightFootPitch);

	// torque from Wang data 
	std::ifstream ihip(PROJECT_SOURCE_DIR"/../userFiles/dataWang/hipQq.txt", std::ios::in);
	std::ifstream iknee(PROJECT_SOURCE_DIR"/../userFiles/dataWang/kneeQq.txt", std::ios::in);
	std::ifstream iankle(PROJECT_SOURCE_DIR"/../userFiles/dataWang/ankleQq.txt", std::ios::in);
	if (!ihip.is_open() || !iknee.is_open() || !iankle.is_open())
	{
	  	std::cout << "Error: cant open wang hip files" << std::endl;
	  	exit(1);
	}
	double num = 0.0;
	while (ihip >> num)
		RHipQq.push_back(num);
	   num = 0.0;
	while (iknee >> num)
		RKneeQq.push_back(num);
	num = 0.0;
	while (iankle >> num)
		RAnkleQq.push_back(num);

	serror = 0.0;

	l_coman = 42.7;
	l_wang = 93.0;
}

/*! \brief destructor
 */
TorqueFitness::~TorqueFitness()
{

}

/*! \brief compute variables at each time step
 */
void TorqueFitness::compute()
{
	double t;
	double index;
	double err_hip, err_knee, err_ankle;

	t = mbs_data->tsim;

	if (t >= T_START_SECOND_CYCLE)
	{
		index = t/dt;

		err_hip   = ((outputs->get_Qq_ref(RightHipPitch_id )/(MASS_COMAN*l_coman)) - (RHipQq[index]/l_wang));
		err_knee  = ((outputs->get_Qq_ref(RightKneePitch_id)/(MASS_COMAN*l_coman)) - (RKneeQq[index]/l_wang));
		err_ankle = ((outputs->get_Qq_ref(RightFootPitch_id)/(MASS_COMAN*l_coman)) - (RAnkleQq[index]/l_wang));

		serror += err_hip*err_hip + err_knee*err_knee + err_ankle*err_ankle;
	}
}

/*! \brief get fitness
 * 
 * \return fitness
 */
double TorqueFitness::get_fitness()
{
	return -serror;
}

/*! \brief detect if next stage is unlocked
 * 
 * \return 1 if next stage unlocked, 0 otherwise
 */
int TorqueFitness::next_stage_unlocked()
{
	return 1;
}
