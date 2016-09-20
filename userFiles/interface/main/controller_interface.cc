/*! 
 * \author Nicolas Van der Noot
 * \file controller_interface.cc
 * \brief Controller interface
 */

#include "controller_io.hh"
#include "CppInterface.hh"
#include "LowLevelGains.hh"
#include "SensorsInfo.hh"
#include "SimuCtrl.hh"
#include "user_model.h"
#include "user_IO.h"
#include "OptiClass.hh"
#include "MotorCoManIndex.hh"
#include "RobotActuators.hh"

//inputs and outputs structure
static Inputs_ctrl  *ivs;
static Outputs_ctrl *ovs;
static LowLevelGains *low_level_gains;
static MotorCoManIndex *coman_index;

// function prototypes
void controller_inputs(MbsData *mbs_data, MotorCoManIndex *coman_index, Inputs_ctrl *ivs, SensorsInfo *sens_info);
void controller_outputs(MotorCoManIndex *coman_index, RobotActuators *actuators, LowLevelGains *gains, Outputs_ctrl *ovs);
void controller_tracking_mode(MotorCoManIndex *coman_index, LowLevelGains *gains);

/*! \brief get the controller from mbs_data
 * 
 * \param[in] mbs_data Robotran structure
 * \return controller
 */
Ctrl* get_ctrl(MbsData *mbs_data)
{
	CppInterface *cppInterface = static_cast<CppInterface*>(mbs_data->user_model->cppInterface);

	return cppInterface->get_ctrl();
}

/*! \brief get Sensors information from mbs_data
 * 
 * \param[in] mbs_data Robotran structure
 * \return sensors information
 */
SensorsInfo* get_sens_info(MbsData *mbs_data)
{
	CppInterface *cppInterface = static_cast<CppInterface*>(mbs_data->user_model->cppInterface);

	return static_cast<SensorsInfo*>(cppInterface->get_simu_ctrl()->get_computation(SENSORS_INFO));
}

/*! \brief get actuators list from mbs_data
 * 
 * \param[in] mbs_data Robotran structure
 * \return sctuators list
 */
RobotActuators* get_actuators(MbsData *mbs_data)
{
	CppInterface *cppInterface = static_cast<CppInterface*>(mbs_data->user_model->cppInterface);

	return cppInterface->get_simu_ctrl()->get_actuators();
}

/*! \brief interface of the controller: initialization
 * 
 * \param[in] mbs_data Roboptran structure
 */
void controller_init_interface(MbsData *mbs_data)
{
	int nb_mot;

	ModelSimuIndex simu_index(mbs_data, mbs_data->user_IO->options);
	SensorsInfo sens_info(mbs_data, &simu_index);

	Ctrl *ctrl = get_ctrl(mbs_data);

	// coman_index of the motors
	coman_index = new MotorCoManIndex(0);

	nb_mot = coman_index->get_nb_mot();

	// io structures initialization
	ivs = init_Inputs_ctrl(nb_mot);
	ovs = init_Outputs_ctrl(nb_mot);

	low_level_gains = new LowLevelGains(get_actuators(mbs_data));

	// choice between position and torque tracking
	controller_tracking_mode(coman_index, low_level_gains);

	// interface: inputs (generic)
	sens_info.compute();
	controller_inputs(mbs_data, coman_index, ivs, &sens_info);

	ctrl->get_user_ctrl()->set_v_request(mbs_data->user_IO->options->opti_speed_ref);

	ctrl->set_inputs(ivs);
	ctrl->init_ctrl();
}

/*! \brief interface of the controller: loop
 * 
 * \param[in] mbs_data Robotran structure
 */
void controller_loop_interface(MbsData *mbs_data)
{
	Ctrl *ctrl = get_ctrl(mbs_data);

	// interface: inputs (generic)
	controller_inputs(mbs_data, coman_index, ivs, get_sens_info(mbs_data));

	ctrl->set_inputs(ivs);
	ctrl->loop_ctrl();
	ctrl->set_outputs(ovs);

	// interface: outputs (generic)
	controller_outputs(coman_index, get_actuators(mbs_data), low_level_gains, ovs);
}

/*! \brief interface of the controller: finish
 * 
 * \param[in] mbs_data Robotran structure
 */
void controller_finish_interface(MbsData *mbs_data)
{
	get_ctrl(mbs_data)->finish_ctrl();

	// io structures release memory
	delete low_level_gains;
	delete coman_index;

	free_Inputs_ctrl(ivs);
	free_Outputs_ctrl(ovs);
}
