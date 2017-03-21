/*! 
 * \author Nicolas Van der Noot
 * \file simu_finish.cc
 * \brief Close the simulation
 */

#include "user_simu_functions.h"
#include "CppInterface.hh"
#include "SimuCtrl.hh"
#include "Fitness.hh"
#include "OptiClass.hh"
#include "GaitFeatures.hh"
#include "MeanSpeedAnalysis.hh"
#include "user_model.h"
#include "user_IO.h"

/*! \brief last call of the simulation
 * 
 * \param[in] mbs_data Robotran structure
 */
void simu_finish(MbsData *mbs_data)
{
	// variables declaration
	SimuCtrl *simu_ctrl;
	CppInterface *cppInterface;
	OptiClass *optiClass;
	Fitness *fitness_simu;
	GaitFeatures *gait_features;
	MeanSpeedAnalysis *mean_speed;

	// get requested classes
	cppInterface = static_cast<CppInterface*>(mbs_data->user_model->cppInterface);
	optiClass = static_cast<OptiClass*>(mbs_data->user_IO->optiClass);

	// set fitness
	simu_ctrl = cppInterface->get_simu_ctrl();
	fitness_simu = static_cast<Fitness*>(simu_ctrl->get_computation(FITNESS));
	optiClass->set_fitness(fitness_simu->get_total_fitness());
	optiClass->set_fitness_details(fitness_simu->get_fitness_details());


	// features
	gait_features = static_cast<GaitFeatures*>(simu_ctrl->get_computation(GAIT_FEATURES));
	mean_speed = static_cast<MeanSpeedAnalysis*>(gait_features->get_feature(MEAN_SPEED_FEAT));
	optiClass->set_v_real(mean_speed->get_mean_speed());


	// remove interface
	controller_finish_interface(mbs_data);

	delete cppInterface;
}
