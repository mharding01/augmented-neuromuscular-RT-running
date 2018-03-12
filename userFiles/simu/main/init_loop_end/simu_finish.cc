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
#include "SwingStanceAnalysis.hh"
#include "MetEnergyFitness.hh"

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
	SwingStanceAnalysis *sw_st;
	MetEnergyFitness *energy_fitness;

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

	sw_st = static_cast<SwingStanceAnalysis*>(gait_features->get_feature(SWING_STANCE_FEAT));
	optiClass->set_stride_period_mean(sw_st->get_stride_period_mean());
	optiClass->set_stride_length_mean(sw_st->get_stride_length_mean());
	optiClass->set_take_off_mean(sw_st->get_take_off_mean());
	optiClass->set_ds_cycle_mean(sw_st->get_ds_cycle_mean());
	optiClass->set_flight_cycle_mean(sw_st->get_flight_cycle_mean());

	energy_fitness = static_cast<MetEnergyFitness*>(fitness_simu->get_stage(5));
	optiClass->set_met_energy_legs(energy_fitness->get_met_energy_legs_norm());
	optiClass->set_met_energy_total(energy_fitness->get_met_energy_total_norm());


	// final time
	optiClass->set_t_final(mbs_data->tsim);

	// remove interface
	controller_finish_interface(mbs_data);

	delete cppInterface;
}
