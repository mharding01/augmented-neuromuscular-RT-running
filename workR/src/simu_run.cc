#include "def_main.hh"
#include "opti_gestion.hh"
#include "SimuOptions.h"
#include "cmake_define.h"
#include "cmake_config.h"
#include "user_IO.h"
#include "mbs_data.h"
#include "mbs_part.h"
#include "mbs_load_xml.h"
#include "mbs_dirdyn.h"
#include "realtime.h"
#include "mbs_set.h"
#include "user_all_id.h"
#include "random_ctrl.hh"

#include <iostream>
#include <vector>


/*! \brief run one simulation
 * 
 * \param[in] optiClass optimization class
 * \return fitness value
 */
double simu_run(OptiClass *optiClass)
{
	// variables declaration
	int current_t_sec, current_t_usec;
	double fitness, total_t_sec;
	std::vector<double> fitness_details;

	struct timeval timeValue;

	MbsData *mbs_data;
	MbsPart *mbs_part;
	MbsDirdyn *mbs_dirdyn;
	UserIO *uvs;

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*                   SEED FOR RANDOM                         *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	gettimeofday(&timeValue, NULL);
	srand(timeValue.tv_usec * timeValue.tv_sec);


	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*                     LOADING                               *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	mbs_data = mbs_load(PROJECT_SOURCE_DIR"/../dataR/coman_" COMAN_VERSION".mbs");

	simu_constraint_apply(mbs_data);


	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*              COORDINATE PARTITIONING                      *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	mbs_part = mbs_new_part(mbs_data);
	mbs_part->options->rowperm = 1;
	mbs_part->options->verbose = 0;
	mbs_run_part(mbs_part, mbs_data);
	
	mbs_delete_part(mbs_part);


	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*                   DIRECT DYNANMICS                        *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	// create new dirdyn module
	mbs_dirdyn = mbs_new_dirdyn(mbs_data);

	// dirdyn options (see documentations for additional options)
	mbs_dirdyn->options->dt0 = 125e-6;
	mbs_dirdyn->options->tf  = 70.0;

	// special time for evaluations
	#ifdef EVAL_RUN
		mbs_dirdyn->options->tf  = 30.0;
	#endif

	// results
	mbs_dirdyn->options->save2file = 0;
	mbs_dirdyn->options->saveperiod = 4;
	mbs_dirdyn->options->max_save_user = 12;
	mbs_dirdyn->options->respath = PROJECT_SOURCE_DIR"/../resultsR";
	mbs_dirdyn->options->animpath = PROJECT_SOURCE_DIR"/../animationR";

	// by default, use real-time modules
	mbs_dirdyn->options->realtime = 1;

	// by default, use accelred
	mbs_dirdyn->options->accelred = 0;

	if (!strcmp(COMAN_VERSION, "short_feet_ball"))
	{
		mbs_dirdyn->options->dopri5->flag_use = 1;
	}

	/*if (!strcmp(COMAN_VERSION, "spring_toe_feet"))
	{
		mbs_dirdyn->options->dopri5->flag_use = 1;
    	mbs_dirdyn->options->dopri5->rtoler = 1e-3;
    	mbs_dirdyn->options->dopri5->atoler = 1e-3;
    	mbs_dirdyn->options->dopri5->dt_max = 1e-3;
	}*/

	// special debug parameters
	#ifdef DEBUG_VERSION
	mbs_dirdyn->options->realtime = 0;
	mbs_dirdyn->options->tf = 0.1;
	#endif

	// special optimization parameters
	#if defined(EVAL_RUN) || defined(OPTI_RUN)
	mbs_dirdyn->options->realtime = 0;
	mbs_dirdyn->options->save2file = 0;
	#endif

	
	// get UserIO
	uvs = mbs_data->user_IO;

	// set speed reference
	uvs->optiClass = optiClass;
	uvs->options->opti_speed_ref = optiClass->get_v_ref();

	// run the simulation
	mbs_run_dirdyn(mbs_dirdyn, mbs_data);

	// extract fitness
	fitness = extract_OptiClass(mbs_data);
	fitness_details = extract_OptiClass_details(mbs_data);

	// delete dirdyn module
	mbs_delete_dirdyn(mbs_dirdyn, mbs_data);


	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*                   CLOSING OPERATIONS                      *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	// get total timing
	gettimeofday( &timeValue, NULL );
	current_t_sec  = (int)timeValue.tv_sec;
	current_t_usec = (int)timeValue.tv_usec;
	
	total_t_sec = (current_t_sec - uvs->init_t_sec) + 1.0e-6 * (current_t_usec - uvs->init_t_usec);

	// print statistics
	if (mbs_data->user_IO->options->print == 1)
	{
		std::cout << "\n-------------------------------" << std::endl;
		std::cout << "t start: " << mbs_data->t0 << std::endl;
		std::cout << "t end:   " << mbs_data->tf << std::endl;
		std::cout << "-------------------------------" << std::endl;
		std::cout << "t final: " << mbs_data->tsim << std::endl;
		std::cout << "executed in: " << total_t_sec << std::endl;
		std::cout << "-------------------------------" << std::endl;
		std::cout << "fitness: " << fitness <<  " ( ";
		for(unsigned int i=0; i<fitness_details.size(); i++)
			std::cout << fitness_details[i] << "  " ;
		std::cout <<  ")" << std::endl;
		std::cout << "-------------------------------\n" << std::endl;
	}

	// delete mbs_data module
	mbs_delete_data(mbs_data);

	return fitness;
}

/*! \brief get the mean fitness for nb_runs simulation runs
 * 
 * \param[in] optiClass optimization class
 * \param[in] nb_runs number of simulation runs
 * \return mean fitness value
 */
double mean_fitness(OptiClass *optiClass, int nb_runs)
{
	double fitness = 0.0;

	for(int i=0; i<nb_runs; i++)
	{
		fitness += simu_run(optiClass);
	}

	return (fitness / nb_runs);
}

/*! \brief get the mean fitness for simulation runs
 * 
 * \param[in] optiClass optimization class
 * \param[in] nb_runs number of simulation runs
 * \return mean fitness value
 */
double speed_range_fitness(OptiClass *optiClass)
{
	// parameters declaration
	int nb_runs;
	double delta_v, init_v;
	double fitness;

	std::vector<double> v_ref_tab;

	// variables initialization
	nb_runs = 17; 	// 1.1 - 1.9m/s
	init_v  = 1.1;
	delta_v = 0.05;

	fitness = 0.0;

	// fill the speed tabular
	for(int i=0; i<nb_runs; i++)
	{
		v_ref_tab.push_back(init_v + i*delta_v);
	}

	// run the simulations
	fitness = 0.0;

	for(unsigned int i=0; i<v_ref_tab.size(); i++)
	{
		optiClass->set_v_ref(v_ref_tab[i]);
		fitness += simu_run(optiClass);
	}
	
	return (fitness / v_ref_tab.size());
}

/*! \brief apply the simulation constraints (if any)
 *
 * \param[in,out] mbs_data Robotran structure
 */
void simu_constraint_apply(MbsData *mbs_data)
{
	SimuOptions *options;

	options = init_SimuOptions();

	switch (options->simu_constraint)
	{
		case SIMU_FREE_CONSTRAINT:
			break;

		case SIMU_2D_CONSTRAINT:
			mbs_set_qdriven(mbs_data, FJ_T2_Coman_id);
			mbs_set_qdriven(mbs_data, FJ_R1_Coman_id);
			mbs_set_qdriven(mbs_data, FJ_R3_Coman_id);
			break;

		case SIMU_SKYWALKER:
			mbs_set_qdriven(mbs_data, FJ_T1_Coman_id);
			mbs_set_qdriven(mbs_data, FJ_T2_Coman_id);
			mbs_set_qdriven(mbs_data, FJ_T3_Coman_id);
			mbs_set_qdriven(mbs_data, FJ_R1_Coman_id);
			mbs_set_qdriven(mbs_data, FJ_R2_Coman_id);
			mbs_set_qdriven(mbs_data, FJ_R3_Coman_id);

			mbs_data->q[FJ_T3_Coman_id] = 0.75;
			//mbs_data->q[FJ_R2_Coman_id] = M_PI; //tete à l'envers
			break;

		case SIMU_Qq_MATCH_WANG:
			// 2D constraint
			mbs_set_qdriven(mbs_data, FJ_T2_Coman_id);
			mbs_set_qdriven(mbs_data, FJ_R1_Coman_id);
			mbs_set_qdriven(mbs_data, FJ_R3_Coman_id);
			// no contact with ground
			mbs_set_qdriven(mbs_data, FJ_T3_Coman_id);
			mbs_data->q[FJ_T3_Coman_id] = 0.75;
			//right leg
			mbs_set_qdriven(mbs_data, RHipSag_id);
			mbs_set_qdriven(mbs_data, RKneeSag_id);
			mbs_set_qdriven(mbs_data, RAnkSag_id);
			//left leg
			mbs_set_qdriven(mbs_data, LHipSag_id);
			mbs_set_qdriven(mbs_data, LKneeSag_id);
			mbs_set_qdriven(mbs_data, LAnkSag_id);
			//forward trans and rot
			mbs_set_qdriven(mbs_data, FJ_R2_Coman_id);
			mbs_set_qdriven(mbs_data, FJ_T1_Coman_id);
			//remove gravity
			mbs_data->g[3] = 0.0;
			break;

		default:
			break;
	}

	free(options);
}
