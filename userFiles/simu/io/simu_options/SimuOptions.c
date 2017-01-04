#include "SimuOptions.h"
#include "cmake_define.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/*! \brief initialize the options
 * 
 * \return requested structure initialized
 */
SimuOptions* init_SimuOptions()
{
	SimuOptions *options;

	options = (SimuOptions*) malloc(sizeof(SimuOptions));

	// flag to describe which COMAN model is used
	options->coman_model = coman_model_index();

	// simulation constraints
	options->simu_constraint = SIMU_2D_CONSTRAINT;

	// 1 to print comments, 0 otherwise
	#if defined(OPTI_BIOROB) || defined(OPTI_MPI)
		options->print = 0;
	#else
		options->print = 1;
	#endif

	// 1 to add friction on the COMAN joints (measured on the EPFL COMAN), 0 otherwise
	options->joints_friction = 1;
	if (options->simu_constraint == SIMU_Qq_MATCH_WANG)
	{
		options->joints_friction = 0;
	}

	// 1 to add noise on the torque readings, 0 otherwise
	options->torque_noise = 1;
	if (options->simu_constraint == SIMU_Qq_MATCH_WANG)
	{
		options->torque_noise = 0;
	}

	// speed reference for the optimizazion [m/s]
	options->opti_speed_ref = CMAKE_VAR_1;

	// 1 to optimize the robot speed, 0 otherwise
	options->speed_opti = 1;

	// 1 to minimise double suport, 0 otherwise
	options->opti_run = 0;

	// 1 to run an optimization, 0 otherwise
	#ifdef OPTI_RUN
		options->opti = 1;
	#else
		options->opti = 0;
	#endif

	// 1 to use the body contact model, 0 otherwise
	options->body_contact = 0;

	// GCM model
	options->gcm_model = MESH_GCM_MODEL;

	//must to be the same value in CtrlOptions (used to unactivate noise before switch)
	options->ctrl_two_parts = 1;
	if (options->simu_constraint == SIMU_Qq_MATCH_WANG)
	{
		options->ctrl_two_parts = 0;
	}

	return options;
}

/*! \brief release the memory of a SimuOptions structure
 * 
 * \param[out] options structure to release
 */
void free_SimuOptions(SimuOptions *options)
{
	free(options);
}

/*! \brief get coman_model index
 * 
 * \return COMAN model index
 */
int coman_model_index()
{
	if      (!strcmp(COMAN_VERSION, "init_feet"))  return INIT_FEET_COMAN;
	else if (!strcmp(COMAN_VERSION, "short_feet")) return SHORT_FEET_COMAN;
	else if (!strcmp(COMAN_VERSION, "long_feet"))  return LONG_FEET_COMAN;
	else if (!strcmp(COMAN_VERSION, "flex_feet"))  return FLEX_FEET_COMAN;
	else if (!strcmp(COMAN_VERSION, "short_feet_ball")) return SHORT_FEET_BALL_COMAN;
	else if (!strcmp(COMAN_VERSION, "spring_toe_short_feet")) return SPRING_TOE_SHORT_FEET_COMAN;
	else if (!strcmp(COMAN_VERSION, "spring_toe_feet")) return SPRING_TOE_FEET_COMAN;
	else
	{
		printf("Error: unknow COMAN mbs model: %s !\n", COMAN_VERSION);
		exit(EXIT_FAILURE);
	}
}
