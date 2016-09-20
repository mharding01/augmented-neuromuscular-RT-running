/*! 
 * \author Nicolas Van der Noot
 * \file SimuOptions.hh
 * \brief simulation options
 */
#ifndef _SIMU_OPTIONS_H_
#define _SIMU_OPTIONS_H_

#include "user_realtime.h"

// COMAN version
enum {INIT_FEET_COMAN, SHORT_FEET_COMAN, LONG_FEET_COMAN,
	FLEX_FEET_COMAN, SHORT_FEET_BALL_COMAN, SPRING_TOE_SHORT_FEET_COMAN, SPRING_TOE_FEET_COMAN};

// simulation constraints
enum {SIMU_FREE_CONSTRAINT, SIMU_2D_CONSTRAINT, SIMU_SKYWALKER, SIMU_Qq_MATCH_WANG};

// Ground Contact Model
enum {NO_GCM_MODEL, MESH_GCM_MODEL, PRIM_GCM_MODEL};

/*! \brief simulation options structure
 */
typedef struct SimuOptions
{
	int coman_model;     ///< flag to describe which COMAN model is used
	int simu_constraint; ///< constraints in the simulation environment
	int print;           ///< 1 to print comments, 0 otherwise
	int joints_friction; ///< 1 to add friction on the COMAN joints (measured on the EPFL COMAN), 0 otherwise
	int torque_noise;    ///< 1 to add noise on the torque readings, 0 otherwise
	int speed_opti;      ///< 1 to optimize the robot speed, 0 otherwise
	int opti_run; 		 ///< 1 to minimise double support, 0 otherwise
	int opti;            ///< 1 to run an optimization, 0 otherwise
	int body_contact;    ///< 1 to use the body contact model, 0 otherwise
	int gcm_model;       ///< GCM used
	int ctrl_two_parts;  ///< 1 for ctrl in two parts : first with know results and after with opti's results, 0 otherwise

	double opti_speed_ref; ///< speed reference for the optimizazion [m/s]

} SimuOptions;

// functions prototypes
#ifdef __cplusplus
extern "C" {
#endif
	SimuOptions* init_SimuOptions();
	void free_SimuOptions(SimuOptions *options);
	int coman_model_index();
#ifdef __cplusplus
}
#endif

#endif
