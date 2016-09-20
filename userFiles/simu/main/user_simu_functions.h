/*! 
 * \author Nicolas Van der Noot
 * \file user_simu_functions.h
 * \brief Simulation functions used in the user files
 */

#ifndef _USER_SIMU_FUNCTIONS_H_
#define _USER_SIMU_FUNCTIONS_H_

#include "mbs_data.h"
#include "controller_io.hh"

#ifdef __cplusplus
extern "C" {
#endif

	// simulation init
	void simu_init(MbsData *mbs_data);
	void simu_init_contact(MbsData *mbs_data);

	// simulation loop
	void simu_controller_loop(MbsData *mbs_data);

	// simulation close
	void simu_finish(MbsData *mbs_data);

	// GCM (Ground Contact Model)
	double trapezoidal_ground(double x, double x1, double x2, double x3, double x4, double y_max);
	double get_ground_height(double x, double y, double tsim, MbsData *mbs_data);
	void ground_mesh_model(double PxF[4], double RxF[4][4],
						   double VxF[4], double OMxF[4],
						   MbsData *mbs_data, double tsim,
						   int ixF, double *dxF, double *SWr);
	void init_GCM(MbsData *mbs_data);
	double z_left_foot(double x, double y);
	double z_right_foot(double x, double y);

#ifdef __cplusplus
}
#endif

#endif
