/*! 
 * \author Nicolas Van der Noot
 * \file compute_forces.h
 * \brief Function to make the interface between C++ and C for the external forces (GCM)
 */

#ifndef _COMPUTE_FORCES_H_
#define _COMPUTE_FORCES_H_

#include "mbs_data.h"

void compute_gcm(double F[3], double T[3], double PxF[4], double RxF[4][4], 
		double VxF[4], double OMxF[4], MbsData* mbs_data, int index);

#ifdef __cplusplus
extern "C" {
#endif
	void compute_ext_forces(double *SWr, double PxF[4], double RxF[4][4], double VxF[4],
		double OMxF[4], double AxF[4], double OMPxF[4], MbsData *mbs_data, double tsim, int ixF);
#ifdef __cplusplus
}
#endif

#endif
