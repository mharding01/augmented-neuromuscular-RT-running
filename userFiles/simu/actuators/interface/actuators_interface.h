/*! 
 * \author Nicolas Van der Noot
 * \file actuator_interface.h
 * \brief interface with C code to compute the actuators dynamics
 */

#ifndef _ACTUATORS_INTERFACE_H_
#define _ACTUATORS_INTERFACE_H_

#include "mbs_data.h"

#ifdef __cplusplus
extern "C" {
#endif
	void compute_actuators_derivative(MbsData* mbs_data, double *ux, double *uxd);
	void compute_actuators_torque(MbsData* mbs_data, double *ux, double *uxd, double *Qq);
#ifdef __cplusplus
}
#endif

#endif
