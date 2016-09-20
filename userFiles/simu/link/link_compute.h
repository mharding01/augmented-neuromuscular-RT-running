/*! 
 * \author Nicolas Van der Noot
 * \file link_compute.h
 * \brief links computation (with C interface)
 */

#ifndef _LINK_COMPUTE_H_
#define _LINK_COMPUTE_H_

#include "mbs_data.h"

#ifdef __cplusplus
extern "C" {
#endif
	double link_compute(double Z, double Zd, MbsData *mbs_data, double tsim, int ilnk);
#ifdef __cplusplus
}
#endif

#endif
