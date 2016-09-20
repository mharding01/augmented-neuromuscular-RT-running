/*! 
 * \author Nicolas Van der Noot
 * \file opti_gestion.hh
 * \brief gestion of the optimization modules
 */

#ifndef _OPTI_GESTION_HH_
#define _OPTI_GESTION_HH_

#include "OptiClass.hh"
#include "mbs_data.h"

OptiClass* get_rnd_OptiClass();
double extract_OptiClass(MbsData *mbs_data);
std::vector<double> extract_OptiClass_details(MbsData *mbs_data);

#endif
