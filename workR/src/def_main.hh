/*! 
 * \author Nicolas Van der Noot
 * \file def_main.hh
 * \brief definitions for the main files
 */

#ifndef _DEF_MAIN_HH_
#define _DEF_MAIN_HH_

#include "OptiClass.hh"
#include "mbs_data.h"

double simu_run(OptiClass *optiClass);
double mean_fitness(OptiClass *optiClass, int nb_runs);
double speed_range_fitness(OptiClass *optiClass);
void simu_constraint_apply(MbsData *mbs_data);

#endif
