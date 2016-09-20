/*! 
 * \author Nicolas Van der Noot
 * \file opti_params.hh
 * \brief optimization parameters and functions prototypes
 */

#ifndef _OPTI_PARAMS_HH_
#define _OPTI_PARAMS_HH_

#include <vector>

double convert_to_optiParams(std::vector<double> optiNorms);
int get_nb_optiParams();
double new_norm_start(int index);

#endif
