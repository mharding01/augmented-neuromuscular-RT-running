/*! 
 * \author Nicolas Van der Noot
 * \file ctrl_functions.hh
 * \brief Useful functions for the controller
 */

#ifndef _CTR_FUNCTIONS_HH_
#define _CTR_FUNCTIONS_HH_

#include <cmath>

double limit_angle(double angle);
double limit_range(double value, double this_min, double this_max);
double linear_interpol(double y_0, double y_end, double t_0, double t_end, double t);
double rnd(void);
double powInt(double value, int expo);

#endif

