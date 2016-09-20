/*! 
 * \author Nicolas Van der Noot
 * \file simu_functions.hh
 * \brief Useful functions for the simulation
 */

#ifndef _SIMU_FUNCTIONS_HH_
#define _SIMU_FUNCTIONS_HH_

double linear_interp(double x, double x1, double x2, double y1, double y2);
double rnd_simu(void);

/*! \brief get min value
 * 
 * \param[in] a first value
 * \param[in] b second value
 * \return minimal value
 */
inline double get_min_simu(double a, double b) { return (a < b) ? a : b; }

/*! \brief get max value
 * 
 * \param[in] a first value
 * \param[in] b second value
 * \return maximal value
 */
inline double get_max_simu(double a, double b) { return (a < b) ? b : a; }

#endif

