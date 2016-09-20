
#include "simu_functions.hh"

#include <cstdlib>

/*! \brief return the interpolation for the abscissa x (x1, x2, y1 and y2 define the line)
 * 
 * \param[in] x x value
 * \param[in] x1 x min
 * \param[in] x2 x max
 * \param[in] y1 y min
 * \param[in] y2 y max
 * \return requested interpolation
 */
double linear_interp(double x, double x1, double x2, double y1, double y2)
{
    double slope;

    if (x1 == x2)
    {
        return y1;
    }

    slope = (y2 - y1) / (x2 - x1);

    return y1 + slope * (x - x1);
}

/*! \brief generate random number in [0 ; 1]
 * 
 * \return requested random number [0 ; 1]
 */
double rnd_simu(void)
{
    return ((double)rand())/((double)RAND_MAX);
}
