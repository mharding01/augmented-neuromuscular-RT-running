
#include "ctrl_functions.hh"
#include <cstdlib>

/*! \brief return an angle in the ]-pi ; pi] interval
 * 
 * \param[in] angle input angle
 * \return bounded angle
 */
double limit_angle(double angle)
{
	while(angle <= -M_PI)
	{
		angle += 2.0 * M_PI;
	}
	while(angle > M_PI)
	{
		angle -= 2.0 * M_PI;
	}

	return angle;
}

/*! \brief return 'value' bounded in the [this_min; this_max] range
 * 
 * \param[in] value value to bound
 * \param[in] this_min minimal bound value
 * \param[in] this_max maximal bound value
 * \return requested value
 */
double limit_range(double value, double this_min, double this_max)
{
	if (value < this_min)
	{
		return this_min;
	}
	else if (value > this_max)
	{
		return this_max;
	}
	else
	{
		return value;
	}
}

/*! \brief linear interpolation: evaluation of y(t) with y(t_0) = y_0, y(t_end) = y_end
 * 
 * \param[in] y_0 initial bound value
 * \param[in] y_end final bound value
 * \param[in] t_0 initial bound time [s]
 * \param[in] t_end final bound time [s]
 * \param[in] t current time [s]
 * \return requested interpolation
 */
double linear_interpol(double y_0, double y_end, double t_0, double t_end, double t)
{
	if (t < t_0)
	{
		return y_0;
	}
	else if (t < t_end)
	{
		return y_0 + (t - t_0) / (t_end - t_0) * (y_end - y_0);
	}
	else
	{
		return y_end;
	}
}

/*! \brief generate random number in [0;1]
 * 
 * \return requested random parameter in [0;1]
 */
double rnd(void)
{
	return ((double)rand())/((double)RAND_MAX);
}

/*! \brief compute the power of a value (with only integers for the exponent, as opposed to 'pow')
 * 
 * \param[in] value base
 * \param[in] expo exponent (integer)
 * \return computed value^expo */
double powInt(double value, int expo)
{
	int i;
	double cur_value;
	
	if (expo < 1)
	{
		return 1.0;
	}
	else
	{
		cur_value = value;
		
		for (i=1; i<expo; i++)
		{
			cur_value *= value;
		}
		return cur_value;
	}
}
