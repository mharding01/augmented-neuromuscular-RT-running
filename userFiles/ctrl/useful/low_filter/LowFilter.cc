
#include "LowFilter.hh"

/*! \brief constructor
 * 
 * \param[in] tau time constant [s]
 * \param[in] x current input value
 * \param[in] cur_t current time [s]
 */
LowFilter::LowFilter(double tau, double x, double cur_t)
{
	y = x;

	last_t = cur_t;

	this->tau = tau;
}

/*! \brief destructor
 */
LowFilter::~LowFilter()
{

}

/*! \brief update with a new value
 * 
 * \param[in] x new value
 * \param[in] cur_t current time [s]
 *
 * Differential equation:
 *    tau * dy/dt = x - y
 *    tau * (y_i - y_j)/delta_t = x_i - y_i
 * 
 * Example:
 *    tau * dA/dt = S - A
 *
 * Numerical solution (with f = delta_t/tau):
 *    tau * (y_i - y_j)/delta_t = x_i - y_i
 *    (y_i - y_j)/f = x_i - y_i
 *    y_i * (1 + 1/f) = x_i + y_j/f
 *    y_i * (1 + f) = f * x_i + y_j
 *    y_i = f/(1 + f) * x_i + 1/(1 + f) * y_j
 *    y_i = f * frac * x_i + frac * y_j        (with frac = 1/(1 + f))
 */
void LowFilter::update(double x, double cur_t)
{
	double f = (cur_t - last_t) / tau;
	double frac = 1.0 / (1.0 + f);

	last_t = cur_t;

	y = f * frac * x + frac * y;
}
