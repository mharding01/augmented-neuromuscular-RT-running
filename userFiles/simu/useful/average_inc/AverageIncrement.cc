
#include "AverageIncrement.hh"

/*! \brief constructor
 */
AverageIncrement::AverageIncrement()
{
	mean_count = 0;

	mean_value = 0.0;
}

/*! \brief destructor
 */
AverageIncrement::~AverageIncrement()
{
	
}

/*! \brief update with a new value
 * 
 * \param[in] new_value new value
 */
void AverageIncrement::update(double new_value)
{
	double frac_mean;

	mean_count++;

	frac_mean = 1.0 / mean_count;

	mean_value = (1.0 - frac_mean) * mean_value + frac_mean * new_value;
}
