
#include "AverageInc.hh"

/*! \brief constructor
 */
AverageInc::AverageInc()
{
	mean_count = 0;
	mean_value = 0.0;
}

/*! \brief destrcutor
 */
AverageInc::~AverageInc()
{
	
}

/*! \brief update with a new value
 * 
 * \param[in] new_value new value to add to this average
 */
void AverageInc::update(double new_value)
{
	double frac_mean;

	mean_count++;

	frac_mean = 1.0 / mean_count;

	mean_value = (1.0 - frac_mean) * mean_value + frac_mean * new_value;
}
