
#include "RunningAverage.hh"
#include <stdlib.h>

/*! \brief constructor
 * 
 * \param[in] size_vec size of the vector
 * \param[in] init_value initial value to give to the full vector
 */
RunningAverage::RunningAverage(int size_vec, double init_value)
{
	this->size_vec = size_vec;

	index = 0;

	vec = (double*) malloc(size_vec*sizeof(double));

	for(int i=0; i<size_vec; i++)
	{
		vec[i] = init_value;
	}

	sum = init_value * size_vec;
}

/*! \brief destructor
 */
RunningAverage::~RunningAverage()
{
	free(vec);
}

/*! \brief add a new value to the running average
 * 
 * \param[in] new_val new value to add
 */
void RunningAverage::new_value(double new_val)
{
	sum -= vec[index];
	sum += new_val;

	vec[index] = new_val;

	index++;

	if (index >= size_vec)
	{
		index = 0;
	}
}
