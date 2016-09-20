
#include "Delay.hh"
#include <stdlib.h>

/*! \brief constructor
 * 
 * \param[in] t_delay time of the delay to apply on the signal [s]
 * \param[in] size_vec size of the tabular
 */
Delay::Delay(double t_delay, int size_vec)
{
	index = 0;

	this->t_delay = t_delay;

	this->size_vec = size_vec;

	vec_t   = (double*) malloc(size_vec*sizeof(double));
	vec_val = (double*) malloc(size_vec*sizeof(double));

	for(int i=0; i<size_vec; i++)
	{
		vec_t[i]   = 0.0;
		vec_val[i] = 0.0;
	}
}

/*! \brief destructor
 */
Delay::~Delay()
{
	free(vec_t);
	free(vec_val);
}

/*! \brief add a new value to delay
 * 
 * \param[in] new_t time of the new value (normally, current time) [s]
 * \param[in] new_val new value to delay
 */
void Delay::new_value_time(double new_t, double new_val)
{
	vec_t[index]   = new_t;
	vec_val[index] = new_val;

	index++;

	if (index >= size_vec)
	{
		index = 0;
	}
}

/*! \brief get the current value given the delay
 * 
 * \param[in] cur_t current time [s]
 * \return current delayed value
 */
double Delay::get_value_delay(double cur_t)
{
	int last_index;

	// index of the first element added (the oldest one)
	int cur_index = index;

	// requested time
	double request_t = cur_t - t_delay;

	// loop on all the lements starting with the oldest one
	for(int i=0; i<size_vec; i++)
	{
		if (vec_t[cur_index] >= request_t)
		{
			return vec_val[cur_index];
		}

		cur_index++;

		if (cur_index >= size_vec)
		{
			cur_index = 0;
		}
	}

	// index of the more recent element added
	last_index = index-1;

	if (last_index < 0)
	{
		last_index = size_vec-1;
	}

	// return last element added
	return vec_val[last_index];
}
