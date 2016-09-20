
#include "Oscillators.hh"
#include <iostream>

inline double pos(double x){ return (x > 0.0) ?  x : 0.0; }

/*! \brief constructor
 * 
 * \param[in] nb_neurons number of neurons
 * \param[in] cur_t current time [s]
 */
Oscillators::Oscillators(int nb_neurons, int cur_t)
{
	for(int i=0; i<nb_neurons; i++)
	{
		x.push_back(0.0);
		xd.push_back(0.0);
		u.push_back(0.0);
	}

	last_t = cur_t;
}

/*! \brief destructor
 */
Oscillators::~Oscillators()
{

}

/*! \brief integarte (with Euler explicit) the neurons state
 * 
 * \param[in] cur_t current time
 */
void Oscillators::integrate_neurons(double cur_t)
{
	for(unsigned int i=0; i<x.size(); i++)
	{
		x[i] += xd[i] * (cur_t - last_t);
	}

	last_t = cur_t;
}

/*! \brief get x positive value
 * 
 * \param[in] index index of the neuron
 * \return requested value
 */
double Oscillators::get_x_pos(int index)
{
	return pos(x[index]);
}

/*! \brief get y positive value
 * 
 * \param[in] index index of the output
 * \return requested value
 */
double Oscillators::get_y_pos(int index)
{
	return pos(y[index]);
}
