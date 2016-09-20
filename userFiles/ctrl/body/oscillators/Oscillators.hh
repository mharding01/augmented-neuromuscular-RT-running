/*! 
 * \author Nicolas Van der Noot
 * \file Oscillators.hh
 * \brief Oscillators class
 */

#ifndef _OSCILLATORS_HH_
#define _OSCILLATORS_HH_

#include <vector>

/*! \brief Oscillators (CPG) used to provide the feedforard signals
 */
class Oscillators
{
	public:
		Oscillators(int nb_neurons, int cur_t);
		virtual ~Oscillators();

		void integrate_neurons(double cur_t);
		double get_x_pos(int index);
		double get_y_pos(int index);

		virtual void update(double cur_t) = 0;

		/*! \brief get state, according to index
		 * 
		 * \param[in] index index of the neuron
		 * \return requested state
		 */
		double get_x(int index)
		{
			return x[index];
		}

	protected:
		std::vector<double> x;  ///< state of the neurons
		std::vector<double> xd; ///< state derivative of the neurons
		std::vector<double> u;  ///< excitation of teh neurons
		std::vector<double> y;  ///< outputs of the oscillators

		double last_t; ///< last update time for the oscillators
};

#endif
