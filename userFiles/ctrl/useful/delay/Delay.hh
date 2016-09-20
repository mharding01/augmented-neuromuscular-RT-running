/*! 
 * \author Nicolas Van der Noot
 * \file Delay.hh
 * \brief Delay class
 */

#ifndef _DELAY_HH_
#define _DELAY_HH_

/*! \brief Returning a signal with a time delay
 */
class Delay
{
	public:
		Delay(double t_delay, int size_vec);
		~Delay();

		void new_value_time(double new_t, double new_val);
		double get_value_delay(double cur_t);

		/*! \brief add a new value to delay and get the current delayed value
		 * 
		 * \param[in] cur_t current time (time of the new value) [s]
		 * \param[in] cur_val current value (new one) to delay
		 * \return current delayed value
		 */
		double update_and_get(double cur_t, double cur_val)
		{
			new_value_time(cur_t, cur_val);

			return get_value_delay(cur_t);
		}

	private:
		int index;    ///< index to go over the tabular
		int size_vec; ///< size of the tabular

		double t_delay; ///< time of the delay to apply [s]

		double *vec_val; ///< vector with all the values
		double *vec_t;   ///< vector with all the time
};

#endif