/*! 
 * \author Nicolas Van der Noot
 * \file AverageIncrement.hh
 * \brief AverageIncrement class
 */

#ifndef _AVERAGE_INCREMENT_HH_
#define _AVERAGE_INCREMENT_HH_

/*! \brief Compute the average by incrementing new values
 */
class AverageIncrement
{
	public:
		AverageIncrement();
		~AverageIncrement();

		void update(double new_value);

		/*! \brief get the mean value
		 * 
		 * \return the mean value
		 */
		inline double get()
		{
			return mean_value;
		}

		/*! \brief update with a new value and get the mean one
		 * 
		 * \param[in] new_value new value
		 * \return the mean value
		 */
		inline double update_and_get(double new_value)
		{
			update(new_value);

			return get();
		}

	private:
		int mean_count;    ///< counter
		double mean_value; ///< mean value
};

#endif
